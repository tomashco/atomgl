/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include <atom.h>
#include <bif.h>
#include <context.h>
#include <debug.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <module.h>
#include <port.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>
#include <trace.h>

#include "backlight_gpio.h"
#include "display_common.h"
#include "display_items.h"
#include "spi_display.h"

// if needed it can be lowered to 27000000
#define SPI_CLOCK_HZ 27000000
#define SPI_MODE 0

#define CHAR_WIDTH 8

#define GC9A01_TFTWIDTH 240
#define GC9A01_TFTHEIGHT 240

#define GC9A01_SLPIN 0x10
#define GC9A01_SLPOUT 0x11
#define GC9A01_NORON 0x13
#define GC9A01_INVOFF 0x20
#define GC9A01_INVON 0x21
#define GC9A01_DISPOFF 0x28
#define GC9A01_DISPON 0x29
#define GC9A01_CASET 0x2A
#define GC9A01_RASET 0x2B
#define GC9A01_RAMWR 0x2C
#define GC9A01_MADCTL 0x36
#define GC9A01_COLMOD 0x3A

// rotation
#define GC9A01_MADCTL_MY 0x80
#define GC9A01_MADCTL_MX 0x40
#define GC9A01_MADCTL_MV 0x20
#define GC9A01_MADCTL_ML 0x10
#define GC9A01_MADCTL_RGB 0x00

#define TFT_MAD_RGB 0x00
#define TFT_MAD_BGR 0x08
#define TFT_MAD_COLOR_ORDER TFT_MAD_RGB

#define GC9A01_RST_DELAY 120 // ms
#define GC9A01_SLPIN_DELAY 120 // ms
#define GC9A01_SLPOUT_DELAY 120 // ms

#include "font.c"

static const char *TAG = "gc9a01_display_driver";

static void send_message(term pid, term message, GlobalContext *global);

static inline void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct SPI
{
    struct SPIDisplay spi_disp;
    int dc_gpio;
    int reset_gpio;

    avm_int_t rotation;

    Context *ctx;
};

struct Screen
{
    int w;
    int h;
    uint16_t *pixels;
    uint16_t *pixels_out;
};

static struct Screen *screen;

static inline uint16_t alpha_blend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t) ((result >> 16) | result);
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

static inline uint16_t rgba8888_color_to_rgb565(struct Screen *s, uint32_t color)
{
    uint8_t r = color >> 24;
    uint8_t g = (color >> 16) & 0xFF;
    uint8_t b = (color >> 8) & 0xFF;

    return (((uint16_t) (r >> 3)) << 11) | (((uint16_t) (g >> 2)) << 5) | ((uint16_t) b >> 3);
}

static inline uint16_t rgb565_color_to_surface(struct Screen *s, uint16_t color16)
{
    return (uint16_t) SPI_SWAP_DATA_TX(color16, 16);
}

static inline uint16_t uint32_color_to_surface(struct Screen *s, uint32_t color)
{
    uint16_t color16 = rgba8888_color_to_rgb565(s, color);
    return rgb565_color_to_surface(s, color16);
}

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static QueueHandle_t display_messages_queue;

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);
static void display_init_gc9a01(struct SPI *spi);
static void draw_test_pattern(struct SPI *spi);

static inline void writedata(struct SPI *spi, uint8_t data)
{
    ESP_LOGD(TAG, "Writing data: 0x%02X", data);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 8, data);
    spi_device_release_bus(spi->spi_disp.handle);
}

static inline void writecommand(struct SPI *spi, uint8_t command)
{
    ESP_LOGD(TAG, "Writing command: 0x%02X", command);
    gpio_set_level(spi->dc_gpio, 0);
    writedata(spi, command);
    gpio_set_level(spi->dc_gpio, 1);
}

static inline void set_screen_paint_area(struct SPI *spi, int x, int y, int width, int height)
{
    writecommand(spi, GC9A01_CASET);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(spi->spi_disp.handle);

    writecommand(spi, GC9A01_RASET);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(spi->spi_disp.handle);
}

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(screen, img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(screen, img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(screen, blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_scaled_cropped_img_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data_with_size.pix;

    int drawn_pixels = 0;

    int y_scale = item->y_scale;
    int x_scale = item->x_scale;
    int img_width = item->data.image_data_with_size.width;

    int source_x = item->source_x;
    int source_y = item->source_y;

    uint32_t *pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + ((xpos - x) / x_scale);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (source_x + (width / x_scale) > img_width) {
        width = (img_width - source_x) * x_scale;
    }

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(screen, img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(screen, img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(screen, blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        // TODO: optimize here
        pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + (j / x_scale);
    }

    return drawn_pixels;
}

static int draw_rect_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;
    uint16_t color = uint32_color_to_surface(screen, item->brcolor);

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        pixmem16[drawn_pixels] = color;
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_text_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    uint16_t fgcolor = uint32_color_to_surface(screen, item->data.text_data.fgcolor);
    uint16_t bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    uint16_t *pixmem32 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        int char_index = j / CHAR_WIDTH;
        char c = text[char_index];
        unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;

        unsigned char row = glyph[ypos - y];

        bool opaque;
        int k = j % CHAR_WIDTH;
        if (row & (1 << (7 - k))) {
            opaque = true;
        } else {
            opaque = false;
        }

        if (opaque) {
            pixmem32[drawn_pixels] = fgcolor;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = screen->w;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                drawn_pixels = draw_image_x(xpos, ypos, max_line_len, item);
                break;

            case Rect:
                drawn_pixels = draw_rect_x(xpos, ypos, max_line_len, item);
                break;

            case ScaledCroppedImage:
                drawn_pixels = draw_scaled_cropped_img_x(xpos, ypos, max_line_len, item);
                break;

            case Text:
                drawn_pixels = draw_text_x(xpos, ypos, max_line_len, item);
                break;
            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
}

static void do_update(Context *ctx, term display_list)
{
    ESP_LOGI(TAG, "Starting display update");

    int proper;
    int len = term_list_length(display_list, &proper);
    ESP_LOGI(TAG, "Display list length: %d", len);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);
    if (!items) {
        ESP_LOGE(TAG, "Failed to allocate display items");
        return;
    }

    term t = display_list;

    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }
    int screen_width = screen->w;
    int screen_height = screen->h;
    struct SPI *spi = ctx->platform_data;

    set_screen_paint_area(spi, 0, 0, screen_width, screen_height);
    writecommand(spi, GC9A01_RAMWR);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (ypos == 0 || ypos == screen_height - 1) {
            ESP_LOGI(TAG, "Drawing line %d of %d", ypos, screen_height);
        }

        // Clear the line buffer
        memset(screen->pixels, 0, screen->w * sizeof(uint16_t));

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        if (transaction_in_progress) {
            spi_transaction_t *trans;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        // Send the line
        spi_display_dmawrite(&spi->spi_disp, screen_width * sizeof(uint16_t), screen->pixels);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->spi_disp.handle);
    destroy_items(items, len);

    ESP_LOGI(TAG, "Display update completed");
}

static void draw_buffer(struct SPI *spi, int x, int y, int width, int height, const void *imgdata)
{
    ESP_LOGI(TAG, "Drawing buffer at x:%d y:%d w:%d h:%d", x, y, width, height);

    const uint16_t *data = imgdata;

    set_screen_paint_area(spi, x, y, width, height);

    writecommand(spi, GC9A01_RAMWR);

    int dest_size = width * height;
    int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;

    int chunks = dest_size / 1024;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    for (int i = 0; i < chunks; i++) {
        const uint16_t *data_b = data + 1024 * i;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->spi_disp, buf_pixel_size * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        const uint16_t *data_b = data + chunks * 1024;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }
    spi_device_release_bus(spi->spi_disp.handle);

    free(tmpbuf);

    ESP_LOGI(TAG, "Buffer drawing completed");
}

static void process_message(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        fprintf(stderr, "Received invalid message.");
        AVM_ABORT();
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        AVM_ABORT();
    }
    term cmd = term_get_tuple_element(req, 0);

    struct SPI *spi = ctx->platform_data;

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

    } else if (cmd == context_make_atom(ctx, "\xB"
                                             "draw_buffer")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        unsigned long addr_low = term_to_int(term_get_tuple_element(req, 5));
        unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));

        const void *data = (const void *) ((addr_low | (addr_high << 16)));

        draw_buffer(spi, x, y, width, height, data);

        // draw_buffer is a kind of cast, no need to reply
        return;

    } else {
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void process_messages(void *arg)
{
    struct SPI *args = arg;

    while (true) {
        Message *message;
        xQueueReceive(display_messages_queue, &message, portMAX_DELAY);
        process_message(message, args->ctx);

        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
    }
}

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx)
{
    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    xQueueSend(display_messages_queue, &msg, 1);

    return NativeContinue;
}

static void set_rotation(struct SPI *spi, int rotation)
{
    uint8_t madctl = TFT_MAD_COLOR_ORDER;

    switch (rotation) {
        case 3:
            madctl |= GC9A01_MADCTL_MX | GC9A01_MADCTL_MY | GC9A01_MADCTL_MV;
            break;
        case 2:
            madctl |= GC9A01_MADCTL_MX | GC9A01_MADCTL_MY;
            break;
        case 1:
            madctl |= GC9A01_MADCTL_MV;
            break;
        case 0:
        default:
            break;
    }

    writecommand(spi, GC9A01_MADCTL);
    writedata(spi, madctl);
}

Context *gc9a01_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void send_message(term pid, term message, GlobalContext *global)
{
    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(global, local_process_id, message);
}

static void test_display_with_backlight_config(struct SPI *spi, struct BacklightGPIOConfig *config, const char *test_name)
{
    ESP_LOGI(TAG, "=== Starting test: %s ===", test_name);
    ESP_LOGI(TAG, "Backlight config - GPIO: %d, Active High: %d, Enabled: %d",
        config->gpio, config->active_high, config->enabled);

    backlight_gpio_init(config);
    delay(100); // Give some time for backlight to stabilize

    draw_test_pattern(spi);

    ESP_LOGI(TAG, "=== Completed test: %s ===\n", test_name);
    delay(2000); // Pause between tests
}

static void display_init(Context *ctx, term opts)
{
    ESP_LOGI(TAG, "Starting display initialization...");
    ESP_LOGI(TAG, "Free DMA memory: %d", heap_caps_get_free_size(MALLOC_CAP_DMA));
    ESP_LOGI(TAG, "Largest free DMA block: %d", heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

    screen = malloc(sizeof(struct Screen));
    if (!screen) {
        ESP_LOGE(TAG, "Failed to allocate screen structure");
        return;
    }
    ESP_LOGI(TAG, "Screen structure allocated successfully");

    screen->w = GC9A01_TFTWIDTH;
    screen->h = GC9A01_TFTHEIGHT;

    // Allocate just one line of pixels
    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!screen->pixels) {
        ESP_LOGE(TAG, "Failed to allocate pixels buffer");
        free(screen);
        screen = NULL;
        return;
    }
    ESP_LOGI(TAG, "Pixel buffer allocated successfully");

    // We don't need pixels_out anymore since we're doing line-by-line
    screen->pixels_out = NULL;

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = SPI_MODE;
    spi_config.clock_speed_hz = SPI_CLOCK_HZ;
    spi_config.bit_lsb_first = false; // MSB first
    ESP_LOGI(TAG, "SPI Config - Mode: %d, Clock: %d Hz", spi_config.mode, spi_config.clock_speed_hz);
    spi_display_parse_config(&spi_config, opts, ctx->global);

    // Log parsed SPI config
    ESP_LOGI(TAG, "Parsed SPI Config - MODE: %d, CLK: %d, CS: %d",
        spi_config.mode,
        spi_config.clock_speed_hz,
        spi_config.cs_gpio);

    spi_display_init(&spi->spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);

    if (!ok) {
        ESP_LOGE(TAG, "Failed init: invalid GPIO configuration");
        free(spi);
        return;
    }

    ESP_LOGI(TAG, "Starting GPIO initialization - DC GPIO: %d, Reset GPIO: %d", spi->dc_gpio, spi->reset_gpio);

    // Reset sequence
    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    delay(GC9A01_RST_DELAY);
    gpio_set_level(spi->reset_gpio, 0);
    delay(GC9A01_RST_DELAY);
    gpio_set_level(spi->reset_gpio, 1);
    delay(GC9A01_RST_DELAY);
    ESP_LOGI(TAG, "Reset sequence completed");

    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);

    // Initialize display
    ESP_LOGI(TAG, "Starting GC9A01 initialization...");
    display_init_gc9a01(spi);
    ESP_LOGI(TAG, "GC9A01 initialization completed");

    // Get the base backlight configuration from options
    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);

    // Log the received options
    ESP_LOGI(TAG, "Display options received:");
    term backlight_pin = interop_kv_get_value_default(opts, ATOM_STR("\x9", "backlight"), term_invalid_term(), ctx->global);
    if (backlight_pin != term_invalid_term()) {
        ESP_LOGI(TAG, "Backlight pin configured in options: %d", term_to_int(backlight_pin));
    } else {
        ESP_LOGI(TAG, "No backlight pin specified in options");
    }

    term backlight_active = interop_kv_get_value_default(opts, ATOM_STR("\xF", "backlight_active"), term_invalid_term(), ctx->global);
    if (backlight_active != term_invalid_term()) {
        ESP_LOGI(TAG, "Backlight active mode specified: %s",
            backlight_active == context_make_atom(ctx, "\x4"
                                                       "high")
                ? "high"
                : "low");
    } else {
        ESP_LOGI(TAG, "No backlight_active option specified, defaulting to active high");
    }

    term backlight_enabled = interop_kv_get_value_default(opts, ATOM_STR("\x10", "backlight_enabled"), term_invalid_term(), ctx->global);
    if (backlight_enabled != term_invalid_term()) {
        ESP_LOGI(TAG, "Backlight enabled setting: %s",
            backlight_enabled == TRUE_ATOM ? "true" : "false");
    } else {
        ESP_LOGI(TAG, "No backlight_enabled option specified, defaulting to enabled");
    }

    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);

    // test_display_with_backlight_config(spi, &backlight_config, "Active High, Enabled");

    backlight_gpio_init(&backlight_config);
    ESP_LOGI(TAG, "Restored original backlight configuration");

    // // Draw test pattern
    // draw_test_pattern(spi);
    ESP_LOGI(TAG, "Test pattern drawn");

    ctx->platform_data = spi;
    spi->ctx = ctx;

    display_messages_queue = xQueueCreate(32, sizeof(Message *));
    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}

static void display_init_gc9a01(struct SPI *spi)
{
    ESP_LOGI(TAG, "Sending GC9A01 initialization commands...");

    // Initial delay after reset
    delay(120);

    writecommand(spi, 0xEF);
    writecommand(spi, 0xEB);
    writedata(spi, 0x14);

    writecommand(spi, 0xFE);
    writecommand(spi, 0xEF);

    writecommand(spi, 0xEB);
    writedata(spi, 0x14);

    writecommand(spi, 0x84);
    writedata(spi, 0x40);

    writecommand(spi, 0x85);
    writedata(spi, 0xFF);

    writecommand(spi, 0x86);
    writedata(spi, 0xFF);

    writecommand(spi, 0x87);
    writedata(spi, 0xFF);

    writecommand(spi, 0x88);
    writedata(spi, 0x0A);

    writecommand(spi, 0x89);
    writedata(spi, 0x21);

    writecommand(spi, 0x8A);
    writedata(spi, 0x00);

    writecommand(spi, 0x8B);
    writedata(spi, 0x80);

    writecommand(spi, 0x8C);
    writedata(spi, 0x01);

    writecommand(spi, 0x8D);
    writedata(spi, 0x01);

    writecommand(spi, 0x8E);
    writedata(spi, 0xFF);

    writecommand(spi, 0x8F);
    writedata(spi, 0xFF);

    // Display Function Control
    writecommand(spi, 0xB6);
    writedata(spi, 0x00);
    writedata(spi, 0x20); // Changed: was 0x00

    writecommand(spi, 0x36);
    writedata(spi, 0x08); // Changed: different orientation setting

    writecommand(spi, 0x3A);
    writedata(spi, 0x05); // 16-bit color

    // Positive Voltage Gamma Control
    writecommand(spi, 0xE0);
    writedata(spi, 0xD0);
    writedata(spi, 0x08);
    writedata(spi, 0x11);
    writedata(spi, 0x08);
    writedata(spi, 0x0C);
    writedata(spi, 0x15);
    writedata(spi, 0x39);
    writedata(spi, 0x33);
    writedata(spi, 0x50);
    writedata(spi, 0x36);
    writedata(spi, 0x13);
    writedata(spi, 0x14);
    writedata(spi, 0x29);
    writedata(spi, 0x2D);

    // Negative Voltage Gamma Control
    writecommand(spi, 0xE1);
    writedata(spi, 0xD0);
    writedata(spi, 0x08);
    writedata(spi, 0x10);
    writedata(spi, 0x08);
    writedata(spi, 0x06);
    writedata(spi, 0x06);
    writedata(spi, 0x39);
    writedata(spi, 0x44);
    writedata(spi, 0x51);
    writedata(spi, 0x0B);
    writedata(spi, 0x16);
    writedata(spi, 0x14);
    writedata(spi, 0x2F);
    writedata(spi, 0x31);

    // Sleep Out
    writecommand(spi, 0x11);
    delay(120);

    // Display ON
    writecommand(spi, 0x29);
    delay(20);
}

static void draw_test_pattern(struct SPI *spi)
{
    ESP_LOGI(TAG, "Drawing test pattern - random pixels");

    set_screen_paint_area(spi, 0, 0, GC9A01_TFTWIDTH, GC9A01_TFTHEIGHT);
    writecommand(spi, GC9A01_RAMWR);

    // Create a buffer for one pixel
    uint16_t *pixel = heap_caps_malloc(sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!pixel) {
        ESP_LOGE(TAG, "Failed to allocate test pattern buffer");
        return;
    }

    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);

    // Draw random colored pixels one by one
    for (int y = 0; y < GC9A01_TFTHEIGHT; y++) {
        for (int x = 0; x < GC9A01_TFTWIDTH; x++) {
            // Generate random RGB565 color
            uint16_t color = (rand() & 0xF800) | // Random red
                (rand() & 0x07E0) | // Random green
                (rand() & 0x001F); // Random blue

            pixel[0] = SPI_SWAP_DATA_TX(color, 16);

            // Write single pixel
            spi_display_dmawrite(&spi->spi_disp, sizeof(uint16_t), pixel);

            // Small delay between pixels (1ms)
            delay(5);

            if ((x % 20) == 0 && (y % 20) == 0) {
                ESP_LOGI(TAG, "Drawing pixel at %d,%d with color 0x%04X", x, y, color);
            }
        }
    }

    spi_device_release_bus(spi->spi_disp.handle);
    free(pixel);

    ESP_LOGI(TAG, "Test pattern completed");
}
