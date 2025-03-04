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
#define SPI_CLOCK_HZ 40000000
#define SPI_MODE 0

#define CHAR_WIDTH 8

#define GC9A01_SWRESET 0x01
#define GC9A01_TFTWIDTH 240
#define GC9A01_TFTHEIGHT 240

#define GC9A01_SLPIN 0x10
#define GC9A01_SLPOUT 0x11
#define GC9A01_NORON 0x13
#define GC9A01_INVOFF 0x20
#define GC9A01_INVON 0x21
#define LCD_CMD_DISPOFF      0x28 // Display off (disable frame buffer output)
#define LCD_CMD_DISPON       0x29 // Display on (enable frame buffer output)
#define GC9A01_CASET 0x2A // Column Address Set
#define GC9A01_RASET 0x2B // Row Address Set
#define GC9A01_RAMWR 0x2C // Memory Write
#define GC9A01_MADCTL 0x36 // Memory Access Control
#define GC9A01_COLMOD 0x3A // Color Mode => RGB565

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
static void display_init_std(struct SPI *spi);
static void draw_test_pattern(struct SPI *spi);

static inline void writedata(struct SPI *spi, uint8_t data)
{
    fprintf(stderr, "Writing data: 0x%02X\n", data);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 8, data);
    spi_device_release_bus(spi->spi_disp.handle);
}

static inline void writecommand(struct SPI *spi, uint8_t command)
{
    fprintf(stderr, "Writing command: 0x%02X\n", command);
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
    // uint8_t madctl = TFT_MAD_COLOR_ORDER;

    // switch (rotation) {
    //     case 3:
    //         madctl |= GC9A01_MADCTL_MX | GC9A01_MADCTL_MY | GC9A01_MADCTL_MV;
    //         break;
    //     case 2:
    //         madctl |= GC9A01_MADCTL_MX | GC9A01_MADCTL_MY;
    //         break;
    //     case 1:
    //         madctl |= GC9A01_MADCTL_MV;
    //         break;
    //     case 0:
    //     default:
    //         break;
    // }

    // The reference implementation uses rotation 0 with MADCTL value 0x48
    // This corresponds to:
    // - MX bit (0x40) = 1 : Column address order reversed
    // - MH bit (0x08) = 1 : Display data latch order reversed
    
    writecommand(spi, GC9A01_MADCTL);
    //     writedata(spi, madctl);
    writedata(spi, 0x48);  // Fixed value used by the reference implementation
    
    ESP_LOGI(TAG, "Set display rotation to reference implementation default (0x48)");
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

    bool reset_configured = true;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
        reset_configured = false;
    }

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    spi->rotation = term_to_int(rotation);

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    ESP_LOGI(TAG, "Starting GPIO initialization - DC GPIO: %d, Reset GPIO: %d", spi->dc_gpio, spi->reset_gpio);

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Reset
    if (reset_configured) {
        ESP_LOGE(TAG, "Resetting display at pin %d...", spi->reset_gpio);
        spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
        gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(spi->reset_gpio, 1);
        delay(200);
        gpio_set_level(spi->reset_gpio, 0);
        delay(200);
        gpio_set_level(spi->reset_gpio, 1);
        delay(200);
        spi_device_release_bus(spi->spi_disp.handle);
    }

        writecommand(spi, GC9A01_SLPOUT);
    delay(100);
    writecommand(spi, GC9A01_MADCTL);
    writedata(spi, 0x08);
    writecommand(spi, GC9A01_COLMOD);
    writedata(spi, 0x55);

    

    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);

    if (!reset_configured) {
        writecommand(spi, GC9A01_SWRESET);
        delay(100);
    }

    term init_seq_type_term = interop_kv_get_value_default(opts, ATOM_STR("\xD", "init_seq_type"), term_nil(), ctx->global);
    int str_ok;
    char *init_seq_type_string = interop_term_to_string(init_seq_type_term, &str_ok);
    // if (str_ok && !strcmp(init_seq_type_string, "alt_gamma_2")) {
    //     display_init_alt_gamma_2(spi);
    //     free(init_seq_type_string);
    // } else {
    display_init_std(spi);
    // }

    set_rotation(spi, spi->rotation);

    if (enable_tft_invon) {
        writecommand(spi, GC9A01_INVON);
    }

    // writecommand(spi, GC9A01_DISPON);
    // delay(120);

    // writecommand(spi, GC9A01_SLPOUT);
    // delay(120);
    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);

    // draw_test_pattern(spi);
    // ESP_LOGI(TAG, "Test pattern drawn");
}

static void display_init_std(struct SPI *spi)
{
    ESP_LOGI(TAG, "Sending GC9A01 initialization commands...");
    
    // Initial delay after reset
    delay(120);

    writecommand(spi, 0xFE); // Inter Register Enable1
    writecommand(spi, 0xEF); // Inter Register Enable2

    writecommand(spi, 0xEB);
    writedata(spi, 0x14);


    writecommand(spi, 0xEB);
    writedata(spi, 0x14);
    ///
    writecommand(spi, 0x84);
    writedata(spi, 0x60);

    writecommand(spi, 0x85);
    writedata(spi, 0xFF);

    writecommand(spi, 0x86);
    writedata(spi, 0xFF);

    writecommand(spi, 0x87);
    writedata(spi, 0xFF);
    
    writecommand(spi, 0x8e);
    writedata(spi, 0xFF);

    writecommand(spi, 0x8f);
    writedata(spi, 0xFF);

    writecommand(spi, 0x88);
    writedata(spi, 0x0A);

    writecommand(spi, 0x89);
    writedata(spi, 0x23);    

    writecommand(spi, 0x8A);
    writedata(spi, 0x00);

    writecommand(spi, 0x8B);
    writedata(spi, 0x80);

    writecommand(spi, 0x8C);
    writedata(spi, 0x01);

    writecommand(spi, 0x8D);
    writedata(spi, 0x03);

    writecommand(spi, 0x90);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x08);

    writecommand(spi, 0xFF);
    writedata(spi, 0x60);
    writedata(spi, 0x01);
    writedata(spi, 0x04);

    writecommand(spi, 0xC3); // Power Control 2
    writedata(spi, 0x13);

    writecommand(spi, 0xC4); // Power Control 3
    writedata(spi, 0x13);

    writecommand(spi, 0xC9); // Power Control 4
    writedata(spi, 0x30);

    writecommand(spi, 0xBE);
    writedata(spi, 0x11);

    writecommand(spi, 0xE1);
    writedata(spi, 0x10);
    writedata(spi, 0x0E);

    writecommand(spi, 0xDF);
    writedata(spi, 0x21);
    writedata(spi, 0x0C);
    writedata(spi, 0x02);

    writecommand(spi, 0xF0); // SET_GAMMA1
    writedata(spi, 0x45);
    writedata(spi, 0x09);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x26);
    writedata(spi, 0x2A);

    writecommand(spi, 0xF1); // SET_GAMMA2
    writedata(spi, 0x43);
    writedata(spi, 0x70);
    writedata(spi, 0x72);
    writedata(spi, 0x36);
    writedata(spi, 0x37);
    writedata(spi, 0x6F);

    writecommand(spi, 0xF2); // SET_GAMMA3
    writedata(spi, 0x45);
    writedata(spi, 0x09);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x26);
    writedata(spi, 0x2A);

    writecommand(spi, 0xF3); // SET_GAMMA4
    writedata(spi, 0x43);
    writedata(spi, 0x70);
    writedata(spi, 0x72);
    writedata(spi, 0x36);
    writedata(spi, 0x37);
    writedata(spi, 0x6F);

    writecommand(spi, 0xED);
    writedata(spi, 0x1B);
    writedata(spi, 0x0B);

    writecommand(spi, 0xAE);
    writedata(spi, 0x77);

    writecommand(spi, 0xCD);
    writedata(spi, 0x63);

    writecommand(spi, 0x70);
    writedata(spi, 0x07);
    writedata(spi, 0x07);
    writedata(spi, 0x04);
    writedata(spi, 0x0E);
    writedata(spi, 0x0F);
    writedata(spi, 0x09);
    writedata(spi, 0x07);
    writedata(spi, 0x08);
    writedata(spi, 0x03);

    writecommand(spi, 0xE8);
    writedata(spi, 0x34);

writecommand(spi, 0x60);
    writedata(spi, 0x38);
    writedata(spi, 0x0B);
    writedata(spi, 0x6D);
    writedata(spi, 0x6D);
    writedata(spi, 0x39);
    writedata(spi, 0xF0);
    writedata(spi, 0x6D);
    writedata(spi, 0x6D);

    writecommand(spi, 0x61);
    writedata(spi, 0x38);
    writedata(spi, 0xF4);
    writedata(spi, 0x6D);
    writedata(spi, 0x6D);
    writedata(spi, 0x38);
    writedata(spi, 0xF7);
    writedata(spi, 0x6D);
    writedata(spi, 0x6D);


    writecommand(spi, 0x62);
    writedata(spi, 0x38);
    writedata(spi, 0x0D);
    writedata(spi, 0x71);
    writedata(spi, 0xED);
    writedata(spi, 0x70);
    writedata(spi, 0x70);
    writedata(spi, 0x38);
    writedata(spi, 0x0F);
    writedata(spi, 0x71);
    writedata(spi, 0xEF);
    writedata(spi, 0x70);
    writedata(spi, 0x70);

    writecommand(spi, 0x63);
    writedata(spi, 0x38);
    writedata(spi, 0x11);
    writedata(spi, 0x71);
    writedata(spi, 0xF1);
    writedata(spi, 0x70);
    writedata(spi, 0x70);
    writedata(spi, 0x38);
    writedata(spi, 0x13);
    writedata(spi, 0x71);
    writedata(spi, 0xF3);
    writedata(spi, 0x70);
    writedata(spi, 0x70);

    writecommand(spi, 0x64);
    writedata(spi, 0x28);
    writedata(spi, 0x29);
    writedata(spi, 0xF1);
    writedata(spi, 0x01);
    writedata(spi, 0xF1);
    writedata(spi, 0x00);
    writedata(spi, 0x07);

    writecommand(spi, 0x66);
    writedata(spi, 0x3C);
    writedata(spi, 0x00);
    writedata(spi, 0xCD);
    writedata(spi, 0x67);
    writedata(spi, 0x45);
    writedata(spi, 0x45);
    writedata(spi, 0x10);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);

    writecommand(spi, 0x67);
    writedata(spi, 0x00);
    writedata(spi, 0x3C);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x01);
    writedata(spi, 0x54);
    writedata(spi, 0x10);
    writedata(spi, 0x32);
    writedata(spi, 0x98);

    writecommand(spi, 0x74);
    writedata(spi, 0x10);
    writedata(spi, 0x45);
    writedata(spi, 0x80);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x4E);
    writedata(spi, 0x00);

    writecommand(spi, 0x98);
    writedata(spi, 0x3E);
    writedata(spi, 0x07);

    writecommand(spi, 0x99);
    writedata(spi, 0x3E);
    writedata(spi, 0x07);


    writecommand(spi, LCD_CMD_DISPON);  // Display ON
    ESP_LOGI(TAG, "Display ON");

    delay(120);

    ESP_LOGI(TAG, "GC9A01 initialization completed");
}

static void draw_test_pattern(struct SPI *spi)
{
    ESP_LOGI(TAG, "Drawing test pattern - 5 colored rectangles");

    // Define rectangle dimensions
    const int rect_width = 40;
    const int rect_height = 40;
    const int spacing = 10;
    const int start_x = 20;
    const int start_y = 100;

    // Define 5 colors in RGB565 format
    const uint16_t colors[] = {
        0xF800,  // Red (0b1111100000000000)
        0x07E0,  // Green (0b0000011111100000)
        0x001F,  // Blue (0b0000000000011111)
        0xFFE0,  // Yellow (0b1111111111100000)
        0x780F   // Purple (0b0111100000001111)
    };

    // Allocate buffer for one line of pixels
    uint16_t *line_buffer = heap_caps_malloc(rect_width * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!line_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test pattern buffer");
        return;
    }

    // Draw each rectangle
    for (int rect = 0; rect < 5; rect++) {
        int x = start_x + (rect_width + spacing) * rect;
        
        // Fill line buffer with current color
        uint16_t color = SPI_SWAP_DATA_TX(colors[rect], 16);
        for (int i = 0; i < rect_width; i++) {
            line_buffer[i] = color;
        }

        // Set drawing area for current rectangle
        set_screen_paint_area(spi, x, start_y, rect_width, rect_height);
        writecommand(spi, GC9A01_RAMWR);

        // Draw the rectangle line by line
        spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
        for (int y = 0; y < rect_height; y++) {
            spi_display_dmawrite(&spi->spi_disp, rect_width * sizeof(uint16_t), line_buffer);
        }
        spi_device_release_bus(spi->spi_disp.handle);

        ESP_LOGI(TAG, "Drew rectangle %d at x=%d y=%d", rect + 1, x, start_y);
        delay(1000);
    }

    free(line_buffer);
    ESP_LOGI(TAG, "Test pattern completed");
}
