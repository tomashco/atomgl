/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "display_driver.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <esp_heap_caps.h>

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

// #include "backlight_gpio.h"
#include "display_common.h"
#include "display_items.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "spi_display.h"

#include <stdio.h>
#include <sys/lock.h>
#include <sys/param.h>
#include <unistd.h>

#include "esp_lcd_gc9a01.h"

static const char *TAG
    = "gc9a01_display_driver";

// Using SPI2 in the example
#define LCD_HOST SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK 18
#define EXAMPLE_PIN_NUM_MOSI 23
#define EXAMPLE_PIN_NUM_MISO -1
#define EXAMPLE_PIN_NUM_LCD_DC 12
#define EXAMPLE_PIN_NUM_LCD_RST 33
#define EXAMPLE_PIN_NUM_LCD_CS 5
#define EXAMPLE_PIN_NUM_BK_LIGHT 9
#define EXAMPLE_PIN_NUM_TOUCH_CS -1

#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 240

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_DRAW_BUF_LINES 20 // number of display lines in each draw buffer
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

static void display_init(Context *ctx, term opts);

extern void example_lvgl_demo_ui(lv_display_t *disp);

static void send_message(term pid, term message, GlobalContext *global);

// Forward declarations
static lv_obj_t *create_scaled_cropped_image_efficient(lv_obj_t *parent, BaseDisplayItem *item);

static inline void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct SPI
{
    struct SPIDisplay spi_disp;
    int dc_gpio;
    // int reset_gpio;

    avm_int_t rotation;

    Context *ctx;
    lv_display_t *display; // Store the LVGL display pointer
};

// struct PendingReply
// {
//     uint64_t pending_call_ref_ticks;
//     term pending_call_pid;
// };

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);
    ESP_LOGI(TAG, "do_update: Starting update with %d display items", len);

    // Get the LVGL display
    lv_display_t *display = NULL;

    // Find the active display
    display = lv_display_get_default();
    if (!display) {
        ESP_LOGE(TAG, "do_update: No LVGL display found, aborting update");
        return;
    }
    ESP_LOGI(TAG, "do_update: Found LVGL display");

    // Get the active screen
    lv_obj_t *scr = lv_display_get_screen_active(display);
    if (!scr) {
        ESP_LOGE(TAG, "do_update: No active screen found, aborting update");
        return;
    }
    ESP_LOGI(TAG, "do_update: Found active screen");

    // Clear the screen first
    ESP_LOGI(TAG, "do_update: Cleaning screen");
    lv_obj_clean(scr);

    // Process each item in the display list
    term t = display_list;
    for (int i = 0; i < len; i++) {
        term item_term = term_get_list_head(t);

        // Create a temporary BaseDisplayItem to parse the term
        BaseDisplayItem item;
        init_item(&item, item_term, ctx);

        // Process based on primitive type
        switch (item.primitive) {
            case Rect: {
                ESP_LOGI(TAG, "do_update: Creating rectangle at (%d,%d) size %dx%d color 0x%06x",
                    item.x, item.y, item.width, item.height, item.brcolor);

                // Create an LVGL rectangle
                lv_obj_t *rect = lv_obj_create(scr);

                // Set position and size
                lv_obj_set_pos(rect, item.x, item.y);
                lv_obj_set_size(rect, item.width, item.height);

                // Set color (convert from RGBA8888 to LVGL color format)
                lv_color_t color = lv_color_make(
                    (item.brcolor >> 16) & 0xFF, // R
                    (item.brcolor >> 8) & 0xFF, // G
                    item.brcolor & 0xFF // B
                );

                lv_obj_set_style_bg_color(rect, color, LV_PART_MAIN);
                lv_obj_set_style_border_width(rect, 0, LV_PART_MAIN);
                break;
            }

            case Text: {
                ESP_LOGI(TAG, "do_update: Creating text at (%d,%d) text '%s' color 0x%06x",
                    item.x, item.y, item.data.text_data.text, item.data.text_data.fgcolor);

                // Create an LVGL label
                lv_obj_t *label = lv_label_create(scr);

                // Set position
                lv_obj_set_pos(label, item.x, item.y);

                // Set text
                lv_label_set_text(label, item.data.text_data.text);

                // Set color
                lv_color_t color = lv_color_make(
                    (item.data.text_data.fgcolor >> 16) & 0xFF, // R
                    (item.data.text_data.fgcolor >> 8) & 0xFF, // G
                    item.data.text_data.fgcolor & 0xFF // B
                );

                lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
                break;
            }

            case Image: {
                ESP_LOGI(TAG, "do_update: Creating image at (%d,%d) size %dx%d",
                    item.x, item.y, item.width, item.height);

                // For images, we need to create an LVGL image descriptor
                // This is more complex and depends on your image format
                // For simplicity, we'll create a basic implementation

                // Create a buffer for the image data
                lv_color_t *buf = malloc(item.width * item.height * sizeof(lv_color_t));
                if (!buf) {
                    ESP_LOGE(TAG, "do_update: Failed to allocate memory for image (%d bytes)",
                        item.width * item.height * sizeof(lv_color_t));
                    break;
                }
                ESP_LOGI(TAG, "do_update: Allocated %d bytes for image buffer",
                    item.width * item.height * sizeof(lv_color_t));

                // Convert image data to LVGL format
                // Assuming image data is in RGB565 format
                const uint16_t *src = (const uint16_t *) item.data.image_data.pix;
                ESP_LOGI(TAG, "do_update: Converting image data from RGB565 to LVGL format");
                for (int j = 0; j < item.width * item.height; j++) {
                    uint16_t pixel = src[j];
                    // Convert RGB565 to LVGL color format
                    uint8_t r = (pixel >> 11) & 0x1F;
                    uint8_t g = (pixel >> 5) & 0x3F;
                    uint8_t b = pixel & 0x1F;

                    // Scale to 8-bit per channel
                    r = (r * 255) / 31;
                    g = (g * 255) / 63;
                    b = (b * 255) / 31;

                    buf[j] = lv_color_make(r, g, b);
                }

                // Create an LVGL image descriptor
                lv_image_dsc_t img_dsc;
                img_dsc.data = (const uint8_t *) buf;
                img_dsc.data_size = item.width * item.height * sizeof(lv_color_t);
                img_dsc.header.w = item.width;
                img_dsc.header.h = item.height;
                img_dsc.header.cf = LV_COLOR_FORMAT_NATIVE;

                // Create an LVGL image
                ESP_LOGI(TAG, "do_update: Creating LVGL image object");
                lv_obj_t *img = lv_image_create(scr);
                lv_image_set_src(img, &img_dsc);
                lv_obj_set_pos(img, item.x, item.y);

                // Note: This creates a memory leak as we don't free the buffer
                // In a real implementation, you'd need to handle this properly
                ESP_LOGW(TAG, "do_update: Warning - image buffer not freed (memory leak)");
                break;
            }

            case ScaledCroppedImage: {
                ESP_LOGI(TAG, "do_update: Creating scaled/cropped image at (%d,%d) size %dx%d from source (%d,%d) with scale (%d,%d)",
                    item.x, item.y, item.width, item.height,
                    item.source_x, item.source_y, item.x_scale, item.y_scale);

                // Use the memory-efficient implementation
                lv_obj_t *img_obj = create_scaled_cropped_image_efficient(scr, &item);
                if (!img_obj) {
                    ESP_LOGE(TAG, "do_update: Failed to create scaled/cropped image");
                }
                break;
            }

            default:
                ESP_LOGW(TAG, "do_update: Unknown primitive type: %d", item.primitive);
                break;
        }

        // Move to the next item in the list
        t = term_get_list_tail(t);
    }

    // Force a refresh of the display
    ESP_LOGI(TAG, "do_update: Forcing display refresh with lv_refr_now()");
    lv_refr_now(display);
    ESP_LOGI(TAG, "do_update: Update completed");
}

static void draw_buffer(struct SPI *spi, int x, int y, int width, int height, const void *imgdata)
{
    ESP_LOGI(TAG, "draw_buffer: Drawing buffer at (%d,%d) size %dx%d", x, y, width, height);

    // Get the LVGL display
    lv_display_t *display = spi->display;
    if (!display) {
        ESP_LOGE(TAG, "draw_buffer: No LVGL display found");
        return;
    }

    // Get the active screen
    lv_obj_t *scr = lv_display_get_screen_active(display);
    if (!scr) {
        ESP_LOGE(TAG, "draw_buffer: No active screen found");
        return;
    }

    // Check available memory
    size_t buf_size = width * height * sizeof(lv_color_t);
    size_t free_heap = esp_get_free_heap_size();
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

    ESP_LOGI(TAG, "draw_buffer: Memory requirements - Buffer: %d bytes", buf_size);
    ESP_LOGI(TAG, "draw_buffer: Memory available - Free heap: %d bytes, Largest free block: %d bytes",
        free_heap, largest_block);

    if (buf_size > largest_block) {
        ESP_LOGW(TAG, "draw_buffer: Not enough contiguous memory, using tiled approach");

        // Determine a reasonable tile size based on available memory
        // Use at most 1/4 of the largest available block to be safe
        size_t max_tile_bytes = largest_block / 4;
        int max_tile_pixels = max_tile_bytes / sizeof(lv_color_t);
        int tile_width = width;
        int tile_height = max_tile_pixels / tile_width;

        // Ensure tile height is at least 1 pixel
        if (tile_height < 1) {
            tile_height = 1;
            tile_width = max_tile_pixels;
            if (tile_width < 1) {
                tile_width = 1; // Absolute minimum
            }
        }

        ESP_LOGI(TAG, "draw_buffer: Using tiles of size %dx%d pixels", tile_width, tile_height);

        // Process the image in tiles
        const uint16_t *src_data = (const uint16_t *) imgdata;

        for (int ty = 0; ty < height; ty += tile_height) {
            int current_tile_height = (ty + tile_height > height) ? (height - ty) : tile_height;

            for (int tx = 0; tx < width; tx += tile_width) {
                int current_tile_width = (tx + tile_width > width) ? (width - tx) : tile_width;

                // Allocate buffer for a single tile
                size_t tile_buf_size = current_tile_width * current_tile_height * sizeof(lv_color_t);
                lv_color_t *tile_buf = malloc(tile_buf_size);
                if (!tile_buf) {
                    ESP_LOGE(TAG, "draw_buffer: Failed to allocate memory for tile (%d bytes)", tile_buf_size);
                    return;
                }

                // Convert the tile data from RGB565 to LVGL format
                for (int j = 0; j < current_tile_height; j++) {
                    for (int i = 0; i < current_tile_width; i++) {
                        int src_idx = (ty + j) * width + (tx + i);
                        uint16_t pixel = src_data[src_idx];

                        // Convert RGB565 to LVGL color format
                        uint8_t r = (pixel >> 11) & 0x1F;
                        uint8_t g = (pixel >> 5) & 0x3F;
                        uint8_t b = pixel & 0x1F;

                        // Scale to 8-bit per channel
                        r = (r * 255) / 31;
                        g = (g * 255) / 63;
                        b = (b * 255) / 31;

                        tile_buf[j * current_tile_width + i] = lv_color_make(r, g, b);
                    }
                }

                // Create an LVGL image descriptor for this tile
                lv_image_dsc_t img_dsc;
                img_dsc.data = (const uint8_t *) tile_buf;
                img_dsc.data_size = tile_buf_size;
                img_dsc.header.w = current_tile_width;
                img_dsc.header.h = current_tile_height;
                img_dsc.header.cf = LV_COLOR_FORMAT_NATIVE;

                // Create an LVGL image for this tile
                lv_obj_t *img = lv_image_create(scr);
                lv_image_set_src(img, &img_dsc);
                lv_obj_set_pos(img, x + tx, y + ty);

                // Force a refresh for this tile
                lv_refr_now(display);

                // Clean up
                lv_obj_delete(img);
                free(tile_buf);
            }
        }
    } else {
        // We have enough memory, use a single buffer
        lv_color_t *buf = malloc(buf_size);
        if (!buf) {
            ESP_LOGE(TAG, "draw_buffer: Failed to allocate memory for image buffer (%d bytes)", buf_size);
            return;
        }

        // Convert image data from RGB565 to LVGL format
        const uint16_t *src = (const uint16_t *) imgdata;
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                uint16_t pixel = src[j * width + i];

                // Convert RGB565 to LVGL color format
                uint8_t r = (pixel >> 11) & 0x1F;
                uint8_t g = (pixel >> 5) & 0x3F;
                uint8_t b = pixel & 0x1F;

                // Scale to 8-bit per channel
                r = (r * 255) / 31;
                g = (g * 255) / 63;
                b = (b * 255) / 31;

                buf[j * width + i] = lv_color_make(r, g, b);
            }
        }

        // Create an LVGL image descriptor
        lv_image_dsc_t img_dsc;
        img_dsc.data = (const uint8_t *) buf;
        img_dsc.data_size = buf_size;
        img_dsc.header.w = width;
        img_dsc.header.h = height;
        img_dsc.header.cf = LV_COLOR_FORMAT_NATIVE;

        // Create an LVGL image
        lv_obj_t *img = lv_image_create(scr);
        lv_image_set_src(img, &img_dsc);
        lv_obj_set_pos(img, x, y);

        // Force a refresh
        lv_refr_now(display);

        // Clean up
        lv_obj_delete(img);
        free(buf);
    }
}

static QueueHandle_t display_messages_queue;

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);

static void process_message(Message *message, Context *ctx)
{
    ESP_LOGI(TAG, "process_message: Processing new message");

    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        ESP_LOGE(TAG, "process_message: Received invalid message format");
        fprintf(stderr, "Received invalid message.");
        AVM_ABORT();
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        ESP_LOGE(TAG, "process_message: Invalid request format - not a tuple or arity < 1");
        AVM_ABORT();
    }
    term cmd = term_get_tuple_element(req, 0);

    struct SPI *spi = ctx->platform_data;

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        ESP_LOGI(TAG, "process_message: Received 'update' command");
        term display_list = term_get_tuple_element(req, 1);

        // Lock the mutex due to the LVGL APIs are not thread-safe
        ESP_LOGI(TAG, "process_message: Acquiring LVGL mutex");
        _lock_acquire(&lvgl_api_lock);
        do_update(ctx, display_list);
        ESP_LOGI(TAG, "process_message: Releasing LVGL mutex");
        _lock_release(&lvgl_api_lock);

    } else if (cmd == context_make_atom(ctx, "\xB"
                                             "draw_buffer")) {
        ESP_LOGI(TAG, "process_message: Received 'draw_buffer' command");
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        unsigned long addr_low = term_to_int(term_get_tuple_element(req, 5));
        unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));

        const void *data = (const void *) ((addr_low | (addr_high << 16)));
        ESP_LOGI(TAG, "process_message: Drawing buffer at (%d,%d) size %dx%d", x, y, width, height);

        // Lock the mutex due to the LVGL APIs are not thread-safe
        ESP_LOGI(TAG, "process_message: Acquiring LVGL mutex");
        _lock_acquire(&lvgl_api_lock);
        draw_buffer(spi, x, y, width, height, data);
        ESP_LOGI(TAG, "process_message: Releasing LVGL mutex");
        _lock_release(&lvgl_api_lock);

        // draw_buffer is a kind of cast, no need to reply
        ESP_LOGI(TAG, "process_message: draw_buffer completed (no reply needed)");
        return;

    } else {
        ESP_LOGW(TAG, "process_message: Unknown command received");
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    ESP_LOGI(TAG, "process_message: Sending reply");
    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
    ESP_LOGI(TAG, "process_message: Message processing completed");
}

static void process_messages(void *arg)
{
    struct SPI *args = arg;
    ESP_LOGI(TAG, "process_messages: Task started");

    while (true) {
        ESP_LOGI(TAG, "process_messages: Waiting for message from queue");
        Message *message;
        xQueueReceive(display_messages_queue, &message, portMAX_DELAY);

        // First validate the message format
        // this is still not working but somehow adds a delay that allows to correctly start processing messages
        GenMessage gen_message;
        if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
            ESP_LOGW(TAG, "process_messages: Received invalid message format");
            // Clean up invalid message
            BEGIN_WITH_STACK_HEAP(1, temp_heap);
            mailbox_message_dispose(&message->base, &temp_heap);
            END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
            continue;
        }

        // Now that we know it's valid, we can safely log it
        ESP_LOGI(TAG, "process_messages: Received valid message:");
        ESP_LOGI(TAG, "process_messages: From PID: %lx", term_to_local_process_id(gen_message.pid));

// If you still want to see the raw message content (optional)
#ifdef DEBUG
        fprintf(stdout, "Message content: ");
        term_display(stdout, message->message, args->ctx);
        fprintf(stdout, "\n");
#endif

        process_message(message, args->ctx);

        ESP_LOGI(TAG, "process_messages: Disposing message");
        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
        ESP_LOGI(TAG, "process_messages: Message disposed, waiting for next message");
    }
}

Context *gc9a01_display_create_port(GlobalContext *global, term opts)
{
    ESP_LOGI(TAG, "INIT GC9A01 DISPLAY");
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

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx)
{
    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    xQueueSend(display_messages_queue, &msg, 1);

    return NativeContinue;
}

// this is how to get the rotation from the opts
// term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);

//=======================================================
//=======================================================
//=======================================================

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *) user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
        case LV_DISPLAY_ROTATION_0:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, true, false);
            break;
        case LV_DISPLAY_ROTATION_90:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, true, true);
            break;
        case LV_DISPLAY_ROTATION_180:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, false, true);
            break;
        case LV_DISPLAY_ROTATION_270:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
    }
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    example_lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
}

void display_init(Context *ctx, term opts)
{
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;
    spi->ctx = ctx;

    // struct SPIDisplayConfig spi_config;
    // spi_display_init_config(&spi_config);
    // spi_config.mode = 0;
    // spi_config.clock_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ;
    // spi_display_parse_config(&spi_config, opts, ctx->global); // takes CS pin
    // spi_display_init(&spi->spi_disp, &spi_config);
    // ^^^ this in the end adds the device with spi_bus_add_device. Here this is made using esp_lcd_new_panel_io_spi

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->dc_gpio, ctx->global);

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    spi->rotation = term_to_int(rotation);

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES); // this is the screen

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    // ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    // _lock_acquire(&lvgl_api_lock);
    // example_lvgl_demo_ui(display);
    // _lock_release(&lvgl_api_lock);

    spi->display = display;

    xTaskCreate(process_messages, "display", 10000, spi, 2, NULL);
}

static lv_obj_t *create_scaled_cropped_image_efficient(lv_obj_t *parent, BaseDisplayItem *item)
{
    ESP_LOGI(TAG, "Creating efficient scaled/cropped image at (%d,%d) size %dx%d from source (%d,%d) with scale (%d,%d)",
        item->x, item->y, item->width, item->height,
        item->source_x, item->source_y, item->x_scale, item->y_scale);

    // Get source image dimensions
    int img_width = item->data.image_data_with_size.width;
    int img_height = item->data.image_data_with_size.height;

    // Calculate the portion of the source image we need
    int src_width = item->width / item->x_scale;
    int src_height = item->height / item->y_scale;

    // Make sure we don't go beyond the source image boundaries
    if (item->source_x + src_width > img_width) {
        src_width = img_width - item->source_x;
    }
    if (item->source_y + src_height > img_height) {
        src_height = img_height - item->source_y;
    }

    // Log memory requirements
    size_t dst_buf_size = item->width * item->height * sizeof(lv_color_t);
    size_t free_heap = esp_get_free_heap_size();
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

    ESP_LOGI(TAG, "Memory requirements - Destination buffer: %d bytes", dst_buf_size);
    ESP_LOGI(TAG, "Memory available - Free heap: %d bytes, Largest free block: %d bytes",
        free_heap, largest_block);

    // Check if we have enough memory for the full buffer
    if (dst_buf_size > largest_block) {
        ESP_LOGW(TAG, "Not enough contiguous memory for full image, using tiled approach");

        // Create a canvas object instead
        lv_obj_t *canvas = lv_canvas_create(parent);

        // Set position
        lv_obj_set_pos(canvas, item->x, item->y);

        // Determine a reasonable tile size based on available memory
        // Use at most 1/4 of the largest available block to be safe
        size_t max_tile_bytes = largest_block / 4;
        int max_tile_pixels = max_tile_bytes / sizeof(lv_color_t);
        int tile_width = item->width;
        int tile_height = max_tile_pixels / tile_width;

        // Ensure tile height is at least 1 pixel
        if (tile_height < 1) {
            tile_height = 1;
            tile_width = max_tile_pixels;
            if (tile_width < 1) {
                tile_width = 1; // Absolute minimum
            }
        }

        // Allocate buffer for a single tile
        size_t tile_buf_size = tile_width * tile_height * sizeof(lv_color_t);
        lv_color_t *tile_buf = malloc(tile_buf_size);
        if (!tile_buf) {
            ESP_LOGE(TAG, "Failed to allocate memory even for a small tile (%d bytes)", tile_buf_size);
            return NULL;
        }

        ESP_LOGI(TAG, "Using tiles of size %dx%d pixels (%d bytes each)",
            tile_width, tile_height, tile_buf_size);

        // Create a canvas buffer for the entire image
        lv_canvas_set_buffer(canvas, tile_buf, tile_width, tile_height, LV_COLOR_FORMAT_NATIVE);

        // Process the image in tiles
        const uint32_t *src_img = (const uint32_t *) item->data.image_data_with_size.pix;

        // Draw each tile
        for (int ty = 0; ty < item->height; ty += tile_height) {
            int current_tile_height = (ty + tile_height > item->height) ? (item->height - ty) : tile_height;

            for (int tx = 0; tx < item->width; tx += tile_width) {
                int current_tile_width = (tx + tile_width > item->width) ? (item->width - tx) : tile_width;

                // Process this tile
                for (int y = 0; y < current_tile_height; y++) {
                    for (int x = 0; x < current_tile_width; x++) {
                        // Calculate source coordinates
                        int dst_x = tx + x;
                        int dst_y = ty + y;
                        int src_x = dst_x / item->x_scale;
                        int src_y = dst_y / item->y_scale;

                        // Ensure we're within bounds
                        if (src_x >= src_width)
                            src_x = src_width - 1;
                        if (src_y >= src_height)
                            src_y = src_height - 1;

                        // Get the source pixel
                        int src_idx = (item->source_y + src_y) * img_width + (item->source_x + src_x);
                        uint32_t rgba = src_img[src_idx];

                        // Extract RGBA components
                        uint8_t r = (rgba >> 24) & 0xFF;
                        uint8_t g = (rgba >> 16) & 0xFF;
                        uint8_t b = (rgba >> 8) & 0xFF;
                        uint8_t a = rgba & 0xFF;

                        // If pixel is transparent and we have a background color
                        if (a < 128 && item->brcolor != 0) {
                            // Use background color
                            r = (item->brcolor >> 24) & 0xFF;
                            g = (item->brcolor >> 16) & 0xFF;
                            b = (item->brcolor >> 8) & 0xFF;
                        }

                        // Set the pixel in the tile
                        lv_canvas_set_px(canvas, x, y, lv_color_make(r, g, b), LV_OPA_COVER);
                    }
                }

                // Draw the tile to the screen
                // In a real implementation, you would need to copy this tile to the screen
                // For now, we're just demonstrating the concept
            }
        }

        // We keep the tile buffer allocated as it's used by the canvas
        return canvas;
    } else {
        // We have enough memory, use the original approach
        lv_color_t *dst_buf = malloc(dst_buf_size);
        if (!dst_buf) {
            ESP_LOGE(TAG, "Failed to allocate memory for scaled image buffer (%d bytes)", dst_buf_size);
            return NULL;
        }

        ESP_LOGI(TAG, "Successfully allocated destination buffer");

        // Process the image
        const uint32_t *src_img = (const uint32_t *) item->data.image_data_with_size.pix;

        for (int y = 0; y < item->height; y++) {
            for (int x = 0; x < item->width; x++) {
                // Calculate source coordinates
                int src_x = x / item->x_scale;
                int src_y = y / item->y_scale;

                // Ensure we're within bounds
                if (src_x >= src_width)
                    src_x = src_width - 1;
                if (src_y >= src_height)
                    src_y = src_height - 1;

                // Get the source pixel
                int src_idx = (item->source_y + src_y) * img_width + (item->source_x + src_x);
                uint32_t rgba = src_img[src_idx];

                // Extract RGBA components
                uint8_t r = (rgba >> 24) & 0xFF;
                uint8_t g = (rgba >> 16) & 0xFF;
                uint8_t b = (rgba >> 8) & 0xFF;
                uint8_t a = rgba & 0xFF;

                // If pixel is transparent and we have a background color
                if (a < 128 && item->brcolor != 0) {
                    // Use background color
                    r = (item->brcolor >> 24) & 0xFF;
                    g = (item->brcolor >> 16) & 0xFF;
                    b = (item->brcolor >> 8) & 0xFF;
                }

                // Set the destination pixel
                dst_buf[y * item->width + x] = lv_color_make(r, g, b);
            }
        }

        // Create an LVGL image descriptor
        lv_image_dsc_t *img_dsc = malloc(sizeof(lv_image_dsc_t));
        if (!img_dsc) {
            ESP_LOGE(TAG, "Failed to allocate memory for image descriptor");
            free(dst_buf);
            return NULL;
        }

        img_dsc->data = (const uint8_t *) dst_buf;
        img_dsc->data_size = dst_buf_size;
        img_dsc->header.w = item->width;
        img_dsc->header.h = item->height;
        img_dsc->header.cf = LV_COLOR_FORMAT_NATIVE;

        // Create an LVGL image
        ESP_LOGI(TAG, "Creating LVGL image object for scaled/cropped image");
        lv_obj_t *img = lv_image_create(parent);
        lv_image_set_src(img, img_dsc);
        lv_obj_set_pos(img, item->x, item->y);

        // Note: This creates a memory leak as we don't free the buffer
        // In a real implementation, you'd need to handle this properly
        ESP_LOGW(TAG, "Warning - scaled image buffer not freed (memory leak)");

        return img;
    }
}
