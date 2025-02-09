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

#include "font.c"

static const char *TAG = "gc9a01_display_driver";

static void send_message(term pid, term message, GlobalContext *global);

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
    return (uint16_t)((result >> 16) | result);
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

    return (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);
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

static inline void writecommand(struct SPI *spi, uint8_t cmd)
{
    gpio_set_level(spi->dc_gpio, 0);
    spi_display_write(&spi->spi_disp, 8, cmd);
    gpio_set_level(spi->dc_gpio, 1);
}

static inline void writedata(struct SPI *spi, uint8_t data)
{
    spi_display_write(&spi->spi_disp, 8, data);
}

static void display_init_gc9a01(struct SPI *spi)
{
    // Initialize GC9A01 display
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

    writecommand(spi, 0xB6);
    writedata(spi, 0x00);
    writedata(spi, 0x20);

    writecommand(spi, GC9A01_COLMOD);
    writedata(spi, 0x05);

    writecommand(spi, 0x90);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x08);

    writecommand(spi, 0xBD);
    writedata(spi, 0x06);

    writecommand(spi, 0xBC);
    writedata(spi, 0x00);

    writecommand(spi, 0xFF);
    writedata(spi, 0x60);
    writedata(spi, 0x01);
    writedata(spi, 0x04);

    writecommand(spi, 0xC3);
    writedata(spi, 0x13);
    writecommand(spi, 0xC4);
    writedata(spi, 0x13);

    writecommand(spi, 0xC9);
    writedata(spi, 0x22);

    writecommand(spi, 0xBE);
    writedata(spi, 0x11);

    writecommand(spi, 0xE1);
    writedata(spi, 0x10);
    writedata(spi, 0x0E);

    writecommand(spi, 0xDF);
    writedata(spi, 0x21);
    writedata(spi, 0x0c);
    writedata(spi, 0x02);

    writecommand(spi, 0xF0);
    writedata(spi, 0x45);
    writedata(spi, 0x09);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x26);
    writedata(spi, 0x2A);

    writecommand(spi, 0xF1);
    writedata(spi, 0x43);
    writedata(spi, 0x70);
    writedata(spi, 0x72);
    writedata(spi, 0x36);
    writedata(spi, 0x37);
    writedata(spi, 0x6F);

    writecommand(spi, 0xF2);
    writedata(spi, 0x45);
    writedata(spi, 0x09);
    writedata(spi, 0x08);
    writedata(spi, 0x08);
    writedata(spi, 0x26);
    writedata(spi, 0x2A);

    writecommand(spi, 0xF3);
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

    writecommand(spi, 0x98);
    writedata(spi, 0x3E);
    writedata(spi, 0x07);

    writecommand(spi, GC9A01_MADCTL);
    writedata(spi, TFT_MAD_COLOR_ORDER);

    writecommand(spi, GC9A01_SLPOUT);
    vTaskDelay(120 / portTICK_PERIOD_MS);

    writecommand(spi, GC9A01_DISPON);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

static void display_init(Context *ctx, term opts)
{
    struct SPI *spi = malloc(sizeof(struct SPI));
    if (!spi) {
        ESP_LOGE(TAG, "Failed to allocate SPI structure");
        return;
    }

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.clock_speed_hz = SPI_CLOCK_HZ;
    spi_config.mode = SPI_MODE;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(
        opts, ATOM_STR("\x2", "dc"), &spi->dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(
        opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);

    if (!ok) {
        ESP_LOGE(TAG, "Failed init: invalid GPIO configuration");
        free(spi);
        return;
    }

    // Reset
    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);

    display_init_gc9a01(spi);

    ctx->platform_data = spi;
    spi->ctx = ctx;

    display_messages_queue = xQueueCreate(32, sizeof(Message *));
    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}

Context *gc9a01_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
} 