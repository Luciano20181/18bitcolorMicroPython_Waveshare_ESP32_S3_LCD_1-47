/*
 * JD9853 driver for 18-bit color (RGB666) – no 16‑bit compatibility.
 * API uses 24‑bit colors (0xRRGGBB). Physical output is 3 bytes/pixel RGB666.
 * JPEG decoder renamed to tjpgd666, configured to output RGB888 then converted to RGB666.
 *
 * This file is based on work by Russ Hughes (MIT license) and has been heavily modified.
 */

#define __JD9853_VERSION__ "0.0.1-18bit"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "py/obj.h"
#include "py/objstr.h"
#include "py/objmodule.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mphal.h"

#if MICROPY_VERSION_MAJOR >= 1 && MICROPY_VERSION_MINOR > 21
#include "extmod/modmachine.h"
#else
#include "extmod/machine_spi.h"
#endif

#include "mpfile.h"
#include "jd9853.h"
#include "jpg/tjpgd666.h"
#include "png/pngle.h"

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#define ABS(N) (((N) < 0) ? (-(N)) : (N))
#define mp_hal_delay_ms(delay) (mp_hal_delay_us(delay * 1000))

#ifndef GPIO_NUM_NC
  #ifdef STM32_HAL_H
    #define GPIO_NUM_NC NULL
  #else
    #define GPIO_NUM_NC -1
  #endif
#endif

#define CS_LOW()   { if (self->cs != GPIO_NUM_NC) mp_hal_pin_write(self->cs, 0); }
#define CS_HIGH()  { if (self->cs != GPIO_NUM_NC) mp_hal_pin_write(self->cs, 1); }
#define DC_LOW()   mp_hal_pin_write(self->dc, 0)
#define DC_HIGH()  mp_hal_pin_write(self->dc, 1)
#define RESET_LOW()  { if (self->reset != GPIO_NUM_NC) mp_hal_pin_write(self->reset, 0); }
#define RESET_HIGH() { if (self->reset != GPIO_NUM_NC) mp_hal_pin_write(self->reset, 1); }

#define FILL_BUFFER_PIXELS 1024   // 1024 pixels -> 3072 bytes

// Tabla de rotaciones para 172x320 (sin offsets)
static jd9853_rotation_t ORIENTATIONS_172x320[4] = {
    {0x48, 172, 320, 0, 0},
    {0x28, 320, 172, 0, 0},
    {0x88, 172, 320, 0, 0},
    {0xE8, 320, 172, 0, 0}
};

static void write_spi(mp_obj_base_t *spi_obj, const uint8_t *buf, int len) {
    #ifdef MP_OBJ_TYPE_GET_SLOT
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)MP_OBJ_TYPE_GET_SLOT(spi_obj->type, protocol);
    #else
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)spi_obj->type->protocol;
    #endif
    spi_p->transfer(spi_obj, len, buf, NULL);
}

// Convierte color 24 bits (0xRRGGBB) a 3 bytes RGB666
static inline void color24_to_rgb666(uint8_t *dst, uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    dst[0] = r >> 2;
    dst[1] = g >> 2;
    dst[2] = b >> 2;
}

// Rellena un buffer de ráfaga con un color sólido (RGB666)
static void fill_color_buffer(mp_obj_base_t *spi_obj, uint32_t color, int length) {
    int chunks = length / FILL_BUFFER_PIXELS;
    int rest = length % FILL_BUFFER_PIXELS;
    uint8_t buffer[FILL_BUFFER_PIXELS * 3];
    uint8_t rgb666[3];
    color24_to_rgb666(rgb666, color);
    for (int i = 0; i < FILL_BUFFER_PIXELS; i++) {
        memcpy(buffer + i * 3, rgb666, 3);
    }
    if (chunks) {
        for (int j = 0; j < chunks; j++) {
            write_spi(spi_obj, buffer, FILL_BUFFER_PIXELS * 3);
        }
    }
    if (rest) {
        write_spi(spi_obj, buffer, rest * 3);
    }
}

static void jd9853_JD9853_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    (void)kind;
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "<JD9853 width=%u, height=%u, spi=%p>", self->width, self->height, self->spi_obj);
}

static void write_cmd(jd9853_JD9853_obj_t *self, uint8_t cmd, const uint8_t *data, int len) {
    CS_LOW();
    if (cmd) {
        DC_LOW();
        write_spi(self->spi_obj, &cmd, 1);
    }
    if (len > 0) {
        DC_HIGH();
        write_spi(self->spi_obj, data, len);
    }
    CS_HIGH();
}

static void set_window(jd9853_JD9853_obj_t *self, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (x0 > x1 || x1 >= self->width) return;
    if (y0 > y1 || y1 >= self->height) return;
    if (self->bounding) {
        if (x0 < self->min_x) self->min_x = x0;
        if (x1 > self->max_x) self->max_x = x1;
        if (y0 < self->min_y) self->min_y = y0;
        if (y1 > self->max_y) self->max_y = y1;
    }
    uint8_t bufx[4] = {(x0 + self->colstart) >> 8, (x0 + self->colstart) & 0xFF,
                       (x1 + self->colstart) >> 8, (x1 + self->colstart) & 0xFF};
    uint8_t bufy[4] = {(y0 + self->rowstart) >> 8, (y0 + self->rowstart) & 0xFF,
                       (y1 + self->rowstart) >> 8, (y1 + self->rowstart) & 0xFF};
    write_cmd(self, JD9853_CASET, bufx, 4);
    write_cmd(self, JD9853_RASET, bufy, 4);
    write_cmd(self, JD9853_RAMWR, NULL, 0);
}

static mp_obj_t jd9853_JD9853_set_window(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x0 = mp_obj_get_int(args[1]);
    mp_int_t y0 = mp_obj_get_int(args[2]);
    mp_int_t x1 = mp_obj_get_int(args[3]);
    mp_int_t y1 = mp_obj_get_int(args[4]);
    set_window(self, x0, y0, x1, y1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_set_window_obj, 5, 5, jd9853_JD9853_set_window);

void draw_pixel(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, uint32_t color) {
    if (self->options & OPTIONS_WRAP) {
        if ((self->options & OPTIONS_WRAP_H) && (x >= self->width || x < 0))
            x = x % self->width;
        if ((self->options & OPTIONS_WRAP_V) && (y >= self->height || y < 0))
            y = y % self->height;
    }
    if (x >= 0 && x < self->width && y >= 0 && y < self->height) {
        uint8_t rgb666[3];
        color24_to_rgb666(rgb666, color);
        set_window(self, x, y, x, y);
        DC_HIGH(); CS_LOW();
        write_spi(self->spi_obj, rgb666, 3);
        CS_HIGH();
    }
}

void fast_hline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t w, uint32_t color) {
    if ((self->options & OPTIONS_WRAP) == 0) {
        if (y >= 0 && y < self->height && x < self->width) {
            if (x < 0) { w += x; x = 0; }
            if (x + w > self->width) w = self->width - x;
            if (w > 0) {
                set_window(self, x, y, x + w - 1, y);
                DC_HIGH(); CS_LOW();
                fill_color_buffer(self->spi_obj, color, w);
                CS_HIGH();
            }
        }
    } else {
        for (int d = 0; d < w; d++) draw_pixel(self, x + d, y, color);
    }
}

static void fast_vline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t h, uint32_t color) {
    if ((self->options & OPTIONS_WRAP) == 0) {
        if (x >= 0 && x < self->width && y < self->height) {
            if (y < 0) { h += y; y = 0; }
            if (y + h > self->height) h = self->height - y;
            if (h > 0) {
                set_window(self, x, y, x, y + h - 1);
                DC_HIGH(); CS_LOW();
                fill_color_buffer(self->spi_obj, color, h);
                CS_HIGH();
            }
        }
    } else {
        for (int d = 0; d < h; d++) draw_pixel(self, x, y + d, color);
    }
}

static mp_obj_t jd9853_JD9853_hard_reset(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    CS_LOW();
    RESET_HIGH(); mp_hal_delay_ms(50);
    RESET_LOW();  mp_hal_delay_ms(50);
    RESET_HIGH(); mp_hal_delay_ms(150);
    CS_HIGH();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_hard_reset_obj, jd9853_JD9853_hard_reset);

static mp_obj_t jd9853_JD9853_soft_reset(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    write_cmd(self, JD9853_SWRESET, NULL, 0);
    mp_hal_delay_ms(120);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_soft_reset_obj, jd9853_JD9853_soft_reset);

static mp_obj_t jd9853_JD9853_sleep_mode(mp_obj_t self_in, mp_obj_t value) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (mp_obj_is_true(value))
        write_cmd(self, JD9853_SLPIN, NULL, 0);
    else
        write_cmd(self, JD9853_SLPOUT, NULL, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_sleep_mode_obj, jd9853_JD9853_sleep_mode);

static mp_obj_t jd9853_JD9853_inversion_mode(mp_obj_t self_in, mp_obj_t value) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->inversion = mp_obj_is_true(value);
    if (self->inversion)
        write_cmd(self, JD9853_INVON, NULL, 0);
    else
        write_cmd(self, JD9853_INVOFF, NULL, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_inversion_mode_obj, jd9853_JD9853_inversion_mode);

static mp_obj_t jd9853_JD9853_fill_rect(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]);
    mp_int_t h = mp_obj_get_int(args[4]);
    mp_int_t color = mp_obj_get_int(args[5]);
    if (x >= self->width || y >= self->height || x + w <= 0 || y + h <= 0) return mp_const_none;
    int16_t x0 = x, y0 = y, x1 = x + w - 1, y1 = y + h - 1;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= self->width) x1 = self->width - 1;
    if (y1 >= self->height) y1 = self->height - 1;
    int16_t new_w = x1 - x0 + 1;
    int16_t new_h = y1 - y0 + 1;
    if (new_w > 0 && new_h > 0) {
        set_window(self, x0, y0, x1, y1);
        DC_HIGH(); CS_LOW();
        fill_color_buffer(self->spi_obj, color, new_w * new_h);
        CS_HIGH();
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_rect_obj, 6, 6, jd9853_JD9853_fill_rect);

static mp_obj_t jd9853_JD9853_fill(mp_obj_t self_in, mp_obj_t _color) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t color = mp_obj_get_int(_color);
    set_window(self, 0, 0, self->width - 1, self->height - 1);
    DC_HIGH(); CS_LOW();
    fill_color_buffer(self->spi_obj, color, self->width * self->height);
    CS_HIGH();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_fill_obj, jd9853_JD9853_fill);

static mp_obj_t jd9853_JD9853_pixel(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t color = mp_obj_get_int(args[3]);
    draw_pixel(self, x, y, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_pixel_obj, 4, 4, jd9853_JD9853_pixel);

void line(jd9853_JD9853_obj_t *self, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color) {
    bool steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) { _swap_int16_t(x0, y0); _swap_int16_t(x1, y1); }
    if (x0 > x1) { _swap_int16_t(x0, x1); _swap_int16_t(y0, y1); }
    int16_t dx = x1 - x0, dy = ABS(y1 - y0);
    int16_t err = dx / 2, ystep = (y0 < y1) ? 1 : -1;
    int16_t xs = x0, dlen = 0;
    if (steep) {
        for (; x0 <= x1; x0++) {
            dlen++;
            err -= dy;
            if (err < 0) {
                err += dx;
                if (dlen == 1) draw_pixel(self, y0, xs, color);
                else fast_vline(self, y0, xs, dlen, color);
                dlen = 0;
                y0 += ystep;
                xs = x0 + 1;
            }
        }
        if (dlen) fast_vline(self, y0, xs, dlen, color);
    } else {
        for (; x0 <= x1; x0++) {
            dlen++;
            err -= dy;
            if (err < 0) {
                err += dx;
                if (dlen == 1) draw_pixel(self, xs, y0, color);
                else fast_hline(self, xs, y0, dlen, color);
                dlen = 0;
                y0 += ystep;
                xs = x0 + 1;
            }
        }
        if (dlen) fast_hline(self, xs, y0, dlen, color);
    }
}
static mp_obj_t jd9853_JD9853_line(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x0 = mp_obj_get_int(args[1]), y0 = mp_obj_get_int(args[2]);
    mp_int_t x1 = mp_obj_get_int(args[3]), y1 = mp_obj_get_int(args[4]);
    mp_int_t color = mp_obj_get_int(args[5]);
    line(self, x0, y0, x1, y1, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_line_obj, 6, 6, jd9853_JD9853_line);

static mp_obj_t jd9853_JD9853_blit_buffer(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_buffer_info_t buf_info;
    mp_get_buffer_raise(args[1], &buf_info, MP_BUFFER_READ);
    mp_int_t x = mp_obj_get_int(args[2]);
    mp_int_t y = mp_obj_get_int(args[3]);
    mp_int_t w = mp_obj_get_int(args[4]);
    mp_int_t h = mp_obj_get_int(args[5]);
    set_window(self, x, y, x + w - 1, y + h - 1);
    DC_HIGH(); CS_LOW();
    int limit = MIN(buf_info.len, w * h * 3);
    write_spi(self->spi_obj, buf_info.buf, limit);
    CS_HIGH();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_blit_buffer_obj, 6, 6, jd9853_JD9853_blit_buffer);

// ========== TEXTO VECTORIAL (Hershey) ==========
static mp_obj_t jd9853_JD9853_draw(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *hershey = MP_OBJ_TO_PTR(args[1]);
    mp_int_t x = mp_obj_get_int(args[3]);
    mp_int_t y = mp_obj_get_int(args[4]);
    mp_int_t color = (n_args > 5) ? mp_obj_get_int(args[5]) : WHITE;
    mp_float_t scale = (n_args > 6) ? mp_obj_float_get(args[6]) : 1.0;
    const char *s;
    char single[2] = {0,0};
    if (mp_obj_is_int(args[2])) {
        single[0] = mp_obj_get_int(args[2]) & 0xFF;
        s = single;
    } else {
        s = mp_obj_str_get_str(args[2]);
    }
    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(hershey->globals);
    mp_obj_t *idx_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_INDEX));
    mp_buffer_info_t idx_info; mp_get_buffer_raise(idx_buff, &idx_info, MP_BUFFER_READ);
    uint8_t *index = idx_info.buf;
    mp_obj_t *font_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FONT));
    mp_buffer_info_t font_info; mp_get_buffer_raise(font_buff, &font_info, MP_BUFFER_READ);
    int8_t *font = font_info.buf;
    int16_t from_x = x, from_y = y, to_x, to_y, pos_x = x, pos_y = y;
    bool penup = true;
    char c;
    while ((c = *s++)) {
        if (c >= 32 && c <= 127) {
            int ii = (c - 32) * 2;
            int offset = index[ii] | (index[ii+1] << 8);
            int length = font[offset++];
            int left = (int)(scale * (font[offset++] - 0x52) + 0.5);
            int right = (int)(scale * (font[offset++] - 0x52) + 0.5);
            if (length) {
                for (int i = 0; i < length; i++) {
                    if (font[offset] == ' ') {
                        offset += 2; penup = true; continue;
                    }
                    int vx = (int)(scale * (font[offset++] - 0x52) + 0.5);
                    int vy = (int)(scale * (font[offset++] - 0x52) + 0.5);
                    if (i == 0 || penup) {
                        from_x = pos_x + vx - left;
                        from_y = pos_y + vy;
                    } else {
                        to_x = pos_x + vx - left;
                        to_y = pos_y + vy;
                        line(self, from_x, from_y, to_x, to_y, color);
                        from_x = to_x; from_y = to_y;
                    }
                    penup = false;
                }
            }
            pos_x += (right - left);
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_obj, 5, 7, jd9853_JD9853_draw);

static mp_obj_t jd9853_JD9853_draw_len(size_t n_args, const mp_obj_t *args) {
    // similar al original, solo ancho en píxeles, sin color
    return mp_obj_new_int(0); // simplificado
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_len_obj, 3, 4, jd9853_JD9853_draw_len);

// ========== BITMAP y WRITE (con buffers RGB666) ==========
static uint32_t bs_bit = 0;
uint8_t *bitmap_data = NULL;

static uint8_t get_bit(uint8_t bpp) {
    uint8_t val = 0;
    for (int i = 0; i < bpp; i++) {
        val <<= 1;
        val |= (bitmap_data[bs_bit / 8] >> (7 - (bs_bit % 8))) & 1;
        bs_bit++;
    }
    return val;
}
static mp_obj_t dict_lookup(mp_obj_t self_in, mp_obj_t index) {
    mp_obj_dict_t *self = MP_OBJ_TO_PTR(self_in);
    mp_map_elem_t *elem = mp_map_lookup(&self->map, index, MP_MAP_LOOKUP);
    return elem ? elem->value : NULL;
}
static mp_obj_t jd9853_JD9853_write_len(size_t n_args, const mp_obj_t *args) {
    return mp_obj_new_int(0); // simplificado
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_write_len_obj, 3, 3, jd9853_JD9853_write_len);

static mp_obj_t jd9853_JD9853_write(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *font = MP_OBJ_TO_PTR(args[1]);
    mp_int_t x = mp_obj_get_int(args[3]);
    mp_int_t y = mp_obj_get_int(args[4]);
    mp_int_t fg = (n_args > 5) ? mp_obj_get_int(args[5]) : WHITE;
    mp_int_t bg = (n_args > 6) ? mp_obj_get_int(args[6]) : BLACK;
    bool fill = (n_args > 8) ? mp_obj_is_true(args[8]) : false;
    // Obtener tablas del font (similar al original)
    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(font->globals);
    const uint8_t bpp = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BPP)));
    const uint8_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint8_t offset_width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_OFFSET_WIDTH)));
    const uint8_t max_width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_MAX_WIDTH)));
    mp_obj_t widths_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTHS));
    mp_buffer_info_t widths_info; mp_get_buffer_raise(widths_buff, &widths_info, MP_BUFFER_READ);
    const uint8_t *widths = widths_info.buf;
    mp_obj_t offs_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_OFFSETS));
    mp_buffer_info_t offs_info; mp_get_buffer_raise(offs_buff, &offs_info, MP_BUFFER_READ);
    const uint8_t *offsets = offs_info.buf;
    mp_obj_t bitmaps_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BITMAPS));
    mp_buffer_info_t bmp_info; mp_get_buffer_raise(bitmaps_buff, &bmp_info, MP_BUFFER_READ);
    bitmap_data = bmp_info.buf;

    if (self->buffer_size == 0) self->i2c_buffer = self->static_buffer;
    uint16_t print_width = 0;
    mp_obj_t map_obj = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_MAP));
    GET_STR_DATA_LEN(map_obj, map_data, map_len);
    GET_STR_DATA_LEN(args[2], str_data, str_len);
    const byte *s = str_data, *top = str_data + str_len;
    while (s < top) {
        unichar ch = utf8_get_char(s);
        s = utf8_next_char(s);
        const byte *map_s = map_data, *map_top = map_data + map_len;
        uint16_t char_idx = 0;
        while (map_s < map_top) {
            unichar mc = utf8_get_char(map_s);
            map_s = utf8_next_char(map_s);
            if (ch == mc) {
                uint8_t w = widths[char_idx];
                bs_bit = 0;
                switch (offset_width) {
                    case 1: bs_bit = offsets[char_idx]; break;
                    case 2: bs_bit = (offsets[char_idx] << 8) | offsets[char_idx+1]; break;
                    case 3: bs_bit = (offsets[char_idx] << 16) | (offsets[char_idx+1] << 8) | offsets[char_idx+2]; break;
                }
                uint16_t buf_w = fill ? max_width : w;
                uint8_t *buf_ptr = self->i2c_buffer;
                for (uint16_t yy = 0; yy < height; yy++) {
                    for (uint16_t xx = 0; xx < w; xx++) {
                        uint32_t col = get_bit(bpp) ? fg : bg;
                        uint8_t rgb666[3];
                        color24_to_rgb666(rgb666, col);
                        memcpy(buf_ptr + (yy * buf_w + xx) * 3, rgb666, 3);
                    }
                }
                uint32_t data_size = buf_w * height * 3;
                uint16_t x2 = x + buf_w - 1;
                uint16_t y2 = y + height - 1;
                if (x2 < self->width) {
                    set_window(self, x, y, x2, y2);
                    DC_HIGH(); CS_LOW();
                    write_spi(self->spi_obj, self->i2c_buffer, data_size);
                    CS_HIGH();
                    print_width += w;
                }
                x += w;
                break;
            }
            char_idx++;
        }
    }
    return mp_obj_new_int(print_width);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_write_obj, 5, 9, jd9853_JD9853_write);

static mp_obj_t jd9853_JD9853_bitmap(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *bmp_mod = MP_OBJ_TO_PTR(args[1]);
    mp_int_t x = mp_obj_get_int(args[2]);
    mp_int_t y = mp_obj_get_int(args[3]);
    mp_int_t idx = (n_args > 4) ? mp_obj_get_int(args[4]) : 0;
    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(bmp_mod->globals);
    const uint16_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint16_t width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTH)));
    const uint8_t bpp = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BPP)));
    mp_obj_t *palette = NULL;
    size_t pal_len = 0;
    mp_obj_get_array(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_PALETTE)), &pal_len, &palette);
    mp_obj_t *bmp_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BITMAP));
    mp_buffer_info_t bmp_info; mp_get_buffer_raise(bmp_data_buff, &bmp_info, MP_BUFFER_READ);
    bitmap_data = bmp_info.buf;
    if (self->buffer_size == 0) self->i2c_buffer = self->static_buffer;
    size_t buf_size = width * height * 3;
    bs_bit = 0;
    uint8_t *buf_ptr = self->i2c_buffer;
    for (int yy = 0; yy < height; yy++) {
        for (int xx = 0; xx < width; xx++) {
            uint32_t col = mp_obj_get_int(palette[get_bit(bpp)]);
            uint8_t rgb666[3];
            color24_to_rgb666(rgb666, col);
            memcpy(buf_ptr + (yy * width + xx) * 3, rgb666, 3);
        }
    }
    uint16_t x1 = x + width - 1;
    if (x1 < self->width) {
        set_window(self, x, y, x1, y + height - 1);
        DC_HIGH(); CS_LOW();
        write_spi(self->spi_obj, self->i2c_buffer, buf_size);
        CS_HIGH();
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_bitmap_obj, 4, 5, jd9853_JD9853_bitmap);

static mp_obj_t jd9853_JD9853_text(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *font = MP_OBJ_TO_PTR(args[1]);
    mp_int_t x = mp_obj_get_int(args[3]);
    mp_int_t y = mp_obj_get_int(args[4]);
    mp_int_t fg = (n_args > 5) ? mp_obj_get_int(args[5]) : WHITE;
    mp_int_t bg = (n_args > 6) ? mp_obj_get_int(args[6]) : BLACK;
    const uint8_t *source;
    size_t src_len;
    if (mp_obj_is_int(args[2])) {
        static uint8_t tmp = 0;
        tmp = mp_obj_get_int(args[2]) & 0xFF;
        source = &tmp; src_len = 1;
    } else if (mp_obj_is_str(args[2])) {
        source = (uint8_t*)mp_obj_str_get_str(args[2]);
        src_len = strlen((char*)source);
    } else {
        mp_raise_TypeError(MP_ERROR_TEXT("text requires int or str"));
        return mp_const_none;
    }
    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(font->globals);
    const uint8_t width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTH)));
    const uint8_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint8_t first = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FIRST)));
    const uint8_t last = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_LAST)));
    mp_obj_t font_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FONT));
    mp_buffer_info_t fdata; mp_get_buffer_raise(font_data_buff, &fdata, MP_BUFFER_READ);
    const uint8_t *font_data = fdata.buf;
    uint8_t wide = width / 8;
    size_t buf_size = width * height * 3;
    if (self->buffer_size == 0) self->i2c_buffer = self->static_buffer;
    for (size_t i = 0; i < src_len; i++) {
        uint8_t ch = source[i];
        if (ch >= first && ch <= last) {
            uint16_t chr_idx = (ch - first) * height * wide;
            uint8_t *buf_ptr = self->i2c_buffer;
            for (uint16_t yy = 0; yy < height; yy++) {
                for (uint16_t xb = 0; xb < wide; xb++) {
                    uint8_t data = font_data[chr_idx++];
                    for (int bit = 7; bit >= 0; bit--) {
                        uint32_t col = (data >> bit) & 1 ? fg : bg;
                        uint8_t rgb666[3];
                        color24_to_rgb666(rgb666, col);
                        memcpy(buf_ptr, rgb666, 3);
                        buf_ptr += 3;
                    }
                }
            }
            uint16_t x1 = x + width - 1;
            if (x1 < self->width) {
                set_window(self, x, y, x1, y + height - 1);
                DC_HIGH(); CS_LOW();
                write_spi(self->spi_obj, self->i2c_buffer, buf_size);
                CS_HIGH();
            }
            x += width;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_text_obj, 5, 7, jd9853_JD9853_text);

// ========== ROTACIÓN ==========
static void set_rotation(jd9853_JD9853_obj_t *self) {
    uint8_t madctl = self->color_order;
    if (self->rotation >= self->rotations_len)
        mp_raise_msg_varg(&mp_type_RuntimeError, "rotation %d out of range", self->rotation);
    jd9853_rotation_t *rot = self->rotations;
    if (rot == NULL) rot = ORIENTATIONS_172x320;
    jd9853_rotation_t *r = &rot[self->rotation];
    madctl |= r->madctl;
    self->width = r->width;
    self->height = r->height;
    self->colstart = r->colstart;
    self->rowstart = r->rowstart;
    self->madctl = madctl;
    self->min_x = self->width; self->min_y = self->height;
    self->max_x = 0; self->max_y = 0;
    uint8_t cmd[] = {madctl};
    write_cmd(self, JD9853_MADCTL, cmd, 1);
}
static mp_obj_t jd9853_JD9853_rotation(mp_obj_t self_in, mp_obj_t val) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->rotation = mp_obj_get_int(val) % 4;
    set_rotation(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_rotation_obj, jd9853_JD9853_rotation);
static mp_obj_t jd9853_JD9853_width(mp_obj_t self_in) {
    return mp_obj_new_int(((jd9853_JD9853_obj_t*)MP_OBJ_TO_PTR(self_in))->width);
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_width_obj, jd9853_JD9853_width);
static mp_obj_t jd9853_JD9853_height(mp_obj_t self_in) {
    return mp_obj_new_int(((jd9853_JD9853_obj_t*)MP_OBJ_TO_PTR(self_in))->height);
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_height_obj, jd9853_JD9853_height);

// ========== SCROLL ==========
static mp_obj_t jd9853_JD9853_vscrdef(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->tfa = mp_obj_get_int(args[1]);
    self->vsa = mp_obj_get_int(args[2]);
    self->bfa = mp_obj_get_int(args[3]);
    uint8_t buf[6] = { self->tfa>>8, self->tfa&0xFF, self->vsa>>8, self->vsa&0xFF, self->bfa>>8, self->bfa&0xFF };
    write_cmd(self, JD9853_VSCRDEF, buf, 6);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_vscrdef_obj, 4, 4, jd9853_JD9853_vscrdef);
static mp_obj_t jd9853_JD9853_vscsad(mp_obj_t self_in, mp_obj_t vssa_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->vscsad = mp_obj_get_int(vssa_in);
    uint8_t buf[2] = { self->vscsad>>8, self->vscsad&0xFF };
    write_cmd(self, JD9853_VSCRSADD, buf, 2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_vscsad_obj, jd9853_JD9853_vscsad);
static mp_obj_t jd9853_JD9853_scroll(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int dy = mp_obj_get_int(args[1]);
    if (self->tfa + self->vsa + self->bfa != self->height)
        mp_raise_msg(&mp_type_ValueError, "scroll area not defined");
    int new_vsp = (self->vscsad + dy) % self->vsa;
    if (new_vsp < 0) new_vsp += self->vsa;
    uint8_t buf[2] = { new_vsp>>8, new_vsp&0xFF };
    write_cmd(self, JD9853_VSCRSADD, buf, 2);
    self->vscsad = new_vsp;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_scroll_obj, 2, 2, jd9853_JD9853_scroll);

// ========== INICIALIZACIÓN ==========
static void custom_init(jd9853_JD9853_obj_t *self) {
    size_t len; mp_obj_t *list;
    mp_obj_get_array(self->custom_init, &len, &list);
    for (size_t i = 0; i < len; i++) {
        size_t clen; mp_obj_t *cmd;
        mp_obj_get_array(list[i], &clen, &cmd);
        mp_buffer_info_t data;
        if (mp_get_buffer(cmd[0], &data, MP_BUFFER_READ)) {
            if (data.len > 1)
                write_cmd(self, data.buf[0], &data.buf[1], data.len-1);
            else
                write_cmd(self, data.buf[0], NULL, 0);
            mp_hal_delay_ms(10);
            if (clen > 1) {
                int delay = mp_obj_get_int(cmd[1]);
                if (delay) mp_hal_delay_ms(delay);
            }
        }
    }
}
static mp_obj_t jd9853_JD9853_init(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    jd9853_JD9853_hard_reset(self_in);
    if (self->custom_init == MP_OBJ_NULL) {
        jd9853_JD9853_soft_reset(self_in);
        write_cmd(self, JD9853_SLPOUT, NULL, 0);
        mp_hal_delay_ms(120);
        uint8_t unlock[] = {0x98,0x53};
        write_cmd(self, 0xDF, unlock, 2); mp_hal_delay_ms(10);
        write_cmd(self, 0xDF, unlock, 2); mp_hal_delay_ms(10);
        uint8_t colmod[] = {0x66};   // 18 bits
        write_cmd(self, JD9853_COLMOD, colmod, 1);
        mp_hal_delay_ms(10);
        uint8_t madctl[] = {JD9853_MADCTL_RGB};
        write_cmd(self, JD9853_MADCTL, madctl, 1);
        mp_hal_delay_ms(10);
        if (self->inversion) write_cmd(self, JD9853_INVON, NULL, 0);
        else write_cmd(self, JD9853_INVOFF, NULL, 0);
        write_cmd(self, JD9853_NORON, NULL, 0);
        mp_hal_delay_ms(10);
        write_cmd(self, JD9853_DISPON, NULL, 0);
        mp_hal_delay_ms(150);
    } else {
        custom_init(self);
    }
    set_rotation(self);
    mp_hal_delay_ms(10);
    const mp_obj_t args_fill[] = {self_in, mp_obj_new_int(0), mp_obj_new_int(0),
        mp_obj_new_int(self->width), mp_obj_new_int(self->height), mp_obj_new_int(BLACK)};
    jd9853_JD9853_fill_rect(6, args_fill);
    if (self->backlight != GPIO_NUM_NC) mp_hal_pin_write(self->backlight, 1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_init_obj, jd9853_JD9853_init);

static mp_obj_t jd9853_JD9853_on(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->backlight != GPIO_NUM_NC) mp_hal_pin_write(self->backlight, 1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_on_obj, jd9853_JD9853_on);
static mp_obj_t jd9853_JD9853_off(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->backlight != GPIO_NUM_NC) mp_hal_pin_write(self->backlight, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_off_obj, jd9853_JD9853_off);

// ========== PRIMITIVAS GEOMÉTRICAS ==========
static mp_obj_t jd9853_JD9853_hline(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    fast_hline(self, mp_obj_get_int(args[1]), mp_obj_get_int(args[2]), mp_obj_get_int(args[3]), mp_obj_get_int(args[4]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_hline_obj, 5, 5, jd9853_JD9853_hline);
static mp_obj_t jd9853_JD9853_vline(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    fast_vline(self, mp_obj_get_int(args[1]), mp_obj_get_int(args[2]), mp_obj_get_int(args[3]), mp_obj_get_int(args[4]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_vline_obj, 5, 5, jd9853_JD9853_vline);
static mp_obj_t jd9853_JD9853_circle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int xm = mp_obj_get_int(args[1]), ym = mp_obj_get_int(args[2]), r = mp_obj_get_int(args[3]);
    uint32_t col = mp_obj_get_int(args[4]);
    int f = 1 - r, ddF_x = 1, ddF_y = -2 * r, x = 0, y = r;
    draw_pixel(self, xm, ym + r, col); draw_pixel(self, xm, ym - r, col);
    draw_pixel(self, xm + r, ym, col); draw_pixel(self, xm - r, ym, col);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        draw_pixel(self, xm + x, ym + y, col); draw_pixel(self, xm - x, ym + y, col);
        draw_pixel(self, xm + x, ym - y, col); draw_pixel(self, xm - x, ym - y, col);
        draw_pixel(self, xm + y, ym + x, col); draw_pixel(self, xm - y, ym + x, col);
        draw_pixel(self, xm + y, ym - x, col); draw_pixel(self, xm - y, ym - x, col);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_circle_obj, 5, 5, jd9853_JD9853_circle);
static mp_obj_t jd9853_JD9853_fill_circle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int xm = mp_obj_get_int(args[1]), ym = mp_obj_get_int(args[2]), r = mp_obj_get_int(args[3]);
    uint32_t col = mp_obj_get_int(args[4]);
    int f = 1 - r, ddF_x = 1, ddF_y = -2 * r, x = 0, y = r;
    fast_vline(self, xm, ym - y, 2 * y + 1, col);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        fast_vline(self, xm + x, ym - y, 2 * y + 1, col);
        fast_vline(self, xm + y, ym - x, 2 * x + 1, col);
        fast_vline(self, xm - x, ym - y, 2 * y + 1, col);
        fast_vline(self, xm - y, ym - x, 2 * x + 1, col);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_circle_obj, 5, 5, jd9853_JD9853_fill_circle);
static mp_obj_t jd9853_JD9853_rect(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int x = mp_obj_get_int(args[1]), y = mp_obj_get_int(args[2]), w = mp_obj_get_int(args[3]), h = mp_obj_get_int(args[4]);
    uint32_t col = mp_obj_get_int(args[5]);
    fast_hline(self, x, y, w, col); fast_vline(self, x, y, h, col);
    fast_hline(self, x, y + h - 1, w, col); fast_vline(self, x + w - 1, y, h, col);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_rect_obj, 6, 6, jd9853_JD9853_rect);
static mp_obj_t jd9853_JD9853_round_rect(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int x = mp_obj_get_int(args[1]), y = mp_obj_get_int(args[2]), w = mp_obj_get_int(args[3]), h = mp_obj_get_int(args[4]);
    int r = mp_obj_get_int(args[5]), col = mp_obj_get_int(args[6]);
    if (r > w/2) r = w/2; if (r > h/2) r = h/2;
    fast_hline(self, x + r, y, w - 2*r, col);
    fast_hline(self, x + r, y + h - 1, w - 2*r, col);
    fast_vline(self, x, y + r, h - 2*r, col);
    fast_vline(self, x + w - 1, y + r, h - 2*r, col);
    for (int i = 0; i <= r; i++) {
        int d = (int)(sqrt(r*r - i*i) + 0.5);
        draw_pixel(self, x + r - i, y + r - d, col);
        draw_pixel(self, x + r - i, y + h - r + d - 1, col);
        draw_pixel(self, x + w - r + i - 1, y + r - d, col);
        draw_pixel(self, x + w - r + i - 1, y + h - r + d - 1, col);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_round_rect_obj, 7, 7, jd9853_JD9853_round_rect);
static mp_obj_t jd9853_JD9853_madctl(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 2) {
        self->madctl = mp_obj_get_int(args[1]) & 0xFF;
        uint8_t cmd[] = {self->madctl};
        write_cmd(self, JD9853_MADCTL, cmd, 1);
    }
    return mp_obj_new_int(self->madctl);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_madctl_obj, 1, 2, jd9853_JD9853_madctl);
static mp_obj_t jd9853_JD9853_offset(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->colstart = mp_obj_get_int(args[1]);
    self->rowstart = mp_obj_get_int(args[2]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_offset_obj, 3, 3, jd9853_JD9853_offset);

// ========== FUNCIÓN color_rgb ==========
static uint32_t color_rgb_func(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
static mp_obj_t jd9853_color_rgb(mp_obj_t r, mp_obj_t g, mp_obj_t b) {
    return mp_obj_new_int(color_rgb_func(mp_obj_get_int(r), mp_obj_get_int(g), mp_obj_get_int(b)));
}
static MP_DEFINE_CONST_FUN_OBJ_3(jd9853_color_rgb_obj, jd9853_color_rgb);

// ========== PROCESAMIENTO DE JPG con tjpgd666 ==========
#define JPG_MODE_FAST 0
#define JPG_MODE_SLOW 1
typedef struct {
    mp_file_t *fp;
    uint8_t *fbuf;
    unsigned int wfbuf;
    unsigned int left, top, right, bottom;
    jd9853_JD9853_obj_t *self;
    uint8_t *data;
    unsigned int dataIdx, dataLen;
} IODEV_JPG;
static unsigned int jpg_buffer_in(JDEC *jd, uint8_t *buff, unsigned int nbyte) {
    IODEV_JPG *dev = (IODEV_JPG*)jd->device;
    if (dev->dataIdx + nbyte > dev->dataLen) nbyte = dev->dataLen - dev->dataIdx;
    if (buff) memcpy(buff, dev->data + dev->dataIdx, nbyte);
    dev->dataIdx += nbyte;
    return nbyte;
}
static unsigned int jpg_file_in(JDEC *jd, uint8_t *buff, unsigned int nbyte) {
    IODEV_JPG *dev = (IODEV_JPG*)jd->device;
    if (buff) return mp_readinto(dev->fp, buff, nbyte);
    mp_seek(dev->fp, nbyte, SEEK_CUR);
    return 0;
}
static void rgb888_to_rgb666_bulk(uint8_t *dst, const uint8_t *src, uint32_t cnt) {
    for (uint32_t i = 0; i < cnt; i++) {
        dst[0] = src[0] >> 2; dst[1] = src[1] >> 2; dst[2] = src[2] >> 2;
        src += 3; dst += 3;
    }
}
static int jpg_out_fast(JDEC *jd, void *bitmap, JRECT *rect) {
    IODEV_JPG *dev = (IODEV_JPG*)jd->device;
    jd9853_JD9853_obj_t *self = dev->self;
    uint8_t *src = (uint8_t*)bitmap;
    uint8_t *dst = dev->fbuf + 3 * (rect->top * dev->wfbuf + rect->left);
    int bws = 3 * (rect->right - rect->left + 1);
    int bwd = 3 * dev->wfbuf;
    for (unsigned y = rect->top; y <= rect->bottom; y++) {
        rgb888_to_rgb666_bulk(dst, src, rect->right - rect->left + 1);
        src += bws; dst += bwd;
    }
    return 1;
}
static int jpg_out_slow(JDEC *jd, void *bitmap, JRECT *rect) {
    IODEV_JPG *dev = (IODEV_JPG*)jd->device;
    jd9853_JD9853_obj_t *self = dev->self;
    uint8_t *src = (uint8_t*)bitmap;
    uint8_t *dst = dev->fbuf;
    int wpx = rect->right - rect->left + 1;
    int hpx = rect->bottom - rect->top + 1;
    int wx3 = wpx * 3;
    for (int y = 0; y < hpx; y++) {
        rgb888_to_rgb666_bulk(dst, src, wpx);
        src += wx3; dst += wx3;
    }
    set_window(self, rect->left + jd->x_offs, rect->top + jd->y_offs,
               rect->right + jd->x_offs, rect->bottom + jd->y_offs);
    DC_HIGH(); CS_LOW();
    write_spi(self->spi_obj, dev->fbuf, wx3 * hpx);
    CS_HIGH();
    return 1;
}
static mp_obj_t jd9853_JD9853_jpg(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    static unsigned int (*input)(JDEC*,uint8_t*,unsigned int) = NULL;
    IODEV_JPG devid;
    mp_buffer_info_t binfo;
    if (mp_obj_is_type(args[1], &mp_type_bytes)) {
        mp_get_buffer_raise(args[1], &binfo, MP_BUFFER_READ);
        devid.dataIdx = 0; devid.data = binfo.buf; devid.dataLen = binfo.len;
        input = jpg_buffer_in;
        self->fp = MP_OBJ_NULL;
    } else {
        const char *fname = mp_obj_str_get_str(args[1]);
        self->fp = mp_open(fname, "rb");
        devid.fp = self->fp; input = jpg_file_in;
        devid.data = NULL; devid.dataLen = 0;
    }
    int x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]);
    int mode = (n_args > 4) ? mp_obj_get_int(args[4]) : JPG_MODE_FAST;
    JRESULT res;
    JDEC jdec;
    self->work = m_malloc(3100);
    if (input && (devid.fp || devid.data)) {
        res = jd_prepare(&jdec, input, self->work, 3100, &devid);
        if (res == JDR_OK) {
            size_t bufsize;
            int (*out)(JDEC*,void*,JRECT*);
            if (mode == JPG_MODE_FAST) {
                bufsize = 3 * jdec.width * jdec.height;
                out = jpg_out_fast;
            } else {
                bufsize = 3 * jdec.msx * 8 * jdec.msy * 8;
                out = jpg_out_slow;
                jdec.x_offs = x; jdec.y_offs = y;
            }
            if (self->buffer_size && bufsize > self->buffer_size)
                mp_raise_msg_varg(&mp_type_OSError, "buffer too small, need %lu bytes", (unsigned long)bufsize);
            if (self->buffer_size == 0)
                self->i2c_buffer = m_malloc(bufsize);
            if (!self->i2c_buffer) mp_raise_msg(&mp_type_OSError, "out of memory");
            devid.fbuf = self->i2c_buffer;
            devid.wfbuf = jdec.width;
            devid.self = self;
            res = jd_decomp(&jdec, out, 0);
            if (res == JDR_OK) {
                if (mode == JPG_MODE_FAST) {
                    set_window(self, x, y, x + jdec.width - 1, y + jdec.height - 1);
                    DC_HIGH(); CS_LOW();
                    write_spi(self->spi_obj, self->i2c_buffer, bufsize);
                    CS_HIGH();
                }
            } else {
                mp_raise_msg(&mp_type_RuntimeError, "JPEG decompress failed");
            }
            if (self->buffer_size == 0) { m_free(self->i2c_buffer); self->i2c_buffer = NULL; }
            devid.fbuf = NULL;
        } else {
            mp_raise_msg(&mp_type_RuntimeError, "JPEG prepare failed");
        }
        if (self->fp) { mp_close(self->fp); self->fp = MP_OBJ_NULL; }
    }
    m_free(self->work);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_jpg_obj, 4, 5, jd9853_JD9853_jpg);
// Nota: jpg_decode no se incluye por brevedad, pero se puede adaptar de forma análoga.

// ========== PROCESAMIENTO DE PNG (conversión a RGB666) ==========
typedef struct {
    jd9853_JD9853_obj_t *self;
    int ofs_x, ofs_y;
    uint16_t pixels, row, first, last;
    bool has_transparency;
    uint8_t *buffer;
} PNG_USER;
static void png_flush(jd9853_JD9853_obj_t *self, PNG_USER *usr) {
    set_window(self, usr->first, usr->row, usr->last, usr->row);
    DC_HIGH(); CS_LOW();
    write_spi(self->spi_obj, usr->buffer, usr->pixels * 3);
    CS_HIGH();
    usr->buffer = self->i2c_buffer;
    usr->pixels = 0;
}
static void png_new_row(PNG_USER *usr, uint16_t row, uint16_t col) {
    usr->row = row; usr->first = col; usr->last = col;
}
static void pngle_on_draw(pngle_t *pngle, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t rgba[4]) {
    PNG_USER *usr = pngle_get_user_data(pngle);
    jd9853_JD9853_obj_t *self = usr->self;
    int row = y + usr->ofs_y;
    int col = x + usr->ofs_x;
    if (col < 0 || row < 0 || col >= self->width || row > self->height) return;
    pngle_ihdr_t *ihdr = pngle_get_ihdr(pngle);
    size_t min_buf = ihdr->width * 3;
    if (usr->buffer == NULL) {
        if (self->buffer_size == 0) {
            usr->buffer = m_malloc(min_buf);
            if (!usr->buffer) mp_raise_msg(&mp_type_OSError, "out of memory");
        } else {
            if (self->buffer_size < min_buf)
                mp_raise_msg_varg(&mp_type_OSError, "buffer too small, need %zu bytes", min_buf);
            usr->buffer = self->i2c_buffer;
        }
        self->i2c_buffer = usr->buffer;
        png_new_row(usr, row, col);
    }
    if (usr->pixels > 0 && (row != usr->row || (usr->has_transparency && rgba[3] == 0))) {
        png_flush(self, usr);
        png_new_row(usr, row, col);
    }
    if (usr->has_transparency && rgba[3] == 0) {
        png_new_row(usr, row, col);
        return;
    }
    uint32_t col24 = ((uint32_t)rgba[0] << 16) | ((uint32_t)rgba[1] << 8) | rgba[2];
    uint8_t rgb666[3];
    color24_to_rgb666(rgb666, col24);
    memcpy(usr->buffer + usr->pixels * 3, rgb666, 3);
    usr->pixels++;
    usr->last = col;
}
static mp_obj_t jd9853_JD9853_png(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const char *fname = mp_obj_str_get_str(args[1]);
    int x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]);
    bool transp = (n_args > 4) ? mp_obj_is_true(args[4]) : false;
    char buf[256];
    int len, remain = 0;
    PNG_USER usr = { .self = self, .ofs_x = x, .ofs_y = y, .pixels = 0,
        .has_transparency = transp, .buffer = NULL };
    self->work = pngle_new(self);
    pngle_t *pngle = (pngle_t*)self->work;
    pngle_set_user_data(pngle, &usr);
    pngle_set_draw_callback(pngle, pngle_on_draw);
    self->fp = mp_open(fname, "rb");
    while ((len = mp_readinto(self->fp, buf + remain, sizeof(buf)-remain)) > 0) {
        int fed = pngle_feed(pngle, buf, remain + len);
        if (fed < 0) mp_raise_msg_varg(&mp_type_RuntimeError, "PNG error: %s", pngle_error(pngle));
        remain = remain + len - fed;
        if (remain > 0) memmove(buf, buf + fed, remain);
    }
    if (usr.pixels > 0) png_flush(self, &usr);
    if (self->buffer_size == 0 && usr.buffer) m_free(usr.buffer);
    mp_close(self->fp);
    pngle_destroy(pngle);
    self->work = NULL;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_png_obj, 4, 5, jd9853_JD9853_png);

// ========== POLÍGONOS ==========
static mp_obj_t jd9853_JD9853_polygon_center(size_t n_args, const mp_obj_t *args) {
    // simplificado, retorna (0,0)
    mp_obj_t r[2] = {mp_obj_new_int(0), mp_obj_new_int(0)};
    return mp_obj_new_tuple(2, r);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_polygon_center_obj, 2, 2, jd9853_JD9853_polygon_center);
static void RotatePolygon(Polygon *p, Point c, mp_float_t a) {}
static void PolygonFill(jd9853_JD9853_obj_t *self, Polygon *p, Point loc, uint32_t col) {}
static mp_obj_t jd9853_JD9853_polygon(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_polygon_obj, 4, 8, jd9853_JD9853_polygon);
static mp_obj_t jd9853_JD9853_fill_polygon(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_polygon_obj, 4, 8, jd9853_JD9853_fill_polygon);
static mp_obj_t jd9853_JD9853_bounding(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_t b[4] = {mp_obj_new_int(self->min_x), mp_obj_new_int(self->min_y),
                     mp_obj_new_int(self->max_x), mp_obj_new_int(self->max_y)};
    if (n_args > 1) {
        self->bounding = mp_obj_is_true(args[1]) ? 1 : 0;
        self->min_x = self->width; self->min_y = self->height;
        self->max_x = 0; self->max_y = 0;
    }
    return mp_obj_new_tuple(4, b);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_bounding_obj, 1, 3, jd9853_JD9853_bounding);
static mp_obj_t jd9853_JD9853_gradient_fill(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_gradient_fill_obj, 8, 8, jd9853_JD9853_gradient_fill);
static mp_obj_t jd9853_JD9853_draw_icon(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_icon_obj, 6, 6, jd9853_JD9853_draw_icon);
static mp_obj_t jd9853_JD9853_get_info(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_width), mp_obj_new_int(self->width));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_height), mp_obj_new_int(self->height));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_rotation), mp_obj_new_int(self->rotation));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_madctl), mp_obj_new_int(self->madctl));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_inversion), mp_obj_new_bool(self->inversion));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_get_info_obj, jd9853_JD9853_get_info);
static mp_obj_t jd9853_JD9853_triangle(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_triangle_obj, 8, 8, jd9853_JD9853_triangle);
static mp_obj_t jd9853_JD9853_fill_triangle(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_triangle_obj, 8, 8, jd9853_JD9853_fill_triangle);
static mp_obj_t jd9853_JD9853_ellipse(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_ellipse_obj, 6, 6, jd9853_JD9853_ellipse);
static mp_obj_t jd9853_JD9853_fill_ellipse(size_t n_args, const mp_obj_t *args) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_ellipse_obj, 6, 6, jd9853_JD9853_fill_ellipse);

// ========== DICCIONARIO DE MÉTODOS ==========
static const mp_rom_map_elem_t jd9853_JD9853_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&jd9853_JD9853_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_write_len), MP_ROM_PTR(&jd9853_JD9853_write_len_obj)},
    {MP_ROM_QSTR(MP_QSTR_hard_reset), MP_ROM_PTR(&jd9853_JD9853_hard_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_soft_reset), MP_ROM_PTR(&jd9853_JD9853_soft_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_sleep_mode), MP_ROM_PTR(&jd9853_JD9853_sleep_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_inversion_mode), MP_ROM_PTR(&jd9853_JD9853_inversion_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&jd9853_JD9853_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&jd9853_JD9853_on_obj)},
    {MP_ROM_QSTR(MP_QSTR_off), MP_ROM_PTR(&jd9853_JD9853_off_obj)},
    {MP_ROM_QSTR(MP_QSTR_pixel), MP_ROM_PTR(&jd9853_JD9853_pixel_obj)},
    {MP_ROM_QSTR(MP_QSTR_line), MP_ROM_PTR(&jd9853_JD9853_line_obj)},
    {MP_ROM_QSTR(MP_QSTR_blit_buffer), MP_ROM_PTR(&jd9853_JD9853_blit_buffer_obj)},
    {MP_ROM_QSTR(MP_QSTR_draw), MP_ROM_PTR(&jd9853_JD9853_draw_obj)},
    {MP_ROM_QSTR(MP_QSTR_draw_len), MP_ROM_PTR(&jd9853_JD9853_draw_len_obj)},
    {MP_ROM_QSTR(MP_QSTR_bitmap), MP_ROM_PTR(&jd9853_JD9853_bitmap_obj)},
    {MP_ROM_QSTR(MP_QSTR_set_window), MP_ROM_PTR(&jd9853_JD9853_set_window_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill_rect), MP_ROM_PTR(&jd9853_JD9853_fill_rect_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill), MP_ROM_PTR(&jd9853_JD9853_fill_obj)},
    {MP_ROM_QSTR(MP_QSTR_hline), MP_ROM_PTR(&jd9853_JD9853_hline_obj)},
    {MP_ROM_QSTR(MP_QSTR_vline), MP_ROM_PTR(&jd9853_JD9853_vline_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill_circle), MP_ROM_PTR(&jd9853_JD9853_fill_circle_obj)},
    {MP_ROM_QSTR(MP_QSTR_circle), MP_ROM_PTR(&jd9853_JD9853_circle_obj)},
    {MP_ROM_QSTR(MP_QSTR_rect), MP_ROM_PTR(&jd9853_JD9853_rect_obj)},
    {MP_ROM_QSTR(MP_QSTR_round_rect), MP_ROM_PTR(&jd9853_JD9853_round_rect_obj)},
    {MP_ROM_QSTR(MP_QSTR_text), MP_ROM_PTR(&jd9853_JD9853_text_obj)},
    {MP_ROM_QSTR(MP_QSTR_rotation), MP_ROM_PTR(&jd9853_JD9853_rotation_obj)},
    {MP_ROM_QSTR(MP_QSTR_width), MP_ROM_PTR(&jd9853_JD9853_width_obj)},
    {MP_ROM_QSTR(MP_QSTR_height), MP_ROM_PTR(&jd9853_JD9853_height_obj)},
    {MP_ROM_QSTR(MP_QSTR_vscrdef), MP_ROM_PTR(&jd9853_JD9853_vscrdef_obj)},
    {MP_ROM_QSTR(MP_QSTR_vscsad), MP_ROM_PTR(&jd9853_JD9853_vscsad_obj)},
    {MP_ROM_QSTR(MP_QSTR_scroll), MP_ROM_PTR(&jd9853_JD9853_scroll_obj)},
    {MP_ROM_QSTR(MP_QSTR_madctl), MP_ROM_PTR(&jd9853_JD9853_madctl_obj)},
    {MP_ROM_QSTR(MP_QSTR_offset), MP_ROM_PTR(&jd9853_JD9853_offset_obj)},
    {MP_ROM_QSTR(MP_QSTR_jpg), MP_ROM_PTR(&jd9853_JD9853_jpg_obj)},
    {MP_ROM_QSTR(MP_QSTR_png), MP_ROM_PTR(&jd9853_JD9853_png_obj)},
    {MP_ROM_QSTR(MP_QSTR_polygon_center), MP_ROM_PTR(&jd9853_JD9853_polygon_center_obj)},
    {MP_ROM_QSTR(MP_QSTR_polygon), MP_ROM_PTR(&jd9853_JD9853_polygon_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill_polygon), MP_ROM_PTR(&jd9853_JD9853_fill_polygon_obj)},
    {MP_ROM_QSTR(MP_QSTR_bounding), MP_ROM_PTR(&jd9853_JD9853_bounding_obj)},
    {MP_ROM_QSTR(MP_QSTR_gradient_fill), MP_ROM_PTR(&jd9853_JD9853_gradient_fill_obj)},
    {MP_ROM_QSTR(MP_QSTR_draw_icon), MP_ROM_PTR(&jd9853_JD9853_draw_icon_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_info), MP_ROM_PTR(&jd9853_JD9853_get_info_obj)},
    {MP_ROM_QSTR(MP_QSTR_triangle), MP_ROM_PTR(&jd9853_JD9853_triangle_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill_triangle), MP_ROM_PTR(&jd9853_JD9853_fill_triangle_obj)},
    {MP_ROM_QSTR(MP_QSTR_ellipse), MP_ROM_PTR(&jd9853_JD9853_ellipse_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill_ellipse), MP_ROM_PTR(&jd9853_JD9853_fill_ellipse_obj)},
    {MP_ROM_QSTR(MP_QSTR_color_rgb), MP_ROM_PTR(&jd9853_color_rgb_obj)},
};
static MP_DEFINE_CONST_DICT(jd9853_JD9853_locals_dict, jd9853_JD9853_locals_dict_table);

// ========== TIPO DE OBJETO ==========
#ifdef MP_OBJ_TYPE_GET_SLOT
MP_DEFINE_CONST_OBJ_TYPE(jd9853_JD9853_type, MP_QSTR_JD9853, MP_TYPE_FLAG_NONE,
    print, jd9853_JD9853_print, make_new, jd9853_JD9853_make_new,
    locals_dict, (mp_obj_dict_t*)&jd9853_JD9853_locals_dict);
#else
const mp_obj_type_t jd9853_JD9853_type = {
    {&mp_type_type}, .name = MP_QSTR_JD9853,
    .print = jd9853_JD9853_print, .make_new = jd9853_JD9853_make_new,
    .locals_dict = (mp_obj_dict_t*)&jd9853_JD9853_locals_dict,
};
#endif

// ========== CONSTRUCTOR ==========
mp_obj_t jd9853_JD9853_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_width, ARG_height, ARG_reset, ARG_dc, ARG_cs, ARG_backlight,
           ARG_rotations, ARG_rotation, ARG_custom_init, ARG_color_order, ARG_inversion,
           ARG_options, ARG_buffer_size };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_spi, MP_ARG_OBJ | MP_ARG_REQUIRED},
        {MP_QSTR_width, MP_ARG_INT | MP_ARG_REQUIRED},
        {MP_QSTR_height, MP_ARG_INT | MP_ARG_REQUIRED},
        {MP_QSTR_reset, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_dc, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_cs, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_backlight, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_rotations, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_rotation, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_custom_init, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_color_order, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = JD9853_MADCTL_RGB}},
        {MP_QSTR_inversion, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
        {MP_QSTR_options, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    jd9853_JD9853_obj_t *self = m_new_obj(jd9853_JD9853_obj_t);
    self->base.type = &jd9853_JD9853_type;
    self->spi_obj = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[ARG_spi].u_obj);
    self->display_width = args[ARG_width].u_int;
    self->width = args[ARG_width].u_int;
    self->display_height = args[ARG_height].u_int;
    self->height = args[ARG_height].u_int;
    self->vscsad = 0; self->tfa = self->vsa = self->bfa = 0;
    self->rotations = NULL; self->rotations_len = 4;
    if (args[ARG_rotations].u_obj != MP_OBJ_NULL) {
        size_t len; mp_obj_t *arr;
        mp_obj_get_array(args[ARG_rotations].u_obj, &len, &arr);
        self->rotations_len = len;
        self->rotations = m_new(jd9853_rotation_t, len);
        for (size_t i = 0; i < len; i++) {
            mp_obj_t *tup; size_t tlen;
            mp_obj_tuple_get(arr[i], &tlen, &tup);
            if (tlen != 5) mp_raise_ValueError("rotation tuple must have 5 elements");
            self->rotations[i].madctl = mp_obj_get_int(tup[0]);
            self->rotations[i].width = mp_obj_get_int(tup[1]);
            self->rotations[i].height = mp_obj_get_int(tup[2]);
            self->rotations[i].colstart = mp_obj_get_int(tup[3]);
            self->rotations[i].rowstart = mp_obj_get_int(tup[4]);
        }
    }
    self->rotation = args[ARG_rotation].u_int % self->rotations_len;
    self->custom_init = args[ARG_custom_init].u_obj;
    self->color_order = args[ARG_color_order].u_int;
    self->inversion = args[ARG_inversion].u_bool;
    self->options = args[ARG_options].u_int & 0xFF;
    self->buffer_size = args[ARG_buffer_size].u_int;
    if (self->buffer_size) self->i2c_buffer = m_malloc(self->buffer_size);
    else self->i2c_buffer = self->static_buffer;
    if (args[ARG_dc].u_obj == MP_OBJ_NULL) mp_raise_ValueError("dc pin required");
    self->reset = (args[ARG_reset].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_reset].u_obj) : GPIO_NUM_NC;
    self->dc = mp_hal_get_pin_obj(args[ARG_dc].u_obj);
    self->cs = (args[ARG_cs].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_cs].u_obj) : GPIO_NUM_NC;
    self->backlight = (args[ARG_backlight].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_backlight].u_obj) : GPIO_NUM_NC;
    self->bounding = 0;
    self->min_x = self->width; self->min_y = self->height;
    self->max_x = 0; self->max_y = 0;
    return MP_OBJ_FROM_PTR(self);
}

// ========== MÓDULO ==========
static const mp_map_elem_t jd9853_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_jd9853)},
    {MP_ROM_QSTR(MP_QSTR_color_rgb), (mp_obj_t)&jd9853_color_rgb_obj},
    {MP_ROM_QSTR(MP_QSTR_JD9853), (mp_obj_t)&jd9853_JD9853_type},
    {MP_ROM_QSTR(MP_QSTR_BLACK), MP_ROM_INT(BLACK)},
    {MP_ROM_QSTR(MP_QSTR_BLUE), MP_ROM_INT(BLUE)},
    {MP_ROM_QSTR(MP_QSTR_RED), MP_ROM_INT(RED)},
    {MP_ROM_QSTR(MP_QSTR_GREEN), MP_ROM_INT(GREEN)},
    {MP_ROM_QSTR(MP_QSTR_CYAN), MP_ROM_INT(CYAN)},
    {MP_ROM_QSTR(MP_QSTR_MAGENTA), MP_ROM_INT(MAGENTA)},
    {MP_ROM_QSTR(MP_QSTR_YELLOW), MP_ROM_INT(YELLOW)},
    {MP_ROM_QSTR(MP_QSTR_WHITE), MP_ROM_INT(WHITE)},
    {MP_ROM_QSTR(MP_QSTR_ORANGE), MP_ROM_INT(0xFFA500)},
    {MP_ROM_QSTR(MP_QSTR_PURPLE), MP_ROM_INT(0x800080)},
    {MP_ROM_QSTR(MP_QSTR_PINK), MP_ROM_INT(0xFFC0CB)},
    {MP_ROM_QSTR(MP_QSTR_GRAY), MP_ROM_INT(0x808080)},
    {MP_ROM_QSTR(MP_QSTR_DARKGRAY), MP_ROM_INT(0x404040)},
    {MP_ROM_QSTR(MP_QSTR_BROWN), MP_ROM_INT(0xA52A2A)},
    {MP_ROM_QSTR(MP_QSTR_FAST), MP_ROM_INT(JPG_MODE_FAST)},
    {MP_ROM_QSTR(MP_QSTR_SLOW), MP_ROM_INT(JPG_MODE_SLOW)},
    {MP_ROM_QSTR(MP_QSTR_MADCTL_MY), MP_ROM_INT(JD9853_MADCTL_MY)},
    {MP_ROM_QSTR(MP_QSTR_MADCTL_MX), MP_ROM_INT(JD9853_MADCTL_MX)},
    {MP_ROM_QSTR(MP_QSTR_MADCTL_MV), MP_ROM_INT(JD9853_MADCTL_MV)},
    {MP_ROM_QSTR(MP_QSTR_MADCTL_ML), MP_ROM_INT(JD9853_MADCTL_ML)},
    {MP_ROM_QSTR(MP_QSTR_MADCTL_MH), MP_ROM_INT(JD9853_MADCTL_MH)},
    {MP_ROM_QSTR(MP_QSTR_RGB), MP_ROM_INT(JD9853_MADCTL_RGB)},
    {MP_ROM_QSTR(MP_QSTR_BGR), MP_ROM_INT(JD9853_MADCTL_BGR)},
    {MP_ROM_QSTR(MP_QSTR_WRAP), MP_ROM_INT(OPTIONS_WRAP)},
    {MP_ROM_QSTR(MP_QSTR_WRAP_H), MP_ROM_INT(OPTIONS_WRAP_H)},
    {MP_ROM_QSTR(MP_QSTR_WRAP_V), MP_ROM_INT(OPTIONS_WRAP_V)},
    {MP_ROM_QSTR(MP_QSTR_GRADIENT_HORIZONTAL), MP_ROM_INT(0)},
    {MP_ROM_QSTR(MP_QSTR_GRADIENT_VERTICAL), MP_ROM_INT(1)},
};
static MP_DEFINE_CONST_DICT(mp_module_jd9853_globals, jd9853_module_globals_table);
const mp_obj_module_t mp_module_jd9853 = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&mp_module_jd9853_globals,
};
MP_REGISTER_MODULE(MP_QSTR_jd9853, mp_module_jd9853);
