/*
 * Modifications and additions Copyright (c) 2020, 2021 Russ Hughes
 * Enhanced for JD9853 18-bit color, ESP32-S3, 172x320 display
 *
 * This file licensed under the MIT License.
 */

#define __JD9853_VERSION__ "0.0.1"
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
#include "jpg/tjpgd565.h"
#include "png/pngle.h"

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#define _swap_bytes(val) ((((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00))

#define ABS(N) (((N) < 0) ? (-(N)) : (N))
#define mp_hal_delay_ms(delay) (mp_hal_delay_us(delay * 1000))

#ifndef GPIO_NUM_NC
  #ifdef  STM32_HAL_H
    #define GPIO_NUM_NC NULL
  #else
    #define GPIO_NUM_NC -1
  #endif
#endif

#define CS_LOW()                       \
{                                      \
    if (self->cs != GPIO_NUM_NC) {     \
        mp_hal_pin_write(self->cs, 0); \
    }                                  \
}

#define CS_HIGH()                      \
{                                      \
    if (self->cs != GPIO_NUM_NC) {     \
        mp_hal_pin_write(self->cs, 1); \
    }                                  \
}

#define DC_LOW() (mp_hal_pin_write(self->dc, 0))
#define DC_HIGH() (mp_hal_pin_write(self->dc, 1))

#define RESET_LOW()                         \
{                                           \
    if (self->reset != GPIO_NUM_NC) {       \
        mp_hal_pin_write(self->reset, 0);   \
    }                                       \
}

#define RESET_HIGH()                        \
{                                           \
    if (self->reset != GPIO_NUM_NC) {       \
        mp_hal_pin_write(self->reset, 1);   \
    }                                       \
}

// ========== MEJORA: buffer de transferencia más grande ==========
#define FILL_BUFFER_PIXELS 1024   // antes 128, aumenta velocidad

// ========== Tabla de rotaciones fija para 172x320 ==========
jd9853_rotation_t ORIENTATIONS_172x320[4] = {
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

static void jd9853_JD9853_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    (void)kind;
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "<JD9853 width=%u, height=%u, spi=%p>", self->width, self->height, self->spi_obj);
}

static void write_cmd(jd9853_JD9853_obj_t *self, uint8_t cmd, const uint8_t *data, int len) {
    CS_LOW()
    if (cmd) {
        DC_LOW();
        write_spi(self->spi_obj, &cmd, 1);
    }
    if (len > 0) {
        DC_HIGH();
        write_spi(self->spi_obj, data, len);
    }
    CS_HIGH()
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

// ========== fill_color_buffer optimizado con buffer grande ==========
static void fill_color_buffer(mp_obj_base_t *spi_obj, uint16_t color, int length) {
    int chunks = length / FILL_BUFFER_PIXELS;
    int rest = length % FILL_BUFFER_PIXELS;
    uint16_t color_swapped = _swap_bytes(color);
    uint16_t buffer[FILL_BUFFER_PIXELS];

    for (int i = 0; i < FILL_BUFFER_PIXELS; i++) buffer[i] = color_swapped;

    if (chunks) {
        for (int j = 0; j < chunks; j++) {
            write_spi(spi_obj, (uint8_t *)buffer, FILL_BUFFER_PIXELS * 2);
        }
    }
    if (rest) {
        write_spi(spi_obj, (uint8_t *)buffer, rest * 2);
    }
}

int mod(int x, int m) {
    int r = x % m;
    return (r < 0) ? r + m : r;
}

void draw_pixel(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, uint16_t color) {
    if ((self->options & OPTIONS_WRAP)) {
        if ((self->options & OPTIONS_WRAP_H) && ((x >= self->width) || (x < 0)))
            x = mod(x, self->width);
        if ((self->options & OPTIONS_WRAP_V) && ((y >= self->height) || (y < 0)))
            y = mod(y, self->height);
    }

    if ((x < self->width) && (y < self->height) && (x >= 0) && (y >= 0)) {
        uint8_t hi = color >> 8, lo = color & 0xff;
        set_window(self, x, y, x, y);
        DC_HIGH();
        CS_LOW();
        write_spi(self->spi_obj, &hi, 1);
        write_spi(self->spi_obj, &lo, 1);
        CS_HIGH();
    }
}

void fast_hline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t w, uint16_t color) {
    if ((self->options & OPTIONS_WRAP) == 0) {
        if (y >= 0 && self->width > x && self->height > y) {
            if (0 > x) { w += x; x = 0; }
            if (self->width < x + w) w = self->width - x;
            if (w > 0) {
                int16_t x2 = x + w - 1;
                set_window(self, x, y, x2, y);
                DC_HIGH();
                CS_LOW();
                fill_color_buffer(self->spi_obj, color, w);
                CS_HIGH();
            }
        }
    } else {
        for (int d = 0; d < w; d++) draw_pixel(self, x + d, y, color);
    }
}

static void fast_vline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t h, uint16_t color) {
    if ((self->options & OPTIONS_WRAP) == 0) {
        if (x >= 0 && self->width > x && self->height > y) {
            if (0 > y) { h += y; y = 0; }
            if (self->height < y + h) h = self->height - y;
            if (h > 0) {
                int16_t y2 = y + h - 1;
                set_window(self, x, y, x, y2);
                DC_HIGH();
                CS_LOW();
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
    RESET_HIGH();
    mp_hal_delay_ms(50);
    RESET_LOW();
    mp_hal_delay_ms(50);
    RESET_HIGH();
    mp_hal_delay_ms(150);
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
        DC_HIGH();
        CS_LOW();
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
    DC_HIGH();
    CS_LOW();
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

void line(jd9853_JD9853_obj_t *self, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color) {
    bool steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) { _swap_int16_t(x0, y0); _swap_int16_t(x1, y1); }
    if (x0 > x1) { _swap_int16_t(x0, x1); _swap_int16_t(y0, y1); }

    int16_t dx = x1 - x0, dy = ABS(y1 - y0);
    int16_t err = dx >> 1, ystep = -1, xs = x0, dlen = 0;
    if (y0 < y1) ystep = 1;

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
    mp_int_t x0 = mp_obj_get_int(args[1]);
    mp_int_t y0 = mp_obj_get_int(args[2]);
    mp_int_t x1 = mp_obj_get_int(args[3]);
    mp_int_t y1 = mp_obj_get_int(args[4]);
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
    DC_HIGH();
    CS_LOW();

    const int buf_size = 256;
    int limit = MIN(buf_info.len, w * h * 2);
    int chunks = limit / buf_size;
    int rest = limit % buf_size;
    int i = 0;
    for (; i < chunks; i++)
        write_spi(self->spi_obj, (const uint8_t *)buf_info.buf + i * buf_size, buf_size);
    if (rest)
        write_spi(self->spi_obj, (const uint8_t *)buf_info.buf + i * buf_size, rest);
    CS_HIGH();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_blit_buffer_obj, 6, 6, jd9853_JD9853_blit_buffer);

// ========== FUNCIONES DE TEXTO Y DIBUJO ==========
static mp_obj_t jd9853_JD9853_draw(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    char single_char_s[] = {0, 0};
    const char *s;

    mp_obj_module_t *hershey = MP_OBJ_TO_PTR(args[1]);

    if (mp_obj_is_int(args[2])) {
        mp_int_t c = mp_obj_get_int(args[2]);
        single_char_s[0] = c & 0xff;
        s = single_char_s;
    } else {
        s = mp_obj_str_get_str(args[2]);
    }

    mp_int_t x = mp_obj_get_int(args[3]);
    mp_int_t y = mp_obj_get_int(args[4]);
    mp_int_t color = (n_args > 5) ? mp_obj_get_int(args[5]) : WHITE;
    mp_float_t scale = 1.0;
    if (n_args > 6) {
        if (mp_obj_is_float(args[6]))
            scale = mp_obj_float_get(args[6]);
        else if (mp_obj_is_int(args[6]))
            scale = (mp_float_t)mp_obj_get_int(args[6]);
    }

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(hershey->globals);
    mp_obj_t *index_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_INDEX));
    mp_buffer_info_t index_bufinfo;
    mp_get_buffer_raise(index_data_buff, &index_bufinfo, MP_BUFFER_READ);
    uint8_t *index = index_bufinfo.buf;

    mp_obj_t *font_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FONT));
    mp_buffer_info_t font_bufinfo;
    mp_get_buffer_raise(font_data_buff, &font_bufinfo, MP_BUFFER_READ);
    int8_t *font = font_bufinfo.buf;

    int16_t from_x = x;
    int16_t from_y = y;
    int16_t to_x = x;
    int16_t to_y = y;
    int16_t pos_x = x;
    int16_t pos_y = y;
    bool penup = true;
    char c;
    int16_t ii;

    while ((c = *s++)) {
        if (c >= 32 && c <= 127) {
            ii = (c - 32) * 2;
            int16_t offset = index[ii] | (index[ii + 1] << 8);
            int16_t length = font[offset++];
            int16_t left = (int)(scale * (font[offset++] - 0x52) + 0.5);
            int16_t right = (int)(scale * (font[offset++] - 0x52) + 0.5);
            int16_t width = right - left;

            if (length) {
                int16_t i;
                for (i = 0; i < length; i++) {
                    if (font[offset] == ' ') {
                        offset += 2;
                        penup = true;
                        continue;
                    }
                    int16_t vector_x = (int)(scale * (font[offset++] - 0x52) + 0.5);
                    int16_t vector_y = (int)(scale * (font[offset++] - 0x52) + 0.5);

                    if (!i || penup) {
                        from_x = pos_x + vector_x - left;
                        from_y = pos_y + vector_y;
                    } else {
                        to_x = pos_x + vector_x - left;
                        to_y = pos_y + vector_y;
                        line(self, from_x, from_y, to_x, to_y, color);
                        from_x = to_x;
                        from_y = to_y;
                    }
                    penup = false;
                }
            }
            pos_x += width;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_obj, 5, 7, jd9853_JD9853_draw);

static mp_obj_t jd9853_JD9853_draw_len(size_t n_args, const mp_obj_t *args) {
    char single_char_s[] = {0, 0};
    const char *s;
    mp_obj_module_t *hershey = MP_OBJ_TO_PTR(args[1]);

    if (mp_obj_is_int(args[2])) {
        mp_int_t c = mp_obj_get_int(args[2]);
        single_char_s[0] = c & 0xff;
        s = single_char_s;
    } else {
        s = mp_obj_str_get_str(args[2]);
    }

    mp_float_t scale = 1.0;
    if (n_args > 3) {
        if (mp_obj_is_float(args[3]))
            scale = mp_obj_float_get(args[3]);
        else if (mp_obj_is_int(args[3]))
            scale = (mp_float_t)mp_obj_get_int(args[3]);
    }

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(hershey->globals);
    mp_obj_t *index_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_INDEX));
    mp_buffer_info_t index_bufinfo;
    mp_get_buffer_raise(index_data_buff, &index_bufinfo, MP_BUFFER_READ);
    uint8_t *index = index_bufinfo.buf;

    mp_obj_t *font_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FONT));
    mp_buffer_info_t font_bufinfo;
    mp_get_buffer_raise(font_data_buff, &font_bufinfo, MP_BUFFER_READ);
    int8_t *font = font_bufinfo.buf;

    int16_t print_width = 0;
    char c;
    int16_t ii;

    while ((c = *s++)) {
        if (c >= 32 && c <= 127) {
            ii = (c - 32) * 2;
            int16_t offset = (index[ii] | (index[ii + 1] << 8)) + 1;
            int16_t left =  font[offset++] - 0x52;
            int16_t right = font[offset++] - 0x52;
            print_width += (right - left);
        }
    }
    return mp_obj_new_int((int)(print_width * scale + 0.5));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_len_obj, 3, 4, jd9853_JD9853_draw_len);

// ========== BITMAP y WRITE ==========
static uint32_t bs_bit = 0;
uint8_t *bitmap_data = NULL;

static uint8_t get_color(uint8_t bpp) {
    uint8_t color = 0;
    for (int i = 0; i < bpp; i++) {
        color <<= 1;
        color |= (bitmap_data[bs_bit / 8] & (1 << (7 - (bs_bit % 8)))) ? 1 : 0;
        bs_bit++;
    }
    return color;
}

static mp_obj_t dict_lookup(mp_obj_t self_in, mp_obj_t index) {
    mp_obj_dict_t *self = MP_OBJ_TO_PTR(self_in);
    mp_map_elem_t *elem = mp_map_lookup(&self->map, index, MP_MAP_LOOKUP);
    return elem ? elem->value : NULL;
}

static mp_obj_t jd9853_JD9853_write_len(size_t n_args, const mp_obj_t *args) {
    mp_obj_module_t *font = MP_OBJ_TO_PTR(args[1]);
    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(font->globals);
    mp_obj_t widths_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTHS));
    mp_buffer_info_t widths_bufinfo;
    mp_get_buffer_raise(widths_data_buff, &widths_bufinfo, MP_BUFFER_READ);
    const uint8_t *widths_data = widths_bufinfo.buf;

    uint16_t print_width = 0;
    mp_obj_t map_obj = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_MAP));
    GET_STR_DATA_LEN(map_obj, map_data, map_len);
    GET_STR_DATA_LEN(args[2], str_data, str_len);
    const byte *s = str_data, *top = str_data + str_len;

    while (s < top) {
        unichar ch = utf8_get_char(s);
        s = utf8_next_char(s);
        const byte *map_s = map_data, *map_top = map_data + map_len;
        uint16_t char_index = 0;
        while (map_s < map_top) {
            unichar map_ch = utf8_get_char(map_s);
            map_s = utf8_next_char(map_s);
            if (ch == map_ch) {
                print_width += widths_data[char_index];
                break;
            }
            char_index++;
        }
    }
    return mp_obj_new_int(print_width);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_write_len_obj, 3, 3, jd9853_JD9853_write_len);

static mp_obj_t jd9853_JD9853_write(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *font = MP_OBJ_TO_PTR(args[1]);

    mp_int_t x = mp_obj_get_int(args[3]);
    mp_int_t y = mp_obj_get_int(args[4]);
    mp_int_t fg_color = (n_args > 5) ? _swap_bytes(mp_obj_get_int(args[5])) : _swap_bytes(WHITE);
    mp_int_t bg_color = (n_args > 6) ? _swap_bytes(mp_obj_get_int(args[6])) : _swap_bytes(BLACK);

    mp_obj_t *tuple_data = NULL;
    size_t tuple_len = 0;
    mp_buffer_info_t background_bufinfo;
    uint16_t background_width = 0, background_height = 0;
    uint16_t *background_data = NULL;

    if (n_args > 7) {
        mp_obj_tuple_get(args[7], &tuple_len, &tuple_data);
        if (tuple_len > 2) {
            mp_get_buffer_raise(tuple_data[0], &background_bufinfo, MP_BUFFER_READ);
            background_data = background_bufinfo.buf;
            background_width = mp_obj_get_int(tuple_data[1]);
            background_height = mp_obj_get_int(tuple_data[2]);
        }
    }
    bool fill = (n_args > 8) ? mp_obj_is_true(args[8]) : false;

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(font->globals);
    const uint8_t bpp = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BPP)));
    const uint8_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint8_t offset_width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_OFFSET_WIDTH)));
    const uint8_t max_width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_MAX_WIDTH)));

    mp_obj_t widths_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTHS));
    mp_buffer_info_t widths_bufinfo;
    mp_get_buffer_raise(widths_data_buff, &widths_bufinfo, MP_BUFFER_READ);
    const uint8_t *widths_data = widths_bufinfo.buf;

    mp_obj_t offsets_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_OFFSETS));
    mp_buffer_info_t offsets_bufinfo;
    mp_get_buffer_raise(offsets_data_buff, &offsets_bufinfo, MP_BUFFER_READ);
    const uint8_t *offsets_data = offsets_bufinfo.buf;

    mp_obj_t bitmaps_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BITMAPS));
    mp_buffer_info_t bitmaps_bufinfo;
    mp_get_buffer_raise(bitmaps_data_buff, &bitmaps_bufinfo, MP_BUFFER_READ);
    bitmap_data = bitmaps_bufinfo.buf;

    // Usar buffer interno estático si no se proporcionó uno externo
    if (self->buffer_size == 0) {
        self->i2c_buffer = self->static_buffer;
    }

    if (fill && background_data && self->i2c_buffer) {
        memcpy(self->i2c_buffer, background_data, background_width * background_height * 2);
    }

    uint16_t print_width = 0;
    mp_obj_t map_obj = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_MAP));
    GET_STR_DATA_LEN(map_obj, map_data, map_len);
    GET_STR_DATA_LEN(args[2], str_data, str_len);
    const byte *s = str_data, *top = str_data + str_len;

    while (s < top) {
        unichar ch = utf8_get_char(s);
        s = utf8_next_char(s);
        const byte *map_s = map_data, *map_top = map_data + map_len;
        uint16_t char_index = 0;
        while (map_s < map_top) {
            unichar map_ch = utf8_get_char(map_s);
            map_s = utf8_next_char(map_s);
            if (ch == map_ch) {
                uint8_t width = widths_data[char_index];
                bs_bit = 0;
                switch (offset_width) {
                    case 1: bs_bit = offsets_data[char_index * offset_width]; break;
                    case 2: bs_bit = (offsets_data[char_index * offset_width] << 8) + offsets_data[char_index * offset_width + 1]; break;
                    case 3: bs_bit = (offsets_data[char_index * offset_width] << 16) + (offsets_data[char_index * offset_width + 1] << 8) + offsets_data[char_index * offset_width + 2]; break;
                }
                uint16_t buffer_width = (fill) ? max_width : width;
                uint16_t color = 0;
                for (uint16_t yy = 0; yy < height; yy++) {
                    for (uint16_t xx = 0; xx < width; xx++) {
                        if (background_data && (xx <= background_width && yy <= background_height)) {
                            if (get_color(bpp) == bg_color) {
                                color = background_data[(yy * background_width + xx)];
                            } else {
                                color = fg_color;
                            }
                        } else {
                            color = get_color(bpp) ? fg_color : bg_color;
                        }
                        ((uint16_t*)self->i2c_buffer)[yy * buffer_width + xx] = color;
                    }
                }
                uint32_t data_size = buffer_width * height * 2;
                uint16_t x2 = x + buffer_width - 1;
                uint16_t y2 = y + height - 1;
                if (x2 < self->width) {
                    set_window(self, x, y, x2, y2);
                    DC_HIGH();
                    CS_LOW();
                    write_spi(self->spi_obj, (uint8_t *)self->i2c_buffer, data_size);
                    CS_HIGH();
                    print_width += width;
                }
                x += width;
                break;
            }
            char_index++;
        }
    }
    return mp_obj_new_int(print_width);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_write_obj, 5, 9, jd9853_JD9853_write);

// ========== BITMAP ==========
static mp_obj_t jd9853_JD9853_bitmap(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_module_t *bitmap = MP_OBJ_TO_PTR(args[1]);
    mp_int_t x = mp_obj_get_int(args[2]);
    mp_int_t y = mp_obj_get_int(args[3]);
    mp_int_t idx = (n_args > 4) ? mp_obj_get_int(args[4]) : 0;

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(bitmap->globals);
    const uint16_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint16_t width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTH)));
    uint16_t bitmaps = 0;
    const uint8_t bpp = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BPP)));
    mp_obj_t *palette_arg = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_PALETTE));
    mp_obj_t *palette = NULL;
    size_t palette_len = 0;

    mp_map_elem_t *elem = dict_lookup(bitmap->globals, MP_OBJ_NEW_QSTR(MP_QSTR_BITMAPS));
    if (elem) bitmaps = mp_obj_get_int(elem);

    mp_obj_get_array(palette_arg, &palette_len, &palette);
    mp_obj_t *bitmap_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_BITMAP));
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(bitmap_data_buff, &bufinfo, MP_BUFFER_READ);
    bitmap_data = bufinfo.buf;

    size_t buf_size = width * height * 2;
    if (self->buffer_size == 0) {
        self->i2c_buffer = self->static_buffer;
    }

    size_t ofs = 0;
    bs_bit = 0;
    if (bitmaps) {
        if (idx < bitmaps) {
            bs_bit = height * width * bpp * idx;
        } else {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("index out of range"));
        }
    }

    for (int yy = 0; yy < height; yy++) {
        for (int xx = 0; xx < width; xx++) {
            ((uint16_t*)self->i2c_buffer)[ofs++] = mp_obj_get_int(palette[get_color(bpp)]);
        }
    }

    uint16_t x1 = x + width - 1;
    if (x1 < self->width) {
        set_window(self, x, y, x1, y + height - 1);
        DC_HIGH();
        CS_LOW();
        write_spi(self->spi_obj, (uint8_t *)self->i2c_buffer, buf_size);
        CS_HIGH();
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_bitmap_obj, 4, 5, jd9853_JD9853_bitmap);

// ========== TEXTO (FONTS MONO) ==========
static mp_obj_t jd9853_JD9853_text(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint8_t single_char_s;
    const uint8_t *source = NULL;
    size_t source_len = 0;

    mp_obj_module_t *font = MP_OBJ_TO_PTR(args[1]);

    if (mp_obj_is_int(args[2])) {
        mp_int_t c = mp_obj_get_int(args[2]);
        single_char_s = (c & 0xff);
        source = &single_char_s;
        source_len = 1;
    } else if (mp_obj_is_str(args[2])) {
        source = (uint8_t *) mp_obj_str_get_str(args[2]);
        source_len = strlen((char *)source);
    } else if (mp_obj_is_type(args[2], &mp_type_bytes)) {
        mp_obj_t text_data_buff = args[2];
        mp_buffer_info_t text_bufinfo;
        mp_get_buffer_raise(text_data_buff, &text_bufinfo, MP_BUFFER_READ);
        source = text_bufinfo.buf;
        source_len = text_bufinfo.len;
    } else {
        mp_raise_TypeError(MP_ERROR_TEXT("text requires either int, str or bytes."));
        return mp_const_none;
    }

    mp_int_t x0 = mp_obj_get_int(args[3]);
    mp_int_t y0 = mp_obj_get_int(args[4]);

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(font->globals);
    const uint8_t width = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_WIDTH)));
    const uint8_t height = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_HEIGHT)));
    const uint8_t first = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FIRST)));
    const uint8_t last = mp_obj_get_int(mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_LAST)));

    mp_obj_t font_data_buff = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(MP_QSTR_FONT));
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(font_data_buff, &bufinfo, MP_BUFFER_READ);
    const uint8_t *font_data = bufinfo.buf;

    mp_int_t fg_color = (n_args > 5) ? _swap_bytes(mp_obj_get_int(args[5])) : _swap_bytes(WHITE);
    mp_int_t bg_color = (n_args > 6) ? _swap_bytes(mp_obj_get_int(args[6])) : _swap_bytes(BLACK);

    uint8_t wide = width / 8;
    size_t buf_size = width * height * 2;

    if (self->buffer_size == 0) {
        self->i2c_buffer = self->static_buffer;
    }

    uint8_t chr;
    while (source_len--) {
        chr = *source++;
        if (chr >= first && chr <= last) {
            uint16_t buf_idx = 0;
            uint16_t chr_idx = (chr - first) * (height * wide);
            for (uint8_t line = 0; line < height; line++) {
                for (uint8_t line_byte = 0; line_byte < wide; line_byte++) {
                    uint8_t chr_data = font_data[chr_idx];
                    for (uint8_t bit = 8; bit; bit--) {
                        if (chr_data >> (bit - 1) & 1) {
                            ((uint16_t*)self->i2c_buffer)[buf_idx] = fg_color;
                        } else {
                            ((uint16_t*)self->i2c_buffer)[buf_idx] = bg_color;
                        }
                        buf_idx++;
                    }
                    chr_idx++;
                }
            }
            uint16_t x1 = x0 + width - 1;
            if (x1 < self->width) {
                set_window(self, x0, y0, x1, y0 + height - 1);
                DC_HIGH();
                CS_LOW();
                write_spi(self->spi_obj, (uint8_t *)self->i2c_buffer, buf_size);
                CS_HIGH();
            }
            x0 += width;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_text_obj, 5, 7, jd9853_JD9853_text);

// ========== ROTACIÓN CORREGIDA Y FORZADA ==========
static void set_rotation(jd9853_JD9853_obj_t *self) {
    uint8_t madctl_value = self->color_order;

    if (self->rotation >= self->rotations_len) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("Invalid rotation value %d > %d"), self->rotation, self->rotations_len);
    }

    jd9853_rotation_t *rotations = self->rotations;
    // FORZAR siempre la tabla 172x320
    if (rotations == NULL) {
        rotations = ORIENTATIONS_172x320;
    }

    if (rotations) {
        jd9853_rotation_t *rotation = &rotations[self->rotation];
        madctl_value |= rotation->madctl;
        self->width = rotation->width;
        self->height = rotation->height;
        self->colstart = rotation->colstart;
        self->rowstart = rotation->rowstart;
    }

    self->madctl = madctl_value & 0xff;
    self->min_x = self->width;
    self->min_y = self->height;
    self->max_x = 0;
    self->max_y = 0;

    const uint8_t madctl[] = {madctl_value};
    write_cmd(self, JD9853_MADCTL, madctl, 1);
}

static mp_obj_t jd9853_JD9853_rotation(mp_obj_t self_in, mp_obj_t value) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t rotation = mp_obj_get_int(value) % 4;
    self->rotation = rotation;
    set_rotation(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_rotation_obj, jd9853_JD9853_rotation);

static mp_obj_t jd9853_JD9853_width(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->width);
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_width_obj, jd9853_JD9853_width);

static mp_obj_t jd9853_JD9853_height(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->height);
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_height_obj, jd9853_JD9853_height);

// ========== SCROLL VERTICAL ==========
static mp_obj_t jd9853_JD9853_vscrdef(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t tfa = mp_obj_get_int(args[1]);
    mp_int_t vsa = mp_obj_get_int(args[2]);
    mp_int_t bfa = mp_obj_get_int(args[3]);

    self->tfa = tfa;
    self->vsa = vsa;
    self->bfa = bfa;

    uint8_t buf[6] = { (tfa >> 8) & 0xFF, tfa & 0xFF,
                       (vsa >> 8) & 0xFF, vsa & 0xFF,
                       (bfa >> 8) & 0xFF, bfa & 0xFF };
    write_cmd(self, JD9853_VSCRDEF, buf, 6);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_vscrdef_obj, 4, 4, jd9853_JD9853_vscrdef);

static mp_obj_t jd9853_JD9853_vscsad(mp_obj_t self_in, mp_obj_t vssa_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t vssa = mp_obj_get_int(vssa_in);
    self->vscsad = vssa;
    uint8_t buf[2] = { (vssa >> 8) & 0xFF, vssa & 0xFF };
    write_cmd(self, JD9853_VSCRSADD, buf, 2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(jd9853_JD9853_vscsad_obj, jd9853_JD9853_vscsad);

static mp_obj_t jd9853_JD9853_scroll(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t dy = mp_obj_get_int(args[1]);

    if (self->tfa + self->vsa + self->bfa != self->height) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Scroll area not defined (call vscrdef first)"));
        return mp_const_none;
    }

    uint16_t new_vsp = (self->vscsad + dy) % self->vsa;
    uint8_t buf[2] = { (new_vsp >> 8) & 0xFF, new_vsp & 0xFF };
    write_cmd(self, JD9853_VSCRSADD, buf, 2);
    self->vscsad = new_vsp;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_scroll_obj, 2, 2, jd9853_JD9853_scroll);

// ========== INICIALIZACIÓN CON MODO 18 BITS ==========
static void custom_init(jd9853_JD9853_obj_t *self) {
    size_t init_len;
    mp_obj_t *init_list;
    mp_obj_get_array(self->custom_init, &init_len, &init_list);
    for (int idx = 0; idx < init_len; idx++) {
        size_t init_cmd_len;
        mp_obj_t *init_cmd;
        mp_obj_get_array(init_list[idx], &init_cmd_len, &init_cmd);
        mp_buffer_info_t init_cmd_data_info;
        if (mp_get_buffer(init_cmd[0], &init_cmd_data_info, MP_BUFFER_READ)) {
            uint8_t *init_cmd_data = (uint8_t *)init_cmd_data_info.buf;
            if (init_cmd_data_info.len > 1)
                write_cmd(self, init_cmd_data[0], &init_cmd_data[1], init_cmd_data_info.len - 1);
            else
                write_cmd(self, init_cmd_data[0], NULL, 0);
            mp_hal_delay_ms(10);
            if (init_cmd_len > 1) {
                mp_int_t delay = mp_obj_get_int(init_cmd[1]);
                if (delay > 0) mp_hal_delay_ms(delay);
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

        const uint8_t unlock[] = {0x98, 0x53};
        write_cmd(self, 0xDF, unlock, 2);
        mp_hal_delay_ms(10);
        write_cmd(self, 0xDF, unlock, 2);
        mp_hal_delay_ms(10);

        // Modo 18 bits (262K colores)
        const uint8_t color_mode[] = {0x66};
        write_cmd(self, JD9853_COLMOD, color_mode, 1);
        mp_hal_delay_ms(10);

        const uint8_t madctl[] = {JD9853_MADCTL_RGB};
        write_cmd(self, JD9853_MADCTL, madctl, 1);
        mp_hal_delay_ms(10);

        if (self->inversion)
            write_cmd(self, JD9853_INVON, NULL, 0);
        else
            write_cmd(self, JD9853_INVOFF, NULL, 0);

        write_cmd(self, JD9853_NORON, NULL, 0);
        mp_hal_delay_ms(10);
        write_cmd(self, JD9853_DISPON, NULL, 0);
        mp_hal_delay_ms(150);
    } else {
        custom_init(self);
    }

    set_rotation(self);
    mp_hal_delay_ms(10);

    const mp_obj_t args[] = {
        self_in,
        mp_obj_new_int(0),
        mp_obj_new_int(0),
        mp_obj_new_int(self->width),
        mp_obj_new_int(self->height),
        mp_obj_new_int(BLACK)
    };
    jd9853_JD9853_fill_rect(6, args);

    if (self->backlight != GPIO_NUM_NC)
        mp_hal_pin_write(self->backlight, 1);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_init_obj, jd9853_JD9853_init);

static mp_obj_t jd9853_JD9853_on(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->backlight != GPIO_NUM_NC) {
        mp_hal_pin_write(self->backlight, 1);
        mp_hal_delay_ms(10);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_on_obj, jd9853_JD9853_on);

static mp_obj_t jd9853_JD9853_off(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->backlight != GPIO_NUM_NC) {
        mp_hal_pin_write(self->backlight, 0);
        mp_hal_delay_ms(10);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_off_obj, jd9853_JD9853_off);

// ========== OTRAS PRIMITIVAS DE DIBUJO ==========
static mp_obj_t jd9853_JD9853_hline(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]);
    mp_int_t color = mp_obj_get_int(args[4]);
    fast_hline(self, x, y, w, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_hline_obj, 5, 5, jd9853_JD9853_hline);

static mp_obj_t jd9853_JD9853_vline(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]);
    mp_int_t color = mp_obj_get_int(args[4]);
    fast_vline(self, x, y, w, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_vline_obj, 5, 5, jd9853_JD9853_vline);

static mp_obj_t jd9853_JD9853_circle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t xm = mp_obj_get_int(args[1]);
    mp_int_t ym = mp_obj_get_int(args[2]);
    mp_int_t r = mp_obj_get_int(args[3]);
    mp_int_t color = mp_obj_get_int(args[4]);

    int f = 1 - r, ddF_x = 1, ddF_y = -2 * r;
    int x = 0, y = r;
    draw_pixel(self, xm, ym + r, color);
    draw_pixel(self, xm, ym - r, color);
    draw_pixel(self, xm + r, ym, color);
    draw_pixel(self, xm - r, ym, color);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        draw_pixel(self, xm + x, ym + y, color);
        draw_pixel(self, xm - x, ym + y, color);
        draw_pixel(self, xm + x, ym - y, color);
        draw_pixel(self, xm - x, ym - y, color);
        draw_pixel(self, xm + y, ym + x, color);
        draw_pixel(self, xm - y, ym + x, color);
        draw_pixel(self, xm + y, ym - x, color);
        draw_pixel(self, xm - y, ym - x, color);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_circle_obj, 5, 5, jd9853_JD9853_circle);

static mp_obj_t jd9853_JD9853_fill_circle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t xm = mp_obj_get_int(args[1]);
    mp_int_t ym = mp_obj_get_int(args[2]);
    mp_int_t r = mp_obj_get_int(args[3]);
    mp_int_t color = mp_obj_get_int(args[4]);

    int f = 1 - r, ddF_x = 1, ddF_y = -2 * r;
    int x = 0, y = r;
    fast_vline(self, xm, ym - y, 2 * y + 1, color);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        fast_vline(self, xm + x, ym - y, 2 * y + 1, color);
        fast_vline(self, xm + y, ym - x, 2 * x + 1, color);
        fast_vline(self, xm - x, ym - y, 2 * y + 1, color);
        fast_vline(self, xm - y, ym - x, 2 * x + 1, color);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_circle_obj, 5, 5, jd9853_JD9853_fill_circle);

static mp_obj_t jd9853_JD9853_rect(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]);
    mp_int_t h = mp_obj_get_int(args[4]);
    mp_int_t color = mp_obj_get_int(args[5]);
    fast_hline(self, x, y, w, color);
    fast_vline(self, x, y, h, color);
    fast_hline(self, x, y + h - 1, w, color);
    fast_vline(self, x + w - 1, y, h, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_rect_obj, 6, 6, jd9853_JD9853_rect);

static mp_obj_t jd9853_JD9853_round_rect(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]);
    mp_int_t y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]);
    mp_int_t h = mp_obj_get_int(args[4]);
    mp_int_t r = mp_obj_get_int(args[5]);
    mp_int_t color = mp_obj_get_int(args[6]);

    if (r > w/2) r = w/2;
    if (r > h/2) r = h/2;
    fast_hline(self, x + r, y, w - 2 * r, color);
    fast_hline(self, x + r, y + h - 1, w - 2 * r, color);
    fast_vline(self, x, y + r, h - 2 * r, color);
    fast_vline(self, x + w - 1, y + r, h - 2 * r, color);
    for (int i = 0; i <= r; i++) {
        int d = (int)(sqrt(r * r - i * i) + 0.5);
        draw_pixel(self, x + r - i, y + r - d, color);
        draw_pixel(self, x + r - i, y + h - r + d - 1, color);
        draw_pixel(self, x + w - r + i - 1, y + r - d, color);
        draw_pixel(self, x + w - r + i - 1, y + h - r + d - 1, color);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_round_rect_obj, 7, 7, jd9853_JD9853_round_rect);

static mp_obj_t jd9853_JD9853_madctl(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 2) {
        mp_int_t madctl_value = mp_obj_get_int(args[1]) & 0xff;
        const uint8_t madctl[] = {madctl_value};
        write_cmd(self, JD9853_MADCTL, madctl, 1);
        self->madctl = madctl_value & 0xff;
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

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}
static mp_obj_t jd9853_color565(mp_obj_t r, mp_obj_t g, mp_obj_t b) {
    return MP_OBJ_NEW_SMALL_INT(color565(mp_obj_get_int(r), mp_obj_get_int(g), mp_obj_get_int(b)));
}
static MP_DEFINE_CONST_FUN_OBJ_3(jd9853_color565_obj, jd9853_color565);

static void map_bitarray_to_rgb565(uint8_t const *bitarray, uint8_t *buffer, int length, int width,
    uint16_t color, uint16_t bg_color) {
    int row_pos = 0;
    for (int i = 0; i < length; i++) {
        uint8_t byte = bitarray[i];
        for (int bi = 7; bi >= 0; bi--) {
            uint8_t b = byte & (1 << bi);
            uint16_t cur_color = b ? color : bg_color;
            *buffer = (cur_color & 0xff00) >> 8; buffer++;
            *buffer = cur_color & 0xff; buffer++;
            row_pos++;
            if (row_pos >= width) { row_pos = 0; break; }
        }
    }
}
static mp_obj_t jd9853_map_bitarray_to_rgb565(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bitarray_info, buffer_info;
    mp_get_buffer_raise(args[1], &bitarray_info, MP_BUFFER_READ);
    mp_get_buffer_raise(args[2], &buffer_info, MP_BUFFER_WRITE);
    mp_int_t width = mp_obj_get_int(args[3]);
    mp_int_t color = mp_obj_get_int(args[4]);
    mp_int_t bg_color = mp_obj_get_int(args[5]);
    map_bitarray_to_rgb565(bitarray_info.buf, buffer_info.buf, bitarray_info.len, width, color, bg_color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_map_bitarray_to_rgb565_obj, 3, 6, jd9853_map_bitarray_to_rgb565);

// ========== PROCESAMIENTO DE JPG (código original) ==========
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
} IODEV;

static unsigned int buffer_in_func(JDEC *jd, uint8_t *buff, unsigned int nbyte) {
    IODEV *dev = (IODEV *)jd->device;
    if (dev->dataIdx + nbyte > dev->dataLen) nbyte = dev->dataLen - dev->dataIdx;
    if (buff) memcpy(buff, dev->data + dev->dataIdx, nbyte);
    dev->dataIdx += nbyte;
    return nbyte;
}
static unsigned int file_in_func(JDEC *jd, uint8_t *buff, unsigned int nbyte) {
    IODEV *dev = (IODEV *)jd->device;
    if (buff) return (unsigned int)mp_readinto(dev->fp, buff, nbyte);
    mp_seek(dev->fp, nbyte, SEEK_CUR);
    return 0;
}
static int out_fast(JDEC *jd, void *bitmap, JRECT *rect) {
    IODEV *dev = (IODEV *)jd->device;
    uint8_t *src = (uint8_t*)bitmap;
    uint8_t *dst = dev->fbuf + 2 * (rect->top * dev->wfbuf + rect->left);
    int bws = 2 * (rect->right - rect->left + 1);
    int bwd = 2 * dev->wfbuf;
    for (unsigned int y = rect->top; y <= rect->bottom; y++) {
        memcpy(dst, src, bws);
        src += bws;
        dst += bwd;
    }
    return 1;
}
static int out_slow(JDEC *jd, void *bitmap, JRECT *rect) {
    IODEV *dev = (IODEV *)jd->device;
    jd9853_JD9853_obj_t *self = dev->self;
    uint8_t *src = (uint8_t*)bitmap;
    uint8_t *dst = dev->fbuf;
    int wx2 = (rect->right - rect->left + 1) * 2;
    int h = rect->bottom - rect->top + 1;
    for (unsigned int y = rect->top; y <= rect->bottom; y++) {
        memcpy(dst, src, wx2);
        src += wx2;
        dst += wx2;
    }
    set_window(self, rect->left + jd->x_offs, rect->top + jd->y_offs,
               rect->right + jd->x_offs, rect->bottom + jd->y_offs);
    DC_HIGH(); CS_LOW();
    write_spi(self->spi_obj, (uint8_t *)dev->fbuf, wx2 * h);
    CS_HIGH();
    return 1;
}
static mp_obj_t jd9853_JD9853_jpg(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    static unsigned int (*input_func)(JDEC *, uint8_t *, unsigned int) = NULL;
    mp_buffer_info_t bufinfo;
    IODEV devid;
    if (mp_obj_is_type(args[1], &mp_type_bytes)) {
        mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_READ);
        devid.dataIdx = 0; devid.data = bufinfo.buf; devid.dataLen = bufinfo.len;
        input_func = buffer_in_func;
        self->fp = MP_OBJ_NULL;
    } else {
        const char *filename = mp_obj_str_get_str(args[1]);
        self->fp = mp_open(filename, "rb");
        devid.fp = self->fp; input_func = file_in_func;
        devid.data = NULL; devid.dataLen = 0;
    }
    mp_int_t x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]);
    mp_int_t mode = (n_args > 4) ? mp_obj_get_int(args[4]) : JPG_MODE_FAST;
    JRESULT res;
    JDEC jdec;
    self->work = (void *)m_malloc(3100);
    if (input_func && (devid.fp || devid.data)) {
        res = jd_prepare(&jdec, input_func, self->work, 3100, &devid);
        if (res == JDR_OK) {
            size_t bufsize;
            int (*outfunc)(JDEC*,void*,JRECT*);
            if (mode == JPG_MODE_FAST) {
                bufsize = 2 * jdec.width * jdec.height;
                outfunc = out_fast;
            } else {
                bufsize = 2 * jdec.msx * 8 * jdec.msy * 8;
                outfunc = out_slow;
                jdec.x_offs = x; jdec.y_offs = y;
            }
            if (self->buffer_size && bufsize > self->buffer_size)
                mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("buffer too small. %ld bytes required."), (long)bufsize);
            if (self->buffer_size == 0)
                self->i2c_buffer = m_malloc(bufsize);
            if (!self->i2c_buffer)
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("out of memory"));
            devid.fbuf = (uint8_t *)self->i2c_buffer;
            devid.wfbuf = jdec.width;
            devid.self = self;
            res = jd_decomp(&jdec, outfunc, 0);
            if (res == JDR_OK) {
                if (mode == JPG_MODE_FAST) {
                    set_window(self, x, y, x + jdec.width - 1, y + jdec.height - 1);
                    DC_HIGH(); CS_LOW();
                    write_spi(self->spi_obj, (uint8_t *)self->i2c_buffer, bufsize);
                    CS_HIGH();
                }
            } else {
                mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("jpg decompress failed."));
            }
            if (self->buffer_size == 0) { m_free(self->i2c_buffer); self->i2c_buffer = NULL; }
            devid.fbuf = NULL;
        } else {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("jpg prepare failed."));
        }
        if (self->fp) { mp_close(self->fp); self->fp = MP_OBJ_NULL; }
    }
    m_free(self->work);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_jpg_obj, 4, 5, jd9853_JD9853_jpg);

static int out_crop(JDEC *jd, void *bitmap, JRECT *rect) {
    IODEV *dev = (IODEV *)jd->device;
    if (dev->left <= rect->right && dev->right >= rect->left && dev->top <= rect->bottom && dev->bottom >= rect->top) {
        uint16_t left = MAX(dev->left, rect->left);
        uint16_t top = MAX(dev->top, rect->top);
        uint16_t right = MIN(dev->right, rect->right);
        uint16_t bottom = MIN(dev->bottom, rect->bottom);
        uint16_t dev_width = dev->right - dev->left + 1;
        uint16_t rect_width = rect->right - rect->left + 1;
        uint16_t width = (right - left + 1) * 2;
        for (uint16_t row = top; row <= bottom; row++) {
            memcpy((uint16_t *)dev->fbuf + ((row - dev->top) * dev_width) + left - dev->left,
                   (uint16_t *)bitmap + ((row - rect->top) * rect_width) + left - rect->left, width);
        }
    }
    return 1;
}
static mp_obj_t jd9853_JD9853_jpg_decode(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    static unsigned int (*input_func)(JDEC *, uint8_t *, unsigned int) = NULL;
    mp_buffer_info_t bufinfo;
    IODEV devid;
    if (mp_obj_is_type(args[1], &mp_type_bytes)) {
        mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_READ);
        devid.dataIdx = 0; devid.data = bufinfo.buf; devid.dataLen = bufinfo.len;
        input_func = buffer_in_func;
        self->fp = MP_OBJ_NULL;
    } else {
        const char *filename = mp_obj_str_get_str(args[1]);
        self->fp = mp_open(filename, "rb");
        devid.fp = self->fp; input_func = file_in_func;
        devid.data = NULL; devid.dataLen = 0;
    }
    mp_int_t x = 0, y = 0, width = 0, height = 0;
    if (n_args == 2 || n_args == 6) {
        if (n_args == 6) {
            x = mp_obj_get_int(args[2]); y = mp_obj_get_int(args[3]);
            width = mp_obj_get_int(args[4]); height = mp_obj_get_int(args[5]);
        }
        self->work = m_malloc(3100);
        JRESULT res; JDEC jdec;
        if (input_func && (devid.fp || devid.data)) {
            res = jd_prepare(&jdec, input_func, self->work, 3100, &devid);
            if (res == JDR_OK) {
                if (n_args < 6) { x = 0; y = 0; width = jdec.width; height = jdec.height; }
                devid.left = x; devid.top = y; devid.right = x + width - 1; devid.bottom = y + height - 1;
                size_t bufsize = 2 * width * height;
                self->i2c_buffer = m_malloc(bufsize);
                if (!self->i2c_buffer) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("out of memory"));
                memset(self->i2c_buffer, 0, bufsize);
                devid.fbuf = (uint8_t *)self->i2c_buffer;
                devid.wfbuf = jdec.width;
                devid.self = self;
                res = jd_decomp(&jdec, out_crop, 0);
                if (res != JDR_OK) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("jpg decompress failed."));
            } else {
                mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("jpg prepare failed."));
            }
            if (self->fp) { mp_close(self->fp); self->fp = MP_OBJ_NULL; }
        }
        m_free(self->work);
        mp_obj_t result[3] = { mp_obj_new_bytearray(bufsize, (mp_obj_t *)self->i2c_buffer),
                               mp_obj_new_int(width), mp_obj_new_int(height) };
        return mp_obj_new_tuple(3, result);
    }
    mp_raise_TypeError(MP_ERROR_TEXT("jpg_decode requires either 2 or 6 arguments"));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_jpg_decode_obj, 2, 6, jd9853_JD9853_jpg_decode);

// ========== PROCESAMIENTO DE PNG (código original) ==========
typedef struct _PNG_USER_DATA {
    jd9853_JD9853_obj_t *self;
    int ofs_x, ofs_y;
    uint16_t pixels, row, first, last;
    bool has_transparency;
    uint16_t *buffer;
} PNG_USER_DATA;

void png_flush(jd9853_JD9853_obj_t *self, PNG_USER_DATA *user_data) {
    set_window(self, user_data->first, user_data->row, user_data->last, user_data->row);
    DC_HIGH(); CS_LOW();
    write_spi(self->spi_obj, (uint8_t *)self->i2c_buffer, user_data->pixels * 2);
    CS_HIGH();
    user_data->buffer = self->i2c_buffer;
    user_data->pixels = 0;
}
void png_new_row(PNG_USER_DATA *user_data, uint16_t row, uint16_t col) {
    user_data->row = row;
    user_data->first = col;
    user_data->last = col;
}
void pngle_on_draw(pngle_t *pngle, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t rgba[4]) {
    PNG_USER_DATA *user_data = pngle_get_user_data(pngle);
    jd9853_JD9853_obj_t *self = user_data->self;
    int row = y + user_data->ofs_y;
    int col = x + user_data->ofs_x;
    if (col < 0 || row < 0 || col >= self->width || row > self->height) return;
    pngle_ihdr_t *ihdr = pngle_get_ihdr(pngle);
    size_t min_buffer_size = ihdr->width * 2;
    if (user_data->buffer == NULL) {
        if (self->buffer_size == 0) {
            user_data->buffer = m_malloc(min_buffer_size);
            if (!user_data->buffer) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("out of memory allocating buffer"));
        } else {
            if (self->buffer_size < min_buffer_size)
                mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("buffer too small. %zu bytes required."), min_buffer_size);
            user_data->buffer = self->i2c_buffer;
        }
        self->i2c_buffer = user_data->buffer;
        png_new_row(user_data, row, col);
    }
    if (user_data->pixels > 0 && (row != user_data->row || (user_data->has_transparency && rgba[3] == 0))) {
        png_flush(self, user_data);
        png_new_row(user_data, row, col);
    }
    if (user_data->has_transparency && rgba[3] == 0) {
        png_new_row(user_data, row, col);
        return;
    }
    *user_data->buffer++ = _swap_bytes(color565(rgba[0], rgba[1], rgba[2]));
    user_data->pixels++;
    user_data->last = col;
}
#define PNG_FILE_BUFFER_SIZE 256
static mp_obj_t jd9853_JD9853_png(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    const char *filename = mp_obj_str_get_str(args[1]);
    mp_int_t x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]);
    bool transparency = (n_args > 4) ? mp_obj_is_true(args[4]) : false;
    char buf[PNG_FILE_BUFFER_SIZE];
    int len, remain = 0;
    PNG_USER_DATA user_data = { .self = self, .ofs_x = x, .ofs_y = y, .pixels = 0, .row = 0,
        .first = 0, .last = 0, .has_transparency = transparency, .buffer = NULL };
    self->work = pngle_new(self);
    pngle_t *pngle = (pngle_t *)self->work;
    pngle_set_user_data(pngle, &user_data);
    pngle_set_draw_callback(pngle, pngle_on_draw);
    self->fp = mp_open(filename, "rb");
    while ((len = mp_readinto(self->fp, buf + remain, PNG_FILE_BUFFER_SIZE - remain)) > 0) {
        int fed = pngle_feed(pngle, buf, remain + len);
        if (fed < 0) mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("png decompress failed: %s"), pngle_error(pngle));
        remain = remain + len - fed;
        if (remain > 0) memmove(buf, buf + fed, remain);
    }
    if (user_data.pixels > 0) png_flush(self, &user_data);
    if (self->buffer_size == 0) { m_free(self->i2c_buffer); self->i2c_buffer = NULL; }
    mp_close(self->fp);
    pngle_destroy(pngle);
    self->work = NULL;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_png_obj, 4, 5, jd9853_JD9853_png);

// ========== POLÍGONOS ==========
static mp_obj_t jd9853_JD9853_polygon_center(size_t n_args, const mp_obj_t *args) {
    size_t poly_len; mp_obj_t *polygon;
    mp_obj_get_array(args[1], &poly_len, &polygon);
    mp_float_t sum = 0.0; int vsx = 0, vsy = 0;
    if (poly_len > 0) {
        for (int idx = 0; idx < poly_len; idx++) {
            size_t point_len; mp_obj_t *point;
            mp_obj_get_array(polygon[idx], &point_len, &point);
            if (point_len < 2) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
            mp_int_t v1x = mp_obj_get_int(point[0]), v1y = mp_obj_get_int(point[1]);
            mp_obj_get_array(polygon[(idx + 1) % poly_len], &point_len, &point);
            mp_int_t v2x = mp_obj_get_int(point[0]), v2y = mp_obj_get_int(point[1]);
            mp_float_t cross = v1x * v2y - v1y * v2x;
            sum += cross;
            vsx += (int)((v1x + v2x) * cross);
            vsy += (int)((v1y + v2y) * cross);
        }
        mp_float_t z = 1.0 / (3.0 * sum);
        vsx = (int)(vsx * z); vsy = (int)(vsy * z);
    } else {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
    }
    mp_obj_t center[2] = {mp_obj_new_int(vsx), mp_obj_new_int(vsy)};
    return mp_obj_new_tuple(2, center);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_polygon_center_obj, 2, 2, jd9853_JD9853_polygon_center);

#define MAX_POLY_CORNERS 32
static void RotatePolygon(Polygon *polygon, Point center, mp_float_t angle) {
    if (polygon->length == 0) return;
    mp_float_t cosAngle = MICROPY_FLOAT_C_FUN(cos)(angle);
    mp_float_t sinAngle = MICROPY_FLOAT_C_FUN(sin)(angle);
    for (int i = 0; i < polygon->length; i++) {
        mp_float_t dx = polygon->points[i].x - center.x;
        mp_float_t dy = polygon->points[i].y - center.y;
        polygon->points[i].x = center.x + (int)0.5 + (dx * cosAngle - dy * sinAngle);
        polygon->points[i].y = center.y + (int)0.5 + (dx * sinAngle + dy * cosAngle);
    }
}
static void PolygonFill(jd9853_JD9853_obj_t *self, Polygon *polygon, Point location, uint16_t color) {
    int nodes, nodeX[MAX_POLY_CORNERS], pixelY, i, j, swap;
    int minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN;
    for (i = 0; i < polygon->length; i++) {
        if (polygon->points[i].x < minX) minX = polygon->points[i].x;
        if (polygon->points[i].x > maxX) maxX = polygon->points[i].x;
        if (polygon->points[i].y < minY) minY = polygon->points[i].y;
        if (polygon->points[i].y > maxY) maxY = polygon->points[i].y;
    }
    for (pixelY = minY; pixelY < maxY; pixelY++) {
        nodes = 0; j = polygon->length - 1;
        for (i = 0; i < polygon->length; i++) {
            if ((polygon->points[i].y < pixelY && polygon->points[j].y >= pixelY) ||
                (polygon->points[j].y < pixelY && polygon->points[i].y >= pixelY)) {
                if (nodes < MAX_POLY_CORNERS) {
                    nodeX[nodes++] = (int)(polygon->points[i].x + (pixelY - polygon->points[i].y) /
                        (polygon->points[j].y - polygon->points[i].y) *
                        (polygon->points[j].x - polygon->points[i].x));
                } else {
                    mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon too complex increase MAX_POLY_CORNERS."));
                }
            }
            j = i;
        }
        i = 0;
        while (i < nodes - 1) {
            if (nodeX[i] > nodeX[i+1]) {
                swap = nodeX[i]; nodeX[i] = nodeX[i+1]; nodeX[i+1] = swap;
                if (i) i--; else i++;
            } else i++;
        }
        for (i = 0; i < nodes; i += 2) {
            if (nodeX[i] >= maxX) break;
            if (nodeX[i+1] > minX) {
                if (nodeX[i] < minX) nodeX[i] = minX;
                if (nodeX[i+1] > maxX) nodeX[i+1] = maxX;
                fast_hline(self, location.x + nodeX[i], location.y + pixelY, nodeX[i+1] - nodeX[i] + 1, color);
            }
        }
    }
}
static mp_obj_t jd9853_JD9853_polygon(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    size_t poly_len; mp_obj_t *polygon;
    mp_obj_get_array(args[1], &poly_len, &polygon);
    self->work = NULL;
    if (poly_len > 0) {
        mp_int_t x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]), color = mp_obj_get_int(args[4]);
        mp_float_t angle = (n_args > 5 && mp_obj_is_float(args[5])) ? mp_obj_float_get(args[5]) : 0.0f;
        mp_int_t cx = (n_args > 6) ? mp_obj_get_int(args[6]) : 0;
        mp_int_t cy = (n_args > 7) ? mp_obj_get_int(args[7]) : 0;
        self->work = m_malloc(poly_len * sizeof(Point));
        if (self->work) {
            Point *point = (Point *)self->work;
            for (int idx = 0; idx < poly_len; idx++) {
                size_t point_len; mp_obj_t *point_obj;
                mp_obj_get_array(polygon[idx], &point_len, &point_obj);
                if (point_len < 2) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
                point[idx].x = mp_obj_get_int(point_obj[0]);
                point[idx].y = mp_obj_get_int(point_obj[1]);
            }
            Point center = {cx, cy};
            Polygon poly = {poly_len, point};
            if (angle != 0) RotatePolygon(&poly, center, angle);
            for (int idx = 1; idx < poly_len; idx++) {
                line(self, point[idx-1].x + x, point[idx-1].y + y, point[idx].x + x, point[idx].y + y, color);
            }
            m_free(self->work); self->work = NULL;
        } else {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
        }
    } else {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_polygon_obj, 4, 8, jd9853_JD9853_polygon);

static mp_obj_t jd9853_JD9853_fill_polygon(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    size_t poly_len; mp_obj_t *polygon;
    mp_obj_get_array(args[1], &poly_len, &polygon);
    self->work = NULL;
    if (poly_len > 0) {
        mp_int_t x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]), color = mp_obj_get_int(args[4]);
        mp_float_t angle = (n_args > 5) ? mp_obj_float_get(args[5]) : 0.0f;
        mp_int_t cx = (n_args > 6) ? mp_obj_get_int(args[6]) : 0;
        mp_int_t cy = (n_args > 7) ? mp_obj_get_int(args[7]) : 0;
        self->work = m_malloc(poly_len * sizeof(Point));
        if (self->work) {
            Point *point = (Point *)self->work;
            for (int idx = 0; idx < poly_len; idx++) {
                size_t point_len; mp_obj_t *point_obj;
                mp_obj_get_array(polygon[idx], &point_len, &point_obj);
                if (point_len < 2) mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
                point[idx].x = mp_obj_get_int(point_obj[0]);
                point[idx].y = mp_obj_get_int(point_obj[1]);
            }
            Point center = {cx, cy};
            Polygon poly = {poly_len, point};
            if (angle != 0) RotatePolygon(&poly, center, angle);
            Point location = {x, y};
            PolygonFill(self, &poly, location, color);
            m_free(self->work); self->work = NULL;
        } else {
            mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
        }
    } else {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Polygon data error"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_polygon_obj, 4, 8, jd9853_JD9853_fill_polygon);

static mp_obj_t jd9853_JD9853_bounding(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_t bounds[4] = {
        mp_obj_new_int(self->min_x),
        mp_obj_new_int(self->min_y),
        (n_args > 2 && mp_obj_is_true(args[2])) ? mp_obj_new_int(self->max_x - self->min_x + 1) : mp_obj_new_int(self->max_x),
        (n_args > 2 && mp_obj_is_true(args[2])) ? mp_obj_new_int(self->max_y - self->min_y + 1) : mp_obj_new_int(self->max_y)
    };
    if (n_args > 1) {
        self->bounding = mp_obj_is_true(args[1]) ? 1 : 0;
        self->min_x = self->width; self->min_y = self->height;
        self->max_x = 0; self->max_y = 0;
    }
    return mp_obj_new_tuple(4, bounds);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_bounding_obj, 1, 3, jd9853_JD9853_bounding);

// ========== NUEVAS FUNCIONES MEJORADAS (gradient, triangle, etc.) ==========
static mp_obj_t jd9853_JD9853_gradient_fill(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x = mp_obj_get_int(args[1]), y = mp_obj_get_int(args[2]);
    mp_int_t w = mp_obj_get_int(args[3]), h = mp_obj_get_int(args[4]);
    mp_int_t color1 = mp_obj_get_int(args[5]), color2 = mp_obj_get_int(args[6]);
    mp_int_t direction = mp_obj_get_int(args[7]); // 0=horizontal, 1=vertical
    int16_t x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int16_t x1 = x + w - 1, y1 = y + h - 1;
    if (x1 >= self->width) x1 = self->width - 1;
    if (y1 >= self->height) y1 = self->height - 1;
    int16_t width = x1 - x0 + 1, height = y1 - y0 + 1;
    if (direction == 0) {
        for (int16_t row = y0; row <= y1; row++) {
            for (int16_t col = x0; col <= x1; col++) {
                float ratio = (float)(col - x0) / width;
                uint8_t r1 = (color1 >> 8) & 0xF8, g1 = (color1 >> 3) & 0xFC, b1 = (color1 << 3) & 0xF8;
                uint8_t r2 = (color2 >> 8) & 0xF8, g2 = (color2 >> 3) & 0xFC, b2 = (color2 << 3) & 0xF8;
                uint8_t r = r1 + (uint8_t)((r2 - r1) * ratio);
                uint8_t g = g1 + (uint8_t)((g2 - g1) * ratio);
                uint8_t b = b1 + (uint8_t)((b2 - b1) * ratio);
                draw_pixel(self, col, row, color565(r,g,b));
            }
        }
    } else {
        for (int16_t row = y0; row <= y1; row++) {
            float ratio = (float)(row - y0) / height;
            uint8_t r1 = (color1 >> 8) & 0xF8, g1 = (color1 >> 3) & 0xFC, b1 = (color1 << 3) & 0xF8;
            uint8_t r2 = (color2 >> 8) & 0xF8, g2 = (color2 >> 3) & 0xFC, b2 = (color2 << 3) & 0xF8;
            uint8_t r = r1 + (uint8_t)((r2 - r1) * ratio);
            uint8_t g = g1 + (uint8_t)((g2 - g1) * ratio);
            uint8_t b = b1 + (uint8_t)((b2 - b1) * ratio);
            fast_hline(self, x0, row, width, color565(r,g,b));
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_gradient_fill_obj, 8, 8, jd9853_JD9853_gradient_fill);

static mp_obj_t jd9853_JD9853_draw_icon(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_buffer_info_t icon_info;
    mp_get_buffer_raise(args[1], &icon_info, MP_BUFFER_READ);
    mp_int_t x = mp_obj_get_int(args[2]), y = mp_obj_get_int(args[3]);
    mp_int_t size = mp_obj_get_int(args[4]), color = mp_obj_get_int(args[5]);
    const uint8_t *icon = icon_info.buf;
    int icon_size = (size + 7) / 8;
    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            if (icon[row * icon_size + (col / 8)] & (0x80 >> (col % 8))) {
                draw_pixel(self, x + col, y + row, color);
            }
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_draw_icon_obj, 6, 6, jd9853_JD9853_draw_icon);

static mp_obj_t jd9853_JD9853_get_info(mp_obj_t self_in) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_t dict = mp_obj_new_dict(8);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_width), mp_obj_new_int(self->width));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_height), mp_obj_new_int(self->height));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_rotation), mp_obj_new_int(self->rotation));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_buffer_size), mp_obj_new_int(self->buffer_size));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_options), mp_obj_new_int(self->options));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_madctl), mp_obj_new_int(self->madctl));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_inversion), mp_obj_new_bool(self->inversion));
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_1(jd9853_JD9853_get_info_obj, jd9853_JD9853_get_info);

static mp_obj_t jd9853_JD9853_triangle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x0 = mp_obj_get_int(args[1]), y0 = mp_obj_get_int(args[2]);
    mp_int_t x1 = mp_obj_get_int(args[3]), y1 = mp_obj_get_int(args[4]);
    mp_int_t x2 = mp_obj_get_int(args[5]), y2 = mp_obj_get_int(args[6]);
    mp_int_t color = mp_obj_get_int(args[7]);
    line(self, x0, y0, x1, y1, color);
    line(self, x1, y1, x2, y2, color);
    line(self, x2, y2, x0, y0, color);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_triangle_obj, 8, 8, jd9853_JD9853_triangle);

static mp_obj_t jd9853_JD9853_fill_triangle(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t x0 = mp_obj_get_int(args[1]), y0 = mp_obj_get_int(args[2]);
    mp_int_t x1 = mp_obj_get_int(args[3]), y1 = mp_obj_get_int(args[4]);
    mp_int_t x2 = mp_obj_get_int(args[5]), y2 = mp_obj_get_int(args[6]);
    mp_int_t color = mp_obj_get_int(args[7]);
    mp_obj_t points[3];
    mp_obj_t p0[2] = {mp_obj_new_int(x0), mp_obj_new_int(y0)};
    mp_obj_t p1[2] = {mp_obj_new_int(x1), mp_obj_new_int(y1)};
    mp_obj_t p2[2] = {mp_obj_new_int(x2), mp_obj_new_int(y2)};
    points[0] = mp_obj_new_tuple(2, p0);
    points[1] = mp_obj_new_tuple(2, p1);
    points[2] = mp_obj_new_tuple(2, p2);
    mp_obj_t polygon = mp_obj_new_tuple(3, points);
    mp_obj_t fill_args[] = {args[0], polygon, mp_obj_new_int(0), mp_obj_new_int(0), args[7]};
    jd9853_JD9853_fill_polygon(5, fill_args);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_triangle_obj, 8, 8, jd9853_JD9853_fill_triangle);

static mp_obj_t jd9853_JD9853_ellipse(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t xc = mp_obj_get_int(args[1]), yc = mp_obj_get_int(args[2]);
    mp_int_t rx = mp_obj_get_int(args[3]), ry = mp_obj_get_int(args[4]);
    mp_int_t color = mp_obj_get_int(args[5]);
    int x = 0, y = ry, rx2 = rx * rx, ry2 = ry * ry;
    int err = ry2 - rx2 * ry;
    draw_pixel(self, xc + x, yc + y, color);
    draw_pixel(self, xc - x, yc + y, color);
    draw_pixel(self, xc + x, yc - y, color);
    draw_pixel(self, xc - x, yc - y, color);
    while (2 * x * ry2 < 2 * y * rx2) {
        x++;
        if (err < 0) err += 2 * ry2 * x + ry2;
        else { y--; err += 2 * ry2 * x + ry2 - 2 * rx2 * y; }
        draw_pixel(self, xc + x, yc + y, color);
        draw_pixel(self, xc - x, yc + y, color);
        draw_pixel(self, xc + x, yc - y, color);
        draw_pixel(self, xc - x, yc - y, color);
    }
    err = rx2 * (y - 1) * (y - 1) + ry2 * (x + 1) * (x + 1) - rx2 * ry2;
    while (y > 0) {
        y--;
        if (err > 0) err -= 2 * rx2 * y + rx2;
        else { x++; err += -2 * rx2 * y - rx2 + 2 * ry2 * x; }
        draw_pixel(self, xc + x, yc + y, color);
        draw_pixel(self, xc - x, yc + y, color);
        draw_pixel(self, xc + x, yc - y, color);
        draw_pixel(self, xc - x, yc - y, color);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_ellipse_obj, 6, 6, jd9853_JD9853_ellipse);

static mp_obj_t jd9853_JD9853_fill_ellipse(size_t n_args, const mp_obj_t *args) {
    jd9853_JD9853_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t xc = mp_obj_get_int(args[1]), yc = mp_obj_get_int(args[2]);
    mp_int_t rx = mp_obj_get_int(args[3]), ry = mp_obj_get_int(args[4]);
    mp_int_t color = mp_obj_get_int(args[5]);
    int x = 0, y = ry, rx2 = rx * rx, ry2 = ry * ry;
    int err = ry2 - rx2 * ry;
    fast_hline(self, xc - x, yc + y, 2 * x + 1, color);
    fast_hline(self, xc - x, yc - y, 2 * x + 1, color);
    while (2 * x * ry2 < 2 * y * rx2) {
        x++;
        if (err < 0) err += 2 * ry2 * x + ry2;
        else { y--; err += 2 * ry2 * x + ry2 - 2 * rx2 * y; }
        fast_hline(self, xc - x, yc + y, 2 * x + 1, color);
        fast_hline(self, xc - x, yc - y, 2 * x + 1, color);
    }
    err = rx2 * (y - 1) * (y - 1) + ry2 * (x + 1) * (x + 1) - rx2 * ry2;
    while (y > 0) {
        y--;
        if (err > 0) err -= 2 * rx2 * y + rx2;
        else { x++; err += -2 * rx2 * y - rx2 + 2 * ry2 * x; }
        fast_hline(self, xc - x, yc + y, 2 * x + 1, color);
        fast_hline(self, xc - x, yc - y, 2 * x + 1, color);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(jd9853_JD9853_fill_ellipse_obj, 6, 6, jd9853_JD9853_fill_ellipse);

// ========== DICCIONARIO DE MÉTODOS ==========
static const mp_rom_map_elem_t jd9853_JD9853_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&jd9853_JD9853_write_obj)},
    {MP_ROM_QSTR(MP_QSTR_write_len), MP_ROM_PTR(&jd9853_JD9853_write_len_obj)},
    {MP_ROM_QSTR(MP_QSTR_hard_reset), MP_ROM_PTR(&jd9853_JD9853_hard_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_soft_reset), MP_ROM_PTR(&jd9853_JD9853_soft_reset_obj)},
    {MP_ROM_QSTR(MP_QSTR_sleep_mode), MP_ROM_PTR(&jd9853_JD9853_sleep_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_inversion_mode), MP_ROM_PTR(&jd9853_JD9853_inversion_mode_obj)},
    {MP_ROM_QSTR(MP_QSTR_map_bitarray_to_rgb565), MP_ROM_PTR(&jd9853_map_bitarray_to_rgb565_obj)},
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
    {MP_ROM_QSTR(MP_QSTR_jpg_decode), MP_ROM_PTR(&jd9853_JD9853_jpg_decode_obj)},
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
};
static MP_DEFINE_CONST_DICT(jd9853_JD9853_locals_dict, jd9853_JD9853_locals_dict_table);

// ========== DEFINICIÓN DEL TIPO ==========
#ifdef MP_OBJ_TYPE_GET_SLOT
MP_DEFINE_CONST_OBJ_TYPE(
    jd9853_JD9853_type,
    MP_QSTR_JD9853,
    MP_TYPE_FLAG_NONE,
    print, jd9853_JD9853_print,
    make_new, jd9853_JD9853_make_new,
    locals_dict, (mp_obj_dict_t *)&jd9853_JD9853_locals_dict);
#else
const mp_obj_type_t jd9853_JD9853_type = {
    {&mp_type_type},
    .name = MP_QSTR_JD9853,
    .print = jd9853_JD9853_print,
    .make_new = jd9853_JD9853_make_new,
    .locals_dict = (mp_obj_dict_t *)&jd9853_JD9853_locals_dict,
};
#endif

// ========== FUNCIÓN make_new ==========
mp_obj_t jd9853_JD9853_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum {
        ARG_spi, ARG_width, ARG_height, ARG_reset, ARG_dc, ARG_cs,
        ARG_backlight, ARG_rotations, ARG_rotation, ARG_custom_init,
        ARG_color_order, ARG_inversion, ARG_options, ARG_buffer_size
    };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_spi, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_width, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0}},
        {MP_QSTR_height, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = 0}},
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

    self->spi_obj = (mp_obj_base_t *)MP_OBJ_TO_PTR(args[ARG_spi].u_obj);
    self->display_width = args[ARG_width].u_int;
    self->width = args[ARG_width].u_int;
    self->display_height = args[ARG_height].u_int;
    self->height = args[ARG_height].u_int;
    self->vscsad = 0;
    self->tfa = self->vsa = self->bfa = 0;

    self->rotations = NULL;
    self->rotations_len = 4;
    if (args[ARG_rotations].u_obj != MP_OBJ_NULL) {
        size_t len; mp_obj_t *rotations_array;
        mp_obj_get_array(args[ARG_rotations].u_obj, &len, &rotations_array);
        self->rotations_len = len;
        self->rotations = m_new(jd9853_rotation_t, self->rotations_len);
        for (int i = 0; i < self->rotations_len; i++) {
            mp_obj_t *rotation_tuple; size_t rot_len;
            mp_obj_tuple_get(rotations_array[i], &rot_len, &rotation_tuple);
            if (rot_len != 5) mp_raise_ValueError(MP_ERROR_TEXT("rotations tuple must have 5 elements"));
            self->rotations[i].madctl = mp_obj_get_int(rotation_tuple[0]);
            self->rotations[i].width = mp_obj_get_int(rotation_tuple[1]);
            self->rotations[i].height = mp_obj_get_int(rotation_tuple[2]);
            self->rotations[i].colstart = mp_obj_get_int(rotation_tuple[3]);
            self->rotations[i].rowstart = mp_obj_get_int(rotation_tuple[4]);
        }
    }
    self->rotation = args[ARG_rotation].u_int % self->rotations_len;
    self->custom_init = args[ARG_custom_init].u_obj;
    self->color_order = args[ARG_color_order].u_int;
    self->inversion = args[ARG_inversion].u_bool;
    self->options = args[ARG_options].u_int & 0xff;
    self->buffer_size = args[ARG_buffer_size].u_int;

    if (self->buffer_size) {
        self->i2c_buffer = m_malloc(self->buffer_size);
    } else {
        self->i2c_buffer = self->static_buffer;   // buffer interno
    }

    if (args[ARG_dc].u_obj == MP_OBJ_NULL) mp_raise_ValueError(MP_ERROR_TEXT("must specify dc pin"));

    self->reset = (args[ARG_reset].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_reset].u_obj) : GPIO_NUM_NC;
    self->dc = mp_hal_get_pin_obj(args[ARG_dc].u_obj);
    self->cs = (args[ARG_cs].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_cs].u_obj) : GPIO_NUM_NC;
    self->backlight = (args[ARG_backlight].u_obj != MP_OBJ_NULL) ? mp_hal_get_pin_obj(args[ARG_backlight].u_obj) : GPIO_NUM_NC;

    self->bounding = 0;
    self->min_x = self->display_width;
    self->min_y = self->display_height;
    self->max_x = 0;
    self->max_y = 0;

    return MP_OBJ_FROM_PTR(self);
}

// ========== MÓDULO ==========
static const mp_map_elem_t jd9853_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_jd9853)},
    {MP_ROM_QSTR(MP_QSTR_color565), (mp_obj_t)&jd9853_color565_obj},
    {MP_ROM_QSTR(MP_QSTR_map_bitarray_to_rgb565), (mp_obj_t)&jd9853_map_bitarray_to_rgb565_obj},
    {MP_ROM_QSTR(MP_QSTR_JD9853), (mp_obj_t)&jd9853_JD9853_type},
    {MP_ROM_QSTR(MP_QSTR_BLACK), MP_ROM_INT(BLACK)},
    {MP_ROM_QSTR(MP_QSTR_BLUE), MP_ROM_INT(BLUE)},
    {MP_ROM_QSTR(MP_QSTR_RED), MP_ROM_INT(RED)},
    {MP_ROM_QSTR(MP_QSTR_GREEN), MP_ROM_INT(GREEN)},
    {MP_ROM_QSTR(MP_QSTR_CYAN), MP_ROM_INT(CYAN)},
    {MP_ROM_QSTR(MP_QSTR_MAGENTA), MP_ROM_INT(MAGENTA)},
    {MP_ROM_QSTR(MP_QSTR_YELLOW), MP_ROM_INT(YELLOW)},
    {MP_ROM_QSTR(MP_QSTR_WHITE), MP_ROM_INT(WHITE)},
    {MP_ROM_QSTR(MP_QSTR_ORANGE), MP_ROM_INT(ORANGE)},
    {MP_ROM_QSTR(MP_QSTR_PURPLE), MP_ROM_INT(PURPLE)},
    {MP_ROM_QSTR(MP_QSTR_PINK), MP_ROM_INT(PINK)},
    {MP_ROM_QSTR(MP_QSTR_GRAY), MP_ROM_INT(GRAY)},
    {MP_ROM_QSTR(MP_QSTR_DARKGRAY), MP_ROM_INT(DARKGRAY)},
    {MP_ROM_QSTR(MP_QSTR_BROWN), MP_ROM_INT(BROWN)},
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
    .globals = (mp_obj_dict_t *)&mp_module_jd9853_globals,
};

MP_REGISTER_MODULE(MP_QSTR_jd9853, mp_module_jd9853);
