#ifndef __JD9853_H__
#define __JD9853_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "py/obj.h"
#include "py/mphal.h"

// ========== Modos de color (solo 18 bits) ==========
#define COLOR_MODE_18BIT    0x66   // RGB666
#define COLOR_MODE_262K     0x66

// ========== Comandos ==========
#define JD9853_NOP     0x00
#define JD9853_SWRESET 0x01
#define JD9853_RDDID   0x04
#define JD9853_RDDST   0x09

#define JD9853_SLPIN   0x10
#define JD9853_SLPOUT  0x11
#define JD9853_PTLON   0x12
#define JD9853_NORON   0x13

#define JD9853_INVOFF  0x20
#define JD9853_INVON   0x21
#define JD9853_DISPOFF 0x28
#define JD9853_DISPON  0x29
#define JD9853_CASET   0x2A
#define JD9853_RASET   0x2B
#define JD9853_RAMWR   0x2C
#define JD9853_RAMRD   0x2E

#define JD9853_PTLAR   0x30
#define JD9853_VSCRDEF 0x33
#define JD9853_COLMOD  0x3A
#define JD9853_MADCTL  0x36
#define JD9853_VSCRSADD 0x37

#define JD9853_MADCTL_MY  0x80
#define JD9853_MADCTL_MX  0x40
#define JD9853_MADCTL_MV  0x20
#define JD9853_MADCTL_ML  0x10
#define JD9853_MADCTL_MH  0x04
#define JD9853_MADCTL_RGB 0x00
#define JD9853_MADCTL_BGR 0x08

#define JD9853_RDID1   0xDA
#define JD9853_RDID2   0xDB
#define JD9853_RDID3   0xDC
#define JD9853_RDID4   0xDD

// ========== Colores en formato 24 bits (0xRRGGBB) ==========
#define BLACK   0x000000
#define BLUE    0x0000FF
#define RED     0xFF0000
#define GREEN   0x00FF00
#define CYAN    0x00FFFF
#define MAGENTA 0xFF00FF
#define YELLOW  0xFFFF00
#define WHITE   0xFFFFFF
#define ORANGE  0xFFC95B0
#define PURPLE  0x800080
#define PINK    0xFFCCFF
#define GRAY    0x808080
#define DARKGRAY 0x404040
#define BROWN   0xA52A2A

// ========== Opciones ==========
#define OPTIONS_WRAP_V 0x01
#define OPTIONS_WRAP_H 0x02
#define OPTIONS_WRAP   0x03

#define GRADIENT_HORIZONTAL 0
#define GRADIENT_VERTICAL   1

// ========== Estructuras auxiliares ==========
typedef struct _Point {
    mp_float_t x;
    mp_float_t y;
} Point;

typedef struct _Polygon {
    int length;
    Point *points;
} Polygon;

typedef struct _jd9853_rotation_t {
    uint8_t madctl;
    uint16_t width;
    uint16_t height;
    uint16_t colstart;
    uint16_t rowstart;
} jd9853_rotation_t;

// ========== Objeto principal ==========
typedef struct _jd9853_JD9853_obj_t {
    mp_obj_base_t base;
    mp_obj_base_t *spi_obj;
    mp_file_t *fp;

    uint8_t *i2c_buffer;           // buffer de trabajo (bytes)
    uint16_t vscsad;
    uint16_t hscsad;

    // Scroll vertical
    uint16_t tfa;
    uint16_t vsa;
    uint16_t bfa;

    // Buffers internos
    uint8_t static_buffer[4096];

    // Trabajo para PNG/JPG
    void *work;
    uint8_t *scanline_ringbuf;
    uint8_t *palette;
    uint8_t *trans_palette;
    uint8_t *gamma_table;

    size_t buffer_size;
    uint16_t display_width;
    uint16_t width;
    uint16_t display_height;
    uint16_t height;
    uint8_t colstart;
    uint8_t rowstart;
    uint8_t rotation;
    jd9853_rotation_t *rotations;
    uint8_t rotations_len;
    mp_obj_t custom_init;
    uint8_t color_order;
    bool inversion;
    uint8_t madctl;
    uint8_t options;

    mp_hal_pin_obj_t reset;
    mp_hal_pin_obj_t dc;
    mp_hal_pin_obj_t cs;
    mp_hal_pin_obj_t backlight;

    uint8_t bounding;
    uint16_t min_x;
    uint16_t min_y;
    uint16_t max_x;
    uint16_t max_y;

} jd9853_JD9853_obj_t;

// ========== Constructor ==========
mp_obj_t jd9853_JD9853_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);

// ========== Funciones de dibujo (exportadas, ahora con uint32_t) ==========
extern void draw_pixel(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, uint32_t color);
extern void fast_hline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t w, uint32_t color);
extern void line(jd9853_JD9853_obj_t *self, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color);
extern uint32_t color_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif // __JD9853_H__
