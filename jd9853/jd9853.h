#ifndef __JD9853_H__
#define __JD9853_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "py/obj.h"
#include "py/mphal.h"

// color modes
#define COLOR_MODE_65K      0x55
#define COLOR_MODE_262K     0x66
#define COLOR_MODE_12BIT    0x33
#define COLOR_MODE_16BIT    0x55
#define COLOR_MODE_18BIT    0x66
#define COLOR_MODE_16M      0x07

// commands
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
#define JD9853_VSCRSADD 0x37   // nota: en el .c se usa JD9853_VSCRSADD (sin 'S' en medio? se define como 0x37)

#define JD9853_MADCTL_MY  0x80  // Page Address Order
#define JD9853_MADCTL_MX  0x40  // Column Address Order
#define JD9853_MADCTL_MV  0x20  // Page/Column Order
#define JD9853_MADCTL_ML  0x10  // Line Address Order
#define JD9853_MADCTL_MH  0x04  // Display Data Latch Order
#define JD9853_MADCTL_RGB 0x00
#define JD9853_MADCTL_BGR 0x08

#define JD9853_RDID1   0xDA
#define JD9853_RDID2   0xDB
#define JD9853_RDID3   0xDC
#define JD9853_RDID4   0xDD

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFD20
#define PURPLE  0x780F
#define PINK    0xFC97
#define GRAY    0x8410
#define DARKGRAY 0x4208
#define BROWN   0xA145

#define OPTIONS_WRAP_V 0x01
#define OPTIONS_WRAP_H 0x02
#define OPTIONS_WRAP   0x03

// Gradient direction constants
#define GRADIENT_HORIZONTAL 0
#define GRADIENT_VERTICAL   1

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

// this is the actual C-structure for our new object
typedef struct _jd9853_JD9853_obj_t {
    mp_obj_base_t base;
    mp_obj_base_t *spi_obj;
    mp_file_t *fp;              // file object
    uint16_t *i2c_buffer;       // resident buffer if buffer_size given
    uint16_t vscsad;            // vertical scroll start address (for scroll)
    uint16_t hscsad;            // horizontal scroll position (for horizontal scroll)

    // ========== CAMPOS AÑADIDOS PARA SCROLL VERTICAL Y BUFFER ESTÁTICO ==========
    uint16_t tfa;               // Top Fixed Area (de VSCRDEF)
    uint16_t vsa;               // Vertical Scrolling Area (de VSCRDEF)
    uint16_t bfa;               // Bottom Fixed Area (de VSCRDEF)
    uint8_t static_buffer[4096]; // buffer interno estático (evita malloc en write/text/bitmap)
    // ===========================================================================

    // m_malloc'd pointers
    void *work;                 // work buffer for jpg & png decoding
    uint8_t *scanline_ringbuf;  // png scanline_ringbuf
    uint8_t *palette;           // png palette
    uint8_t *trans_palette;     // png trans_palette
    uint8_t *gamma_table;       // png gamma_table

    size_t buffer_size;         // resident buffer size, 0=dynamic (usa static_buffer)
    uint16_t display_width;     // physical width
    uint16_t width;             // logical width (after rotation)
    uint16_t display_height;    // physical height
    uint16_t height;            // logical height (after rotation)
    uint8_t colstart;
    uint8_t rowstart;
    uint8_t rotation;
    jd9853_rotation_t *rotations;   // list of rotation tuples [(madctl, colstart, rowstart)]
    uint8_t rotations_len;          // number of rotations
    mp_obj_t custom_init;           // custom init sequence
    uint8_t color_order;
    bool inversion;
    uint8_t madctl;
    uint8_t options;            // options bit array
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

// Constructor
mp_obj_t jd9853_JD9853_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);

// Funciones de dibujo (exportadas para uso interno)
extern void draw_pixel(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, uint16_t color);
extern void fast_hline(jd9853_JD9853_obj_t *self, int16_t x, int16_t y, int16_t w, uint16_t color);
extern void line(jd9853_JD9853_obj_t *self, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color);
extern uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

#ifdef  __cplusplus
}
#endif /*  __cplusplus */

#endif  /*  __JD9853_H__ */
