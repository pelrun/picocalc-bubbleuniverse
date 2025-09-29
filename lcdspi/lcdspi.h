#ifndef LCDSPI_H
#define LCDSPI_H
#include <hardware/spi.h>

// #define LCD_SPI_SPEED   6000000
// #define LCD_SPI_SPEED   25000000
#define LCD_SPI_SPEED 50000000

#define BYTES_PER_PIXEL 2

#define Pico_LCD_SPI_MOD spi1

#define Pico_LCD_SCK 10  //
#define Pico_LCD_TX 11   // MOSI
#define Pico_LCD_RX 12   // MISO
#define Pico_LCD_CS 13   //
#define Pico_LCD_DC 14
#define Pico_LCD_RST 15

#define LCD_WIDTH 320
#define LCD_HEIGHT 320

#define ORIENT_NORMAL 0

#define RGB565(red, green, blue) \
  __builtin_bswap16((((red) & 0xF8) << 8) | (((green) & 0xFC) << 3) | ((blue)>>3))
#define RGB888(red, green, blue) \
  (unsigned int)((((blue) & 0xFF) << 16) | (((green) & 0xFF) << 8) | (red))

#if BYTES_PER_PIXEL == 2
#define RGB(red, green, blue) RGB565(red, green, blue)
#elif BYTES_PER_PIXEL == 3
#define RGB(red, green, blue) RGB888(red, green, blue)
#endif

#define WHITE RGB(255, 255, 255)
#define YELLOW RGB(255, 255, 0)
#define LILAC RGB(255, 128, 255)
#define BROWN RGB(255, 128, 0)
#define FUCHSIA RGB(255, 64, 255)
#define RUST RGB(255, 64, 0)
#define MAGENTA RGB(255, 0, 255)
#define RED RGB(255, 0, 0)
#define CYAN RGB(0, 255, 255)
#define GREEN RGB(0, 255, 0)
#define CERULEAN RGB(0, 128, 255)
#define MIDGREEN RGB(0, 128, 0)
#define COBALT RGB(0, 64, 255)
#define MYRTLE RGB(0, 64, 0)
#define BLUE RGB(0, 0, 255)
#define BLACK RGB(0, 0, 0)
#define BROWN RGB(255, 128, 0)
#define GRAY RGB(128, 128, 128)
#define LITEGRAY RGB(210, 210, 210)
#define ORANGE RGB(0xff, 0xA5, 0)
#define PINK RGB(0xFF, 0xA0, 0xAB)
#define GOLD RGB(0xFF, 0xD7, 0x00)
#define SALMON RGB(0xFA, 0x80, 0x72)
#define BEIGE RGB(0xF5, 0xF5, 0xDC)

//void define_region_spi(int xstart, int ystart, int xend, int yend, int rw);

void draw_rect_spi(int x1, int y1, int x2, int y2, int c);
void draw_line_spi(int x1, int y1, int x2, int y2, int color);

void lcd_print_string_color(char *s, int fg, int bg);
void draw_battery_icon(int x0, int y0, int level);

void draw_bitmap_spi(int x1, int y1, int width, int height, int scale, int fc, int bc,
                     unsigned char *bitmap);
void draw_buffer_spi(int x1, int y1, int x2, int y2, void *p);

void lcd_init();
void lcd_clear();

void lcd_set_cursor(int x, int y);
void lcd_putc(uint8_t devn, uint8_t c);
char lcd_put_char(char c, int flush);
void lcd_print_char(int fc, int bc, char c, int orientation);
void lcd_print_string(char *s);

void lcd_sleeping(uint8_t devn);

#endif
