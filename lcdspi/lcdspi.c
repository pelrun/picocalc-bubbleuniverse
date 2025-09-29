#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/timer.h>
#include <pico/multicore.h>

#include "lcdspi.h"

#define TFT_SLPOUT 0x11
#define TFT_INVOFF 0x20
#define TFT_INVON 0x21

#define TFT_DISPOFF 0x28
#define TFT_DISPON 0x29
#define TFT_MADCTL 0x36

#define TFT_COLMOD 0x3A
#define TFT_IFMODE 0xB0
#define TFT_FRMCTR1 0xB1
#define TFT_DIC 0xB4
#define TFT_DFC 0xB6
#define TFT_EM 0xB7
#define TFT_PWR1 0xC0
#define TFT_PWR2 0xC1
#define TFT_VCMPCTL 0xC5
#define TFT_PGC 0xE0
#define TFT_NGC 0xE1
#define TFT_SETIMGFUNC 0xE9
#define TFT_ADJCON3 0xF7

#define ILI9341_MADCTL_MX 0x40
#define ILI9341_MADCTL_BGR 0x08

#define ILI9341_COLADDRSET 0x2A
#define ILI9341_PAGEADDRSET 0x2B
#define ILI9341_MEMORYWRITE 0x2C
#define ILI9341_RAMRD 0x2E

#define ILI9341_Portrait ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR

#include "fonts/font1.h"

struct font_s
{
  uint8_t width;
  uint8_t height;
  uint8_t start;
  uint8_t length;
  uint8_t glyphdata[];
};

struct font_s *font;

static int gui_fcolour;
static int gui_bcolour;
static short current_x = 0, current_y = 0;  // next char location
int lcd_char_pos = 0;

static inline void clamp(int *a1, int *a2, int max)
{
  if (*a1 < 0) *a1 = 0;
  if (*a2 < 0) *a2 = 0;
  if (*a1 >= max) *a1 = max - 1;
  if (*a2 >= max) *a2 = max - 1;

  if (*a2 < *a1)
  {
    int t = *a1;
    *a1   = *a2;
    *a2   = t;
  }
}

void __not_in_flash_func(spi_write_fast)(spi_inst_t *spi, const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; ++i)
  {
    while (!spi_is_writable(spi)) tight_loop_contents();
    spi_get_hw(spi)->dr = (uint32_t)src[i];
  }
}

void __not_in_flash_func(spi_finish)(spi_inst_t *spi)
{
  while (spi_is_readable(spi)) (void)spi_get_hw(spi)->dr;
  while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS) tight_loop_contents();
  while (spi_is_readable(spi)) (void)spi_get_hw(spi)->dr;

  // Don't leave overrun flag set
  spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;
}

unsigned char __not_in_flash_func(hw1_swap_spi)(unsigned char data_out)
{
  unsigned char data_in = 0;
  spi_write_read_blocking(spi1, &data_out, &data_in, 1);
  return data_in;
}

void hw_read_spi(unsigned char *buff, int cnt)
{
  spi_read_blocking(Pico_LCD_SPI_MOD, 0xff, buff, cnt);
}

void hw_send_spi(const void *buff, int cnt) { spi_write_blocking(Pico_LCD_SPI_MOD, buff, cnt); }

void set_font(const void *newfont) { font = (struct font_s *)newfont; }

void lcd_spi_raise_cs(void) { gpio_put(Pico_LCD_CS, 1); }
void lcd_spi_lower_cs(void) { gpio_put(Pico_LCD_CS, 0); }

void define_region_spi(int xstart, int ystart, int xend, int yend, int rw)
{
  unsigned char coord[4];
  lcd_spi_lower_cs();

  gpio_put(Pico_LCD_DC, 0);
  hw_send_spi(&(uint8_t){ILI9341_COLADDRSET}, 1);

  coord[0] = xstart >> 8;
  coord[1] = xstart;
  coord[2] = xend >> 8;
  coord[3] = xend;
  gpio_put(Pico_LCD_DC, 1);
  hw_send_spi(coord, 4);

  gpio_put(Pico_LCD_DC, 0);
  hw_send_spi(&(uint8_t){ILI9341_PAGEADDRSET}, 1);

  coord[0] = ystart >> 8;
  coord[1] = ystart;
  coord[2] = yend >> 8;
  coord[3] = yend;
  gpio_put(Pico_LCD_DC, 1);
  hw_send_spi(coord, 4);

  gpio_put(Pico_LCD_DC, 0);
  if (rw)
  {
    hw_send_spi(&(uint8_t){ILI9341_MEMORYWRITE}, 1);
  }
  else
  {
    hw_send_spi(&(uint8_t){ILI9341_RAMRD}, 1);
  }

  gpio_put(Pico_LCD_DC, 1);
}

void draw_pixel(int x, int y, int c)
{
  define_region_spi(x, y, x, y, 1);
  hw_send_spi(&c, BYTES_PER_PIXEL);
}

void draw_line_spi(int x1, int y1, int x2, int y2, int color)
{
  int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
  int dy = abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
  int err = (dx > dy ? dx : -dy) / 2, e2;

  while (1)
  {
    draw_pixel(x1, y1, color);  // Draw a single pixel
    if (x1 == x2 && y1 == y2) break;
    e2 = err;
    if (e2 > -dx)
    {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dy)
    {
      err += dx;
      y1 += sy;
    }
  }
}

void read_buffer_spi(int x1, int y1, int x2, int y2, unsigned char *p)
{
  clamp(&x1, &x2, LCD_WIDTH);
  clamp(&y1, &y2, LCD_HEIGHT);

  int N = (x2 - x1 + 1) * (y2 - y1 + 1) * BYTES_PER_PIXEL;

  define_region_spi(x1, y1, x2, y2, 0);

  spi_set_baudrate(Pico_LCD_SPI_MOD, 6000000);
  hw_read_spi((uint8_t *)p, 1);
  hw_read_spi((uint8_t *)p, N);
  gpio_put(Pico_LCD_DC, 0);
  lcd_spi_raise_cs();
  spi_set_baudrate(Pico_LCD_SPI_MOD, LCD_SPI_SPEED);
}

void draw_buffer_spi(int x1, int y1, int x2, int y2, void *p)
{
  int i, t;

  clamp(&x1, &x2, LCD_WIDTH);
  clamp(&y1, &y2, LCD_HEIGHT);

  // Calculate total number of pixels
  int pixelCount = (x2 - x1 + 1) * (y2 - y1 + 1);

  define_region_spi(x1, y1, x2, y2, 1);

  hw_send_spi(p, pixelCount * BYTES_PER_PIXEL);

  lcd_spi_raise_cs();
}

// Print a bitmap on the video output
//     x, y - the top left of the char
//     width, height - size of the bitmap
//     scale - how much to scale the bitmap
//	   fc, bc - foreground and background colour
//     bitmap - pointer to the bitmap
void draw_bitmap_spi(int x1, int y1, int width, int height, int scale, int fc, int bc,
                     unsigned char *bitmap)
{
  if (x1 >= LCD_WIDTH || y1 >= LCD_HEIGHT) return;

  int left   = x1;
  int top    = y1;
  int right  = x1 + width;   // * scale;
  int bottom = y1 + height;  // * scale;

  if (right < 0 || bottom < 0) return;

  clamp(&left, &right, LCD_WIDTH);
  clamp(&top, &bottom, LCD_HEIGHT);

  if (bc >= 0)
  {
    define_region_spi(left, top, right - 1, bottom - 1, 1);

    char line_buffer[width * BYTES_PER_PIXEL];

    for (int row = top - y1; row < height; row++)
    {
      char *p = line_buffer;
      for (int col = left - x1; col < width; col++)
      {
        int cbit   = 8 - (col & 0xF);
        int cbyte  = (row * width / 8) + (col >> 3);
        int colour = ((bitmap[cbyte] >> cbit) & 1) ? fc : bc;
        *p++       = colour;
        *p++       = colour >> 8;
#if BYTES_PER_PIXEL == 3
        *p++ = colour >> 16;
#endif
      }
      hw_send_spi(line_buffer, width * BYTES_PER_PIXEL);
    }

    lcd_spi_raise_cs();  // set CS high
  }
  else
  {
    // transparent background
    for (int row = top - y1; row < height; row++)
    {
      for (int col = left - x1; col < width; col++)
      {
        int cbit  = 8 - (col & 0xF);
        int cbyte = (row * width / 8) + (col >> 3);
        if ((bitmap[cbyte] >> cbit) & 1)
        {
          draw_pixel(left + col, top + row, fc);
        }
      }
    }
  }
}

// Draw a filled rectangle
// this is the basic drawing promitive used by most drawing routines
//    x1, y1, x2, y2 - the coordinates
//    c - the colour
void draw_rect_spi(int x1, int y1, int x2, int y2, int c)
{
  if (x1 == x2 && y1 == y2)
  {
    if (x1 < 0) return;
    if (x1 >= LCD_WIDTH) return;
    if (y1 < 0) return;
    if (y1 >= LCD_HEIGHT) return;
    draw_pixel(x1, y1, c);
  }
  else
  {
    clamp(&x1, &x2, LCD_WIDTH);
    clamp(&y1, &y2, LCD_HEIGHT);

    define_region_spi(x1, y1, x2, y2, 1);

    int line_bytes = (x2 - x1 + 1) * BYTES_PER_PIXEL;
    uint8_t line_buffer[LCD_WIDTH * BYTES_PER_PIXEL];
    uint8_t *p = line_buffer;

    for (int t = 0; t < line_bytes; t += BYTES_PER_PIXEL)
    {
      *p++ = c & 0xFF;
      *p++ = c >> 8;
#if BYTES_PER_PIXEL == 3
      *p++ = c >> 16;
#endif
    }
    for (int y = y1; y <= y2; y++)
    {
      spi_write_fast(Pico_LCD_SPI_MOD, line_buffer, line_bytes);
    }
  }
  spi_finish(Pico_LCD_SPI_MOD);
  lcd_spi_raise_cs();
}

void draw_battery_icon(int x0, int y0, int level)
{
  draw_rect_spi(x0, y0, x0 + 14, y0 + 6, WHITE);
  draw_rect_spi(x0 + 1, y0 + 1, x0 + 12, y0 + 5, BLACK);

  // (2x2)
  draw_rect_spi(x0 + 14, y0 + 2, x0 + 14 + 2, y0 + 2 + 2, WHITE);

  for (int i = 0; i <= 13; i++)
  {
    if (i < level)
    {
      draw_rect_spi(x0 + 1 + i * 1, y0 + 1, x0 + 1 + i * 1 + 1, y0 + 1 + 4, WHITE);
    }
  }
}

void lcd_print_char(int fc, int bc, char c, int orientation)
{
  int scale = 0x01;

  int index = c - font->start;

  if (index >= 0 && index < font->length)
  {
    uint8_t *glyph = &font->glyphdata[index * (font->height * font->width / 8)];

    draw_bitmap_spi(current_x, current_y, font->width, font->height, scale, fc, bc, glyph);
  }
  else
  {
    if (bc >= 0)
    {
      draw_rect_spi(current_x, current_y, current_x + (font->width * scale) - 1,
                    current_y + (font->height * scale) - 1, bc);
    }
  }

  if (orientation == ORIENT_NORMAL) current_x += font->width * scale;
}

void scroll_lcd_spi(int lines)
{
  if (lines == 0) return;

  unsigned char line_buffer[LCD_WIDTH * BYTES_PER_PIXEL];

  if (lines > 0)
  {
    for (int i = 0; i < LCD_HEIGHT - lines; i++)
    {
      read_buffer_spi(0, i + lines, LCD_WIDTH - 1, i + lines, line_buffer);
      draw_buffer_spi(0, i, LCD_WIDTH - 1, i, line_buffer);
    }
    draw_rect_spi(0, LCD_HEIGHT - lines, LCD_WIDTH - 1, LCD_HEIGHT - 1, gui_bcolour);
  }
  else
  {
    lines = -lines;
    for (int i = LCD_HEIGHT - 1; i >= lines; i--)
    {
      read_buffer_spi(0, i - lines, LCD_WIDTH - 1, i - lines, line_buffer);
      draw_buffer_spi(0, i, LCD_WIDTH - 1, i, line_buffer);
    }
    draw_rect_spi(0, 0, LCD_WIDTH - 1, lines - 1, gui_bcolour);
  }
}

void display_put_c(char c)
{
  // if it is printable and it is going to take us off the right hand end of the
  // screen do a CRLF
  if (c >= font->start && c < font->start + font->length)
  {
    if (current_x + font->width > LCD_WIDTH)
    {
      display_put_c('\r');
      display_put_c('\n');
    }
  }

  // handle the standard control chars
  switch (c)
  {
    case '\b':
      current_x -= font->width;
      if (current_x < 0)
      {                             // Go to end of previous line
        current_y -= font->height;  // Go up one line
        if (current_y < 0) current_y = 0;
        current_x += LCD_WIDTH;
      }
      return;
    case '\r':
      current_x = 0;
      return;
    case '\n':
      current_x = 0;
      current_y += font->height;
      if (current_y + font->height >= LCD_HEIGHT)
      {
        scroll_lcd_spi(current_y + font->height - LCD_HEIGHT);
        current_y -= (current_y + font->height - LCD_HEIGHT);
      }
      return;
    case '\t':
      do
      {
        display_put_c(' ');
      } while ((current_x / font->width) % 2);  // 2 3 4 8
      return;
  }
  lcd_print_char(gui_fcolour, gui_bcolour, c, ORIENT_NORMAL);  // print it
}

char lcd_put_char(char c, int flush)
{
  lcd_putc(0, c);
  if (isprint(c)) lcd_char_pos++;
  if (c == '\r')
  {
    lcd_char_pos = 1;
  }
  return c;
}

void lcd_print_string(char *s)
{
  while (*s)
  {
    lcd_put_char(*s, 1);
    s++;
  }
}

void lcd_print_string_color(char *s, int fg, int bg)
{
  int old_fg = gui_fcolour;
  int old_bg = gui_bcolour;

  gui_fcolour = fg;
  gui_bcolour = bg;

  while (*s)
  {
    lcd_put_char(*s, 0);
    s++;
  }

  gui_fcolour = old_fg;
  gui_bcolour = old_bg;
}

void lcd_clear() { draw_rect_spi(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, BLACK); }

void lcd_putc(uint8_t devn, uint8_t c) { display_put_c(c); }

void reset_controller(void)
{
  gpio_put(Pico_LCD_RST, 0);
  sleep_us(20);
  gpio_put(Pico_LCD_RST, 1);
  sleep_ms(5);
}

void spi_write_data(unsigned char data)
{
  gpio_put(Pico_LCD_DC, 1);
  lcd_spi_lower_cs();
  hw_send_spi(&data, 1);
  lcd_spi_raise_cs();
}

void spi_write_command(unsigned char data)
{
  gpio_put(Pico_LCD_DC, 0);
  gpio_put(Pico_LCD_CS, 0);

  spi_write_blocking(Pico_LCD_SPI_MOD, &data, 1);

  gpio_put(Pico_LCD_CS, 1);
}

void spi_write_cd(unsigned char command, int data, ...)
{
  int i;
  va_list ap;
  va_start(ap, data);
  spi_write_command(command);
  for (i = 0; i < data; i++) spi_write_data((char)va_arg(ap, int));
  va_end(ap);
}

void lcd_cmd_init()
{
  reset_controller();

  spi_write_command(TFT_PGC);
  spi_write_data(0x00);
  spi_write_data(0x03);
  spi_write_data(0x09);
  spi_write_data(0x08);
  spi_write_data(0x16);
  spi_write_data(0x0A);
  spi_write_data(0x3F);
  spi_write_data(0x78);
  spi_write_data(0x4C);
  spi_write_data(0x09);
  spi_write_data(0x0A);
  spi_write_data(0x08);
  spi_write_data(0x16);
  spi_write_data(0x1A);
  spi_write_data(0x0F);

  spi_write_command(TFT_NGC);
  spi_write_data(0x00);
  spi_write_data(0x16);
  spi_write_data(0x19);
  spi_write_data(0x03);
  spi_write_data(0x0F);
  spi_write_data(0x05);
  spi_write_data(0x32);
  spi_write_data(0x45);
  spi_write_data(0x46);
  spi_write_data(0x04);
  spi_write_data(0x0E);
  spi_write_data(0x0D);
  spi_write_data(0x35);
  spi_write_data(0x37);
  spi_write_data(0x0F);

  spi_write_command(TFT_PWR1);
  spi_write_data(0x17);
  spi_write_data(0x15);

  spi_write_command(TFT_PWR2);
  spi_write_data(0x41);

  spi_write_command(TFT_VCMPCTL);
  spi_write_data(0x00);
  spi_write_data(0x12);
  spi_write_data(0x80);

  // spi_write_command(TFT_MADCTL);
  // spi_write_data(0x48);

  spi_write_command(TFT_COLMOD);
#if BYTES_PER_PIXEL == 2
  spi_write_data(0x55);  // RGB565
#elif BYTES_PER_PIXEL == 3
  spi_write_data(0x77);  // RGB888
#endif

  spi_write_command(TFT_IFMODE);
  spi_write_data(0x00);

  spi_write_command(TFT_FRMCTR1);
  spi_write_data(0xA0);
  spi_write_data(0x10);

  spi_write_command(TFT_INVON);

  spi_write_command(TFT_DIC);
  spi_write_data(0x02);

  spi_write_command(TFT_DFC);
  spi_write_data(0x02);
  spi_write_data(0x02);
  spi_write_data(0x28);

  spi_write_command(TFT_EM);
  spi_write_data(0xC6);
  spi_write_command(TFT_SETIMGFUNC);
  spi_write_data(0x00);

  spi_write_command(TFT_ADJCON3);  // Adjust Control 3
  spi_write_data(0xA9);
  spi_write_data(0x51);
  spi_write_data(0x2C);
  spi_write_data(0x82);

  spi_write_command(TFT_SLPOUT);
  sleep_ms(120);

  lcd_clear();

  spi_write_command(TFT_DISPON);
  sleep_ms(120);

  spi_write_cd(TFT_MADCTL, 1, ILI9341_Portrait);
}

void lcd_set_cursor(int x, int y)
{
  current_x = x;
  current_y = y;
}

void lcd_spi_init()
{
  // init GPIO
  gpio_init(Pico_LCD_SCK);
  gpio_init(Pico_LCD_TX);
  gpio_init(Pico_LCD_RX);
  gpio_init(Pico_LCD_CS);
  gpio_init(Pico_LCD_DC);
  gpio_init(Pico_LCD_RST);

  gpio_set_dir(Pico_LCD_SCK, GPIO_OUT);
  gpio_set_dir(Pico_LCD_TX, GPIO_OUT);
  gpio_set_dir(Pico_LCD_CS, GPIO_OUT);
  gpio_set_dir(Pico_LCD_DC, GPIO_OUT);
  gpio_set_dir(Pico_LCD_RST, GPIO_OUT);

  // init SPI
  spi_init(Pico_LCD_SPI_MOD, LCD_SPI_SPEED);
  gpio_set_function(Pico_LCD_SCK, GPIO_FUNC_SPI);
  gpio_set_function(Pico_LCD_TX, GPIO_FUNC_SPI);
  gpio_set_function(Pico_LCD_RX, GPIO_FUNC_SPI);
  gpio_set_input_hysteresis_enabled(Pico_LCD_RX, true);

  gpio_put(Pico_LCD_CS, 1);
  gpio_put(Pico_LCD_RST, 1);
}

void lcd_init()
{
  lcd_spi_init();
  lcd_cmd_init();

  set_font(font1);

  gui_fcolour = WHITE;
  gui_bcolour = BLACK;
}
