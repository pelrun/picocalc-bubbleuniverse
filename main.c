#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>
#include <pico/time.h>

#include "lcdspi.h"
#include "i2ckbd.h"

#define CURVECOUNT 256
#define CURVESTEP 4
#define ITERATIONS 256
#define SCREENWIDTH 320
#define SCREENHEIGHT 320

#define SINTABLEPOWER 14
#define SINTABLEENTRIES (1 << SINTABLEPOWER)
#define ANG1INC (int32_t)((CURVESTEP * SINTABLEENTRIES) / 235)
#define ANG2INC (int32_t)((CURVESTEP * SINTABLEENTRIES) / (2 * M_PI))
#define SCALEMUL (int32_t)(SCREENHEIGHT * M_PI_2)

#define SCALESPEED 1.04
#define MOVESPEED 2.0
#define ANIMSPEEDCHANGE 0.02

float speed, oldSpeed;
float size;
int32_t xOffset, yOffset;
int32_t animationTime;

int32_t SinTable[SINTABLEENTRIES];

uint16_t palette[128];
uint16_t buffer[SCREENWIDTH * SCREENHEIGHT];

void resetValues(void)
{
  speed    = 0.2f;
  oldSpeed = 0;
  size     = 1;
  xOffset  = 0;
  yOffset  = 0;
}

void CreateSinTable()
{
  for (int i = 0; i < SINTABLEENTRIES; ++i)
  {
    SinTable[i] = sin(i * 2 * M_PI / SINTABLEENTRIES) * SINTABLEENTRIES / (2 * M_PI);
  }
  for (int i = 0; i < SINTABLEENTRIES; ++i)
  {
    *((int16_t *)(SinTable + i) + 1) =
        *(int16_t *)(SinTable + ((i + SINTABLEENTRIES / 4) % SINTABLEENTRIES));
  }
}

void InitPalette()
{
  for (int i = 0; i < 8; ++i)
  {
    const uint32_t red = (255 * i) / 7;
    for (int j = 0; j < 16; ++j)
    {
      int index            = i * 16 + j;
      const uint32_t green = (255 * j) / 15;
      const uint32_t blue  = (510 - (red + green)) >> 1;
      uint32_t val         = RGB(red, green, blue);
      palette[index]       = val;
    }
  }
  palette[0] = 0;
}

void Render(uint16_t *ptr)
{
  uint16_t *screenCentre = ptr + (((SCREENWIDTH | 1) * SCREENHEIGHT) >> 1);

  int32_t ang1Start = animationTime;
  int32_t ang2Start = animationTime;
  for (int i = 0; i < CURVECOUNT; i += CURVESTEP)
  {
    int32_t x = 0;
    int32_t y = 0;
    for (int j = 0; j < ITERATIONS; ++j)
    {
      int32_t values1 = SinTable[(ang1Start + x) & (SINTABLEENTRIES - 1)];
      int32_t values2 = SinTable[(ang2Start + y) & (SINTABLEENTRIES - 1)];
      x               = (int32_t)(int16_t)values1 + (int32_t)(int16_t)values2;
      y               = (values1 >> 16) + (values2 >> 16);
      int32_t pX      = ((int)(x * SCALEMUL * size) >> SINTABLEPOWER) + xOffset;
      int32_t pY      = ((int)(y * SCALEMUL * size) >> SINTABLEPOWER) + yOffset;
      if (abs(pX) < SCREENWIDTH / 2 && abs(pY) < SCREENHEIGHT / 2)
      {
        screenCentre[pY * SCREENWIDTH + pX] = palette[((i >> 1) & 0x70) + (j >> 4)];
      }
    }
    ang1Start += ANG1INC;
    ang2Start += ANG2INC;
  }
}

void pollKeyboard()
{
  int key = read_i2c_kbd();

  if (key == KEY_HOME || key == KEY_ESC)
  {
    resetValues();
  }
  if (key == '-') speed -= ANIMSPEEDCHANGE / size;
  if (key == '=') speed += ANIMSPEEDCHANGE / size;
  if (key == KEY_ENTER)
  {
    size *= SCALESPEED;
    xOffset *= SCALESPEED;
    yOffset *= SCALESPEED;
  }
  if (key == KEY_BACKSPACE)
  {
    size /= SCALESPEED;
    xOffset /= SCALESPEED;
    yOffset /= SCALESPEED;
  }
  if (key == ' ')
  {
    float temp = speed;
    speed      = oldSpeed;
    oldSpeed   = temp;
  }
  if (key == KEY_LEFT) xOffset += MOVESPEED;
  if (key == KEY_RIGHT) xOffset -= MOVESPEED;
  if (key == KEY_UP) yOffset += MOVESPEED;
  if (key == KEY_DOWN) yOffset -= MOVESPEED;
}

static inline uint32_t millis(void) { return to_ms_since_boot(get_absolute_time()); }

uint64_t oldTime;
char fps[20];

void loop()
{
  uint64_t time      = millis();
  uint32_t deltaTime = time - oldTime;
  animationTime += deltaTime * speed;
  oldTime = time;

  memset(buffer, 0, sizeof(buffer));
  Render(buffer);
  draw_buffer_spi(0, 0, SCREENWIDTH - 1, SCREENHEIGHT - 1, buffer);

  pollKeyboard();
#if 0
  sprintf(fps, "%d", 1000/deltaTime);
  lcd_set_cursor(0,0);
  lcd_print_string_color(fps, WHITE, -1);
#endif
}

int main()
{
  init_i2c_kbd();
  lcd_init();
  CreateSinTable();
  InitPalette();
  resetValues();
  oldTime = millis();
  while (1) loop();
}
