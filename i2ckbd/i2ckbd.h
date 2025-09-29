#ifndef I2C_KEYBOARD_H
#define I2C_KEYBOARD_H

#define I2C_KBD_MOD i2c1
#define I2C_KBD_SDA 6
#define I2C_KBD_SCL 7

#define I2C_KBD_SPEED 10000  // if dual i2c, then the speed of keyboard i2c should be 10khz

#define I2C_KBD_ADDR 0x1F

enum keycode_e
{
  KEY_JOY_UP     = 0x01,
  KEY_JOY_DOWN   = 0x02,
  KEY_JOY_LEFT   = 0x03,
  KEY_JOY_RIGHT  = 0x04,
  KEY_JOY_CENTER = 0x05,
  KEY_BTN_LEFT1  = 0x06,
  KEY_BTN_RIGHT1 = 0x07,
  KEY_BACKSPACE  = 0x08,
  KEY_TAB        = 0x09,
  KEY_ENTER      = 0x0A,
  KEY_BTN_LEFT2  = 0x11,
  KEY_BTN_RIGHT2 = 0x12,
  KEY_MOD_ALT    = 0xA1,
  KEY_MOD_SHL    = 0xA2,
  KEY_MOD_SHR    = 0xA3,
  KEY_MOD_SYM    = 0xA4,
  KEY_MOD_CTRL   = 0xA5,
  KEY_ESC        = 0xB1,
  KEY_UP         = 0xb5,
  KEY_DOWN       = 0xb6,
  KEY_LEFT       = 0xb4,
  KEY_RIGHT      = 0xb7,
  KEY_BREAK      = 0xd0,
  KEY_INSERT     = 0xD1,
  KEY_HOME       = 0xD2,
  KEY_DEL        = 0xD4,
  KEY_END        = 0xD5,
  KEY_PAGE_UP    = 0xd6,
  KEY_PAGE_DOWN  = 0xd7,
  KEY_CAPS_LOCK  = 0xC1,
  KEY_F1         = 0x81,
  KEY_F2         = 0x82,
  KEY_F3         = 0x83,
  KEY_F4         = 0x84,
  KEY_F5         = 0x85,
  KEY_F6         = 0x86,
  KEY_F7         = 0x87,
  KEY_F8         = 0x88,
  KEY_F9         = 0x89,
  KEY_F10        = 0x90,
  KEY_POWER      = 0x91,
};

void init_i2c_kbd();
int read_i2c_kbd();
int read_battery();

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitClear(value, bit) ((value) &= ~(1 << (bit)))
#define bitSet(value, bit) ((value) |= (1 << (bit)))

#endif
