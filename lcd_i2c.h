#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f10x.h"

void lcd_i2c_init(void);
void lcd_i2c_clear(void);
void lcd_i2c_set_cursor(uint8_t col, uint8_t row);
void lcd_i2c_print(const char *str);
void lcd_i2c_putc(char c);
void delay_ms(uint32_t ms);
#endif
