#include "stm32f10x.h"

#define LCD_ADDR 0x4E
#define LCD_BL   0x08
#define LCD_E    0x04
#define LCD_RS   0x01

void delay(volatile uint32_t t) { while (t--) __NOP(); }

void i2c_init(void) {
    RCC->APB2ENR |= (1 << 3); // GPIOB
    RCC->APB1ENR |= (1 << 21); // I2C1
    GPIOB->CRL &= ~(0xFF << 24);
    GPIOB->CRL |=  (0xBB << 24); // PB6, PB7 AF open-drain
    I2C1->CR2 = 36;
    I2C1->CCR = 180;
    I2C1->TRISE = 37;
    I2C1->CR1 = I2C_CR1_PE;
}

void i2c_start(uint8_t addr) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}

void i2c_write(uint8_t data) {
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
}

void i2c_stop(void) {
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void lcd_nibble(uint8_t nib) {
    i2c_start(LCD_ADDR);
    i2c_write(nib | LCD_BL | LCD_E);
    i2c_write(nib | LCD_BL);
    i2c_stop();
    delay(10000);
}

void lcd_byte(uint8_t val, uint8_t mode) {
    uint8_t hi = val & 0xF0;
    uint8_t lo = (val << 4) & 0xF0;
    lcd_nibble(hi | mode);
    lcd_nibble(lo | mode);
}

void lcd_cmd(uint8_t cmd) {
    lcd_byte(cmd, 0);
}

void lcd_data(char c) {
    lcd_byte(c, LCD_RS);
}

void lcd_init(void) {
    delay(50000);
    lcd_cmd(0x33);
    lcd_cmd(0x32);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    delay(50000);
}

void lcd_str(const char* s) {
    while (*s) lcd_data(*s++);
}

int main(void) {
    i2c_init();
    lcd_init();

    lcd_cmd(0x80); // cursor line 1 pos 0
    lcd_str("HELLO STM32");

    lcd_cmd(0xC0); // line 2
    lcd_str("I2C OK @0x4E");

    while (1);
}
