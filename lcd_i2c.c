#include "stm32f10x.h"
#include "lcd_i2c.h"

#define LCD_ADDR (0x27 << 1)  // I2C адрес дисплея (<<1 для STM32)
#define I2C_DELAY 2000

// Управляющие биты PCF8574
#define LCD_BACKLIGHT 0x08
#define ENABLE        0x04
#define RW            0x02
#define RS            0x01

static void i2c_delay(volatile uint32_t d) {
    while (d--) __NOP();
}

static void i2c_start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
}

static void i2c_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

static void i2c_write_addr(uint8_t addr) {
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static void i2c_write_byte(uint8_t data) {
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
}

// Отправка одного байта с EN импульсом
static void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble & 0xF0) | LCD_BACKLIGHT | (rs ? RS : 0);
    i2c_start();
    i2c_write_addr(LCD_ADDR);
    i2c_write_byte(data | ENABLE);
    i2c_delay(I2C_DELAY);
    i2c_write_byte(data & ~ENABLE);
    i2c_stop();
}

static void lcd_send_byte(uint8_t byte, uint8_t rs) {
    lcd_send_nibble(byte & 0xF0, rs);
    lcd_send_nibble((byte << 4) & 0xF0, rs);
}

static void lcd_send_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
    delay_ms(2);
}

static void lcd_send_data(uint8_t data) {
    lcd_send_byte(data, 1);
    delay_ms(2);
}

void lcd_i2c_init(void) {
    // === Настройка I2C1 ===
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6 = SCL, PB7 = SDA (AF open-drain, 2МГц)
    GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->CRL |=  (0xB << (6 * 4)) | (0xB << (7 * 4));

    I2C1->CR2 = 36;             // APB1 = 36 МГц
    I2C1->CCR = 180;            // 100 кГц
    I2C1->TRISE = 37;           // 1000ns / (1/36МГц) + 1
    I2C1->CR1 = I2C_CR1_PE;

    delay_ms(40); // Подождём старта дисплея

    lcd_send_cmd(0x33); // Инициализация
    lcd_send_cmd(0x32); // 4-битный режим
    lcd_send_cmd(0x28); // 2 строки, 5x8
    lcd_send_cmd(0x0C); // Вкл. дисплей, курсор выкл.
    lcd_send_cmd(0x06); // Автоинкремент
    lcd_send_cmd(0x01); // Очистить
    delay_ms(5);
}

void lcd_i2c_clear(void) {
    lcd_send_cmd(0x01);
    delay_ms(2);
}

void lcd_i2c_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? col : (0x40 + col);
    lcd_send_cmd(0x80 | addr);
}

void lcd_i2c_print(const char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str++));
    }
}

void lcd_i2c_putc(char c) {
    lcd_send_data((uint8_t)c);
}

// Простой миллисекундный delay — если нужно можно заменить на SysTick
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 7200; ++i)
        __NOP();
}
