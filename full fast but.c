#include "stm32f10x.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

/* --------------------------- Константы --------------------------- */
#define SYSCLK_FREQ     72000000UL

#define BUTTON_PIN      10      // PA10
#define RSSI_CHAN       0       // ADC1_IN0 = PA0
#define BURST_PIN       4       // PA4 (LM1881 BURST)
#define LED_PIN         12      // PA12 (индикационный)
#define BLINK_PIN       2       // PB2 (моргаем при смене частоты)

#define SPILE_PIN       12      // PB12  CS/LE
#define SPICLK_PIN      13      // PB13  CLK
#define SPIDATA_PIN     15      // PB15  DATA

#define FREQ_MIN        4850
#define FREQ_MAX        6200
#define FREQ_STEP       5
#define FREQ_CNT        (((FREQ_MAX - FREQ_MIN) / FREQ_STEP) + 1)

#define RSSI_X          10
#define RSSI_Y          45
#define RSSI_W          98
#define RSSI_H          10
#define RSSI_MINVAL     91

#define ABS(x)          ((x) < 0 ? -(x) : (x))

/* --------------------------- Глобальные -------------------------- */
static volatile uint32_t msTicks;
static int last_freq = 5000;
static int freq      = 5000;

static uint16_t freq_list[FREQ_CNT];
static uint32_t pll_cmd_list[FREQ_CNT];

/* Обновление дисплея не чаще чем раз в N шагов (ускорение) */
static uint8_t disp_divider = 0;
#define DISP_EVERY_N_STEPS   8   // менять при необходимости

/* --------------------------- Прототипы --------------------------- */
void clock_config(void);
void SysTick_Handler(void);
void initSysTick(void);
void tim2_init(void);

static inline void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void gpio_init(void);
void adc_init(void);
uint16_t adc_read(void);

void i2c1_init(void);
void spi_gpio_init(void);

uint32_t pulse_width_fast(void);

int32_t RSSI_dbmcalc(int raw);
int      calc_freq_cmd(int f);
void     precompute_freqs(void);

static inline void sendSPICommand_fast(uint32_t cmd25);
static inline void set_freq_idx(uint16_t idx);
static inline void blink_once(void);

void SFrq_draw(int f, int rssi);

/* --------------------------- Тактирование ------------------------ */
void clock_config(void)
{
    RCC->CR |= RCC_CR_HSION;
    RCC->CFGR = 0;
    RCC->CR &= ~((1 << 16) | (1 << 24));
    RCC->CIR = 0;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    RCC->CFGR |= RCC_CFGR_PLLSRC;           // HSE
    RCC->CFGR |= RCC_CFGR_PLLMULL9;         // x9

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = SYSCLK_FREQ;
}

/* --------------------------- SysTick ----------------------------- */
void SysTick_Handler(void) { msTicks++; }

void initSysTick(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

void delay_ms(uint32_t ms)
{
    uint32_t tgt = msTicks + ms;
    while (msTicks < tgt);
}

/* --------------------------- TIM2 + delay_us --------------------- */
void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (SystemCoreClock / 1000000) - 1; // 1 МГц
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static inline void delay_us(uint32_t us)
{
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

/* --------------------------- GPIO -------------------------------- */
void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    /* PA10 - кнопка (pull-up) */
    GPIOA->CRH &= ~(0xF << ((BUTTON_PIN - 8) * 4));
    GPIOA->CRH |=  (0x8 << ((BUTTON_PIN - 8) * 4));
    GPIOA->ODR |=  (1 << BUTTON_PIN);

    /* PA4 - BURST вход (pull-down) */
    GPIOA->CRL &= ~(0xF << (BURST_PIN * 4));
    GPIOA->CRL |=  (0x8 << (BURST_PIN * 4));
    GPIOA->ODR &= ~(1 << BURST_PIN);

    /* PA12 - LED output 2 MHz PP */
    GPIOA->CRH &= ~(0xF << ((LED_PIN - 8) * 4));
    GPIOA->CRH |=  (0x2 << ((LED_PIN - 8) * 4));

    /* PB2 - BLINK output 2 MHz PP */
    GPIOB->CRL &= ~(0xF << (BLINK_PIN * 4));
    GPIOB->CRL |=  (0x2 << (BLINK_PIN * 4));
    GPIOB->BRR  =  (1 << BLINK_PIN);
}

/* --------------------------- ADC --------------------------------- */
void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Sample time минимальный (13.5 циклов) — можно ещё меньше при необходимости точности */
    ADC1->SMPR2 = 0; // канал 0 = 1.5 циклов, ок для скорости

    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = RSSI_CHAN;

    ADC1->CR2 = ADC_CR2_ADON;
    delay_us(10);
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL);
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
}

uint16_t adc_read(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;         // старт
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)(ADC1->DR & 0xFFFF);
}

/* --------------------------- I2C OLED ---------------------------- */
void i2c1_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // PB6=SCL, PB7=SDA AF Open-Drain
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB->CRL |=  ((0xB << 24) | (0xB << 28));

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    uint32_t pclk1 = SystemCoreClock / 2; // 36 МГц

    I2C1->CR2  = (pclk1 / 1000000);
    uint16_t ccr = pclk1 / (3 * 400000);
    I2C1->CCR  = (ccr & 0x0FFF) | I2C_CCR_FS;
    I2C1->TRISE = (pclk1 / 1000000) * 300 / 1000 + 1;

    I2C1->CR1 |= I2C_CR1_PE;
}

/* --------------------------- SPI BitBang ------------------------- */
void spi_gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // PB12, PB13, PB15 - PP outputs
    GPIOB->CRH &= ~((0xF << 16) | (0xF << 20) | (0xF << 28));
    GPIOB->CRH |=  ((0x2 << 16) | (0x2 << 20) | (0x2 << 28));

    GPIOB->BSRR = (1 << SPILE_PIN); // CS high
}

/* 25-битовая команда уже упакована */
static inline void sendSPICommand_fast(uint32_t cmd25)
{
    GPIOB->BRR  = (1 << SPILE_PIN);   // CS LOW
     delay_us(1);

    for (int8_t i = 24; i >= 0; --i)  // MSB first
    {
        if (cmd25 & (1u << i))
            GPIOB->BSRR = (1 << SPIDATA_PIN);
        else
            GPIOB->BRR  = (1 << SPIDATA_PIN);

        GPIOB->BSRR = (1 << SPICLK_PIN); // CLK ↑
         delay_us(1);                // маленькая задержка
        GPIOB->BRR  = (1 << SPICLK_PIN); // CLK ↓
    }

    GPIOB->BSRR = (1 << SPILE_PIN);   // CS HIGH
     delay_us(1);
}

/* --------------------------- BURST измерение --------------------- */
uint32_t pulse_width_fast(void)
{
    /* Ожидаем окончание предыдущего импульса, но не более 300 мкс */
    uint32_t start = TIM2->CNT;
    while ((GPIOA->IDR & (1 << BURST_PIN)) && ((TIM2->CNT - start) < 1000));

    /* Ожидаем фронт, не более 300 мкс */
    start = TIM2->CNT;
    while (!(GPIOA->IDR & (1 << BURST_PIN)) && ((TIM2->CNT - start) < 1000));
    if (!(GPIOA->IDR & (1 << BURST_PIN))) return 0;

    /* Измеряем длительность */
    uint32_t t0 = TIM2->CNT;
    while ((GPIOA->IDR & (1 << BURST_PIN)) && ((TIM2->CNT - t0) < 1000));
    return (TIM2->CNT - t0);
}

/* --------------------------- Utils ------------------------------- */
int32_t RSSI_dbmcalc(int raw_adc)
{
    // Ваше приближённое преобразование оставлено
    return -80 + (raw_adc - 750) / 11;
}

int calc_freq_cmd(int f)
{
    int N = ((f - 479) / 2) / 32;
    int A = ((f - 479) / 2) % 32;
    return (N << 7) | (A & 0x1FFF);
}

void precompute_freqs(void)
{
    for (int i = 0; i < FREQ_CNT; i++)
    {
        int fr = FREQ_MIN + i * FREQ_STEP;
        freq_list[i] = fr;
        uint32_t data = (uint32_t)calc_freq_cmd(fr);
        uint32_t cmd  = (0x01 & 0x0F) | (1u << 4) | (data << 5);
        pll_cmd_list[i] = cmd;
    }
}

static inline void set_freq_idx(uint16_t idx)
{
    sendSPICommand_fast(pll_cmd_list[idx]);
}

static inline void blink_once(void)
{
    GPIOB->BSRR = (1 << BLINK_PIN);
    // ~5-8 мкс импульс
    delay_us(500);
    GPIOB->BRR  = (1 << BLINK_PIN);
}

/* --------------------------- OLED draw --------------------------- */
void SFrq_draw(int f, int rssi)
{
    char buf[32];
    ssd1306_Fill(Black);

    sprintf(buf, "%d MHz", f);
    ssd1306_SetCursor(0, 5);
    ssd1306_WriteString(buf, Font_7x10, White);

    sprintf(buf, "RSSI = %d dBm", rssi);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_DrawRect(RSSI_X, RSSI_Y, RSSI_W, RSSI_H, White);
    int w = rssi + RSSI_MINVAL;
    if (w < 0) w = 0;
    if (w > RSSI_W) w = RSSI_W;
    ssd1306_FillRect(RSSI_X + 1, RSSI_Y + 1, w, RSSI_H - 2, White);

    ssd1306_UpdateScreen();
}

/* --------------------------- main ------------------------------- */
int main(void)
{
    SystemInit();
    clock_config();
    initSysTick();
    tim2_init();
    gpio_init();
    adc_init();
    i2c1_init();
    ssd1306_Init();
    spi_gpio_init();
    precompute_freqs();

    // Индикация запуска
    GPIOA->BSRR = (1 << LED_PIN); delay_ms(50);
    GPIOA->BRR  = (1 << LED_PIN); delay_ms(50);
    GPIOA->BSRR = (1 << LED_PIN); delay_ms(50);
    GPIOA->BRR  = (1 << LED_PIN); delay_ms(50);

    int prev_rssi = -999;

    /* Стартуем с 5110 */
    int start_idx = (5110 - FREQ_MIN) / FREQ_STEP;
    if (start_idx < 0) start_idx = 0;
    if (start_idx >= FREQ_CNT) start_idx = 0;

    uint16_t idx = start_idx;
    set_freq_idx(idx);
    GPIOB->BRR = (1 << BLINK_PIN);

    while (1)
    {
        /* Следующая частота */
        idx++;
        if (idx >= FREQ_CNT) idx = 0;
        freq = freq_list[idx];

        set_freq_idx(idx);
        blink_once();                // мигаем на смену частоты

        int rssi = RSSI_dbmcalc(adc_read());

        /* Обновляем дисплей не на каждом шаге, но сохраняем функционал */
        if (disp_divider++ >= DISP_EVERY_N_STEPS || ABS(rssi - prev_rssi) > 2)
        {
            SFrq_draw(freq + 10, rssi);   // как было у вас
            prev_rssi = rssi;
            disp_divider = 0;
        }

        /* Детекция видеосигнала (10 BURST измерений) */
        int check_fpv = 0;
        for (int i = 0; i < 10; i++)
        {
            uint32_t dur = pulse_width_fast();
            if (dur > 59 && dur < 62) check_fpv++;
        }

        if (check_fpv > 4)
        {
            /* Нашли сигнал — запоминаем частоту, ждём отпускания кнопки */
            while (GPIOA->IDR & (1 << BUTTON_PIN))
            {
                last_freq = freq;
                int rr = RSSI_dbmcalc(adc_read());
                SFrq_draw(freq + 10, rr);
            }
            delay_ms(500); // дебаунс
        }

        /* Кнопка сброса — вернуться назад */
        if (!(GPIOA->IDR & (1 << BUTTON_PIN)))
        {
            freq = last_freq - 10;
            if (freq < FREQ_MIN) freq = FREQ_MIN;
            idx = (freq - FREQ_MIN) / FREQ_STEP;
            set_freq_idx(idx);
            blink_once();
            delay_ms(500);
        }
    }
}
