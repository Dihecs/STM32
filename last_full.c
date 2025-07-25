#include "stm32f10x.h"  // CMSIS-заголовок для STM32F103
#include "ssd1306.h"      // Заголовок для OLED SSD1306
#include "ssd1306_fonts.h"// Шрифты для дисплея SSD1306
#include <stdio.h>         // Для sprintf()
#include <stdint.h> 

/* -------------------- Константы -------------------- */
#define SYSCLK_FREQ     72000000UL  // Частота шины 72 МГц
#define BUTTON_PIN      10          // PA10 — кнопка сброса
#define RSSI_CHAN       0           // ADC1_IN0 = PA0 — канал RSSI
#define BURST_PIN       4           // PA4 — вход BURST от LM1881
#define LED_PIN         12          // PA12 — светодиод старта
#define BLINK_PIN       2           // PB2 — индикатор смены частоты

#define SPILE_PIN       12          // PB12 — CS/LE для RX5808
#define SPICLK_PIN      13          // PB13 — CLK для SPI-битбэнга
#define SPIDATA_PIN     15          // PB15 — DATA для SPI-битбэнга

#define FREQ_MIN        4850        // Минимальная частота, MHz
#define FREQ_MAX        6200        // Максимальная частота, MHz
#define FREQ_STEP       5           // Шаг перестройки, MHz
#define FREQ_CNT        (((FREQ_MAX - FREQ_MIN) / FREQ_STEP) + 1) // Кол-во шагов

#define RSSI_X          10          // X для полоски RSSI
#define RSSI_Y          45          // Y для полоски RSSI
#define RSSI_W          98          // Ширина полоски RSSI
#define RSSI_H          10          // Высота полоски RSSI
#define RSSI_MINVAL     91          // Смещение RSSI для полоски

#define ABS(x)          ((x) < 0 ? -(x) : (x))  // Абсолютное значение

/* -------------------- Глобальные переменные -------------------- */
static volatile uint32_t msTicks;           // Счетчик миллисекунд
static int last_freq = 5000;                // Последняя сохраненная частота
static int freq      = 5000;                // Текущая частота для сканирования

static uint16_t freq_list[FREQ_CNT];       // Список частот для предвычисления
static uint8_t disp_divider = 0;            // Делитель для обновления экрана
#define DISP_EVERY_N_STEPS 8               // Обновлять экран раз в N шагов

/* -------------------- Прототипы функций -------------------- */
void clock_config(void);                    // Настройка тактирования
void SysTick_Handler(void);                 // Обработчик SysTick
void initSysTick(void);                     // Инициализация SysTick
void tim2_init(void);                       // Инициализация TIM2 для delay_us
static inline void delay_us(uint32_t us);   // Задержка в микросекундах
void delay_ms(uint32_t ms);                 // Задержка в миллисекундах
void gpio_init(void);                       // Настройка GPIO
void adc_init(void);                        // Настройка ADC1
uint16_t adc_read(void);                    // Чтение ADC
void i2c1_init(void);                       // Настройка I2C1 для дисплея
void spi_gpio_init(void);                   // Настройка GPIO для SPI битбэнга
uint32_t pulse_width_fast(void);            // Измерение ширины BURST
int32_t RSSI_dbmcalc(int raw_adc);          // Преобразование ADC->dBm
int calc_freq_cmd(int f);                   // Вычисление PLL команды
void precompute_freqs(void);                // Предварительный расчет команд
static inline void sendSPICommand_rx5808(uint8_t addr, uint8_t rw, uint32_t data); // Отправка SPI
static inline void set_freq_idx(uint16_t idx);   // Установка частоты по индексу
static inline void blink_once(void);              // Моргаем PB2 один раз
void SFrq_draw(int f, int rssi);            // Отрисовка экрана

/* -------------------- clock_config -------------------- */
void clock_config(void)
{
    RCC->CR |= RCC_CR_HSION;               // Включить внутренний RC-генератор HSI
    RCC->CFGR = 0;                         // Сброс конфигурации шин
    RCC->CR &= ~((1 << 16) | (1 << 24));   // Выключить HSE и PLL
    RCC->CIR = 0;                          // Отключить прерывания тактирования

    RCC->CR |= RCC_CR_HSEON;               // Включить внешний кварц HSE
    while (!(RCC->CR & RCC_CR_HSERDY));    // Ждать готовности HSE

    FLASH->ACR |= FLASH_ACR_PRFTBE;        // Включить Prefetch-buffer
    FLASH->ACR &= ~FLASH_ACR_LATENCY;      // Сброс задержки
    FLASH->ACR |= FLASH_ACR_LATENCY_2;     // Установить 2 цикла задержки

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;       // AHB = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;      // APB1 = AHB/2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;      // APB2 = AHB
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;     // ADC = PCLK2/6

    RCC->CFGR |= RCC_CFGR_PLLSRC;          // Источник PLL = HSE
    RCC->CFGR |= RCC_CFGR_PLLMULL9;        // Множитель PLL x9

    RCC->CR |= RCC_CR_PLLON;               // Включить PLL
    while (!(RCC->CR & RCC_CR_PLLRDY));    // Ждать готовности PLL

    RCC->CFGR &= ~RCC_CFGR_SW;             // Сброс SW bits
    RCC->CFGR |= RCC_CFGR_SW_PLL;          // Выбор PLL как SYSCLK
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){} // Ждать переключения

    SystemCoreClock = SYSCLK_FREQ;         // Обновить переменную CMSIS
}

/* -------------------- SysTick -------------------- */
void SysTick_Handler(void) { msTicks++; }   // Каждую мс инкремент msTicks

void initSysTick(void)
{
    SysTick_Config(SystemCoreClock / 1000); // Сброс и настройка на 1 кГц
}

void delay_ms(uint32_t ms)
{
    uint32_t tgt = msTicks + ms;           // Целевая метка времени
    while (msTicks < tgt);                 // Ждать обновления SysTick
}

/* -------------------- TIM2 + delay_us -------------------- */
void tim2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Включить тактирование TIM2
    TIM2->PSC = (SystemCoreClock / 1000000) - 1; // Предделитель: 1 такт = 1 us
    TIM2->ARR = 0xFFFFFFFF;                // Максимальный автозагруз
    TIM2->CNT = 0;                         // Сброс счётчика
    TIM2->CR1 |= TIM_CR1_CEN;              // Запустить таймер
}

static inline void delay_us(uint32_t us)
{
    uint32_t start = TIM2->CNT;            // Запомнить текущее значение
    while ((TIM2->CNT - start) < us);      // Ждать пока разница < us
}

/* -------------------- GPIO -------------------- */
void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN; // Тактирование портов A и B

    GPIOA->CRH &= ~(0xF << ((BUTTON_PIN-8)*4)); // Очищаем CNF/MODE PA10
    GPIOA->CRH |=  (0x8 << ((BUTTON_PIN-8)*4)); // PA10 = pull-up input
    GPIOA->ODR |=  (1 << BUTTON_PIN);           // Включить подтяжку вверх

    GPIOA->CRL &= ~(0xF << (BURST_PIN*4));      // Очищаем CNF/MODE PA4
    GPIOA->CRL |=  (0x8 << (BURST_PIN*4));      // PA4 = pull-down input
    GPIOA->ODR &= ~(1 << BURST_PIN);            // Подтяжка вниз

    GPIOA->CRH &= ~(0xF << ((LED_PIN-8)*4));   // Очищаем PA12
    GPIOA->CRH |=  (0x2 << ((LED_PIN-8)*4));   // PA12 = push-pull output 2MHz

    GPIOB->CRL &= ~(0xF << (BLINK_PIN*4));     // Очищаем PB2
    GPIOB->CRL |=  (0x2 << (BLINK_PIN*4));     // PB2 = push-pull output 2MHz
    GPIOB->BRR  =  (1 << BLINK_PIN);           // Сброс PB2 (светодиод выключен)
}

/* -------------------- ADC -------------------- */
void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;       // Включить ADC1
    ADC1->SMPR2 = 0;                          // Sample time = 1.5 cycles для канала 0
    ADC1->SQR3 = RSSI_CHAN;                   // Первый канал преобразования

    ADC1->CR2 = ADC_CR2_ADON;                 // Включить АЦП
    delay_us(10);                             // Паузка после включения
    ADC1->CR2 |= ADC_CR2_RSTCAL;              // Сброс калибровки
    while (ADC1->CR2 & ADC_CR2_RSTCAL) {}     // Ждать сброса
    ADC1->CR2 |= ADC_CR2_CAL;                 // Запустить калибровку
    while (ADC1->CR2 & ADC_CR2_CAL) {}        // Ждать окончания
}

uint16_t adc_read(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;                // Старт преобразования
    while (!(ADC1->SR & ADC_SR_EOC)) {}       // Ждать окончания
    return (uint16_t)(ADC1->DR & 0xFFFF);     // Чтение результата
}

/* -------------------- I2C OLED -------------------- */
void i2c1_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;       // Тактирование I2C1
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;       // Тактирование порт B

    GPIOB->CRL &= ~((0xF<<24)|(0xF<<28));     // Очищаем PB6/PB7
    GPIOB->CRL |=  ((0xB<<24)|(0xB<<28));     // PB6/SCL PB7/SDA AF-OD

    I2C1->CR1 = I2C_CR1_SWRST;                // Сброс I2C
    I2C1->CR1 = 0;
    uint32_t pclk1 = SystemCoreClock/2;      // APB1=36MHz
    I2C1->CR2 = (pclk1/1000000);
    uint16_t ccr = pclk1/(3*400000);
    I2C1->CCR = (ccr&0x0FFF)|I2C_CCR_FS;
    I2C1->TRISE = (pclk1/1000000)*300/1000 +1;
    I2C1->CR1 |= I2C_CR1_PE;                 // Включить I2C
}

/* -------------------- SPI BitBang для RX5808 -------------------- */
void spi_gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;       // Тактирование порт B
    GPIOB->CRH &= ~((0xF<<16)|(0xF<<20)|(0xF<<28)); // Очищаем PB12,PB13,PB15
    GPIOB->CRH |=  ((0x2<<16)|(0x2<<20)|(0x2<<28)); // PP output 2MHz
    GPIOB->BSRR = (1<<SPILE_PIN);             // CS=High
}

static inline void sendSPICommand_rx5808(uint8_t addr, uint8_t rw, uint32_t data)
{
    uint32_t cmd = (addr&0x0F) | ((rw&1)<<4) | (data<<5); // Сборка команды
    GPIOB->BRR  = (1<<SPILE_PIN);            // CS=Low
    __NOP(); __NOP();                        // Мини-задержка
    for(uint8_t i=0;i<25;i++) {               // 25 бит LSB first
        if(cmd & 1) GPIOB->BSRR=(1<<SPIDATA_PIN);
        else       GPIOB->BRR=(1<<SPIDATA_PIN);
        GPIOB->BSRR=(1<<SPICLK_PIN);         // CLK=High
        __NOP(); __NOP();                    // Пауза
        GPIOB->BRR=(1<<SPICLK_PIN);          // CLK=Low
        cmd >>= 1;                           // Следующий бит
    }
    GPIOB->BSRR=(1<<SPILE_PIN);              // CS=High
}

/* -------------------- Измерение BURST -------------------- */
uint32_t pulse_width_fast(void)
{
    uint32_t t0 = TIM2->CNT;                 // Фиксируем время начала
    while((GPIOA->IDR&(1<<BURST_PIN)) && ((TIM2->CNT-t0)<300)); // Ожидание конца старого
    t0 = TIM2->CNT;
    while(!(GPIOA->IDR&(1<<BURST_PIN)) && ((TIM2->CNT-t0)<300)); // Ожидание фронта
    if(!(GPIOA->IDR&(1<<BURST_PIN))) return 0;                   // Пропуск при таймауте
    uint32_t t1 = TIM2->CNT;
    while((GPIOA->IDR&(1<<BURST_PIN)) && ((TIM2->CNT-t1)<300)); // Длительность импульса
    return (TIM2->CNT - t1);
}
int32_t RSSI_dbmcalc(int raw_adc)
{
    // Ваше приближённое преобразование оставлено
    return -80 + (raw_adc - 750) / 11;
}
/* -------------------- Преобразование и предвычисление -------------------- */
int calc_freq_cmd(int f)
{
    int N = ((f-479)/2)/32;                  // Младшие биты N
    int A = ((f-479)/2)%32;                  // Оставшиеся биты A
    return (N<<7)|(A&0x1FFF);
}

void precompute_freqs(void)
{
    for(int i=0;i<FREQ_CNT;i++) {
        freq_list[i] = FREQ_MIN + i*FREQ_STEP;                // Сохраняем частоту
        // PLL команда сразу не храним, вычислим в set_freq_idx
    }
}

static inline void set_freq_idx(uint16_t idx)
{
    freq = freq_list[idx];                    // Обновляем глобальную freq
    sendSPICommand_rx5808(0x01,1,calc_freq_cmd(freq)); // Отправка PLL-команды
    delay_us(200);                             // Ждать стабилизации PLL (~200 us)
}

static inline void blink_once(void)
{
    GPIOB->BSRR = (1<<BLINK_PIN);             // PB2=High
    __NOP();        // Короткий импульс
    GPIOB->BRR  = (1<<BLINK_PIN);             // PB2=Low
}

/* -------------------- Отрисовка экрана -------------------- */
void SFrq_draw(int f,int rssi)
{
    char buf[32];
    ssd1306_Fill(Black);                      // Очистить экран
    sprintf(buf,"%d MHz",f);                // Формируем строку частоты
    ssd1306_SetCursor(0,5);
    ssd1306_WriteString(buf,Font_7x10,White);
    sprintf(buf,"RSSI = %d dBm",rssi);      // Формируем строку RSSI
    ssd1306_SetCursor(0,20);
    ssd1306_WriteString(buf,Font_7x10,White);
    ssd1306_DrawRect(RSSI_X,RSSI_Y,RSSI_W,RSSI_H,White);// Рисуем рамку
    int w = rssi+RSSI_MINVAL;                 // Вычисляем длину полоски
    if(w<0) w=0; if(w>RSSI_W) w=RSSI_W;
    ssd1306_FillRect(RSSI_X+1,RSSI_Y+1,w,RSSI_H-2,White);// Заполняем
    ssd1306_UpdateScreen();                   // Обновить
}

/* -------------------- main -------------------- */
int main(void)
{
    SystemInit();                              // Базовая инициализация
    clock_config();                            // Настройка тактов
    initSysTick();                             // Настройка SysTick
    tim2_init();                               // Настройка TIM2
    gpio_init();                               // Настройка GPIO
    adc_init();                                // Настройка ADC
    i2c1_init();                               // Настройка I2C для OLED
    ssd1306_Init();                            // Инициализация дисплея
    spi_gpio_init();                           // Настройка SPI GPIO
    precompute_freqs();                        // Подготовка списка частот

//    // Индикация старта: 4 мигания LED_PIN
//    GPIOA->BSRR=(1<<LED_PIN); delay_ms(5000);
//    GPIOA->BRR =(1<<LED_PIN); delay_ms(5000);
//    GPIOA->BSRR=(1<<LED_PIN); delay_ms(5000);
//    GPIOA->BRR =(1<<LED_PIN); delay_ms(5000);

    int prev_rssi = -999;                      // Для сравнения изменения RSSI
    int start_idx = (5110 - FREQ_MIN)/FREQ_STEP;// Индекс для 5110 MHz
    if(start_idx<0) start_idx=0;
    if(start_idx>=FREQ_CNT) start_idx=0;
    uint16_t idx = start_idx;
    set_freq_idx(idx);                         // Устанавливаем стартовую частоту
    GPIOB->BRR=(1<<BLINK_PIN);                 // Сброс BLINK_PIN

    while(1)
    {
        idx++;                                 // Переход к следующему индексу
        if(idx>=FREQ_CNT) idx=0;               // Кольцевой обход диапазона
        set_freq_idx(idx);                     // Перестройка и задержка
        blink_once();                          // Индикатор смены частоты

        int rssi = RSSI_dbmcalc(adc_read());  // Считываем RSSI
        if(disp_divider++>=DISP_EVERY_N_STEPS || ABS(rssi-prev_rssi)>2)
        {
            SFrq_draw(freq+10,rssi);          // Отображаем на экране
            prev_rssi = rssi;
            disp_divider = 0;
        }

        int check_fpv = 0;
        for(int i=0;i<10;i++)                 // 10 замеров BURST
        {
            uint32_t dur = pulse_width_fast();
            if(dur>59 && dur<62) check_fpv++;  // Проверка длительности
        }

        if(check_fpv>4)                        // Если найдено ?5 импульсов
        {
            while(GPIOA->IDR & (1<<BUTTON_PIN))// Ждём отпускания кнопки
            {
                last_freq = freq;              // Сохраняем найденную freq
                int rr = RSSI_dbmcalc(adc_read());
                SFrq_draw(freq+10,rr);         // Постоянно обновляем экран
            }
            delay_ms(500);                     // Дебаунс кнопки
        }

        if(!(GPIOA->IDR & (1<<BUTTON_PIN)))    // Если кнопка нажата
        {
            freq = last_freq - 10;             // Шаг назад
            if(freq < FREQ_MIN) freq = FREQ_MIN;
            idx = (freq - FREQ_MIN)/FREQ_STEP;
            set_freq_idx(idx);
            blink_once();                      // Индикатор возврата
            delay_ms(500);                     // Дебаунс
        }
    }
}
