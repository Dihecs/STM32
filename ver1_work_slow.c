#include "stm32f10x.h"          // CMSIS-заголовок для STM32F103
#include "ssd1306.h"            // Заголовок для OLED-дисплея SSD1306 (I2C)
#include <stdio.h>              // Для sprintf()
#include "ssd1306_fonts.h"


// ---------------------------
// Константы и макросы
// ------------------------s---
#define SYSCLK_FREQ    72000000UL     // Частота системной шины — 72 МГц
#define BUTTON_PIN     10             // Кнопка на порту PA10
#define RSSI_CHAN      0              // АЦП канал 0 — это PA0
#define SPILE_PIN      12             // PB12 — вывод CS (SPI Latch Enable)
#define SPIDATA_PIN    15             // PB15 — вывод данных SPI
#define SPICLK_PIN     13             // PB13 — тактовый вывод SPI
#define BURST_PIN    		4             // PA4 — вход от LM1881 (BURST детектор)
#define LED_PIN        12             // PA12 — светодиод индикации
#define BLINK_PIN       2             // PB2 индикатор частоты

// Координаты и размеры полосы RSSI на экране
#define RSSI_X         10
#define RSSI_Y         45
#define RSSI_W         98
#define RSSI_H         10
#define RSSI_MINVAL    91
//#define ABS(x) ((x) < 0 ? -(x) : (x))

// ---------------------------
// Глобальные переменные
// ---------------------------
static volatile uint32_t msTicks; // Счётчик миллисекунд от SysTick
static int last_freq = 5000;      // Последняя зафиксированная частота
static int freq = 5000;           // Текущая частота поиска

// ---------------------------
// Прототипы функций
// ---------------------------
void SysTick_Handler(void);
void initSysTick(void);
void tim2_init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void gpio_init(void);
void adc_init(void);
uint16_t adc_read(void);
void i2c1_init(void);
void spi_gpio_init(void);
uint32_t pulse_width(void);
int32_t RSSI_dbmcalc(int raw);
int calc_freq_cmd(int f);
void sendSPICommand(uint8_t addr, uint8_t rw, uint32_t data);
void set_freq(int f);
//void SFrq_draw(int f, int rssi);
//void clock_config(void);

// ---------------------------
// Настройка тактирования: 72 МГц через PLL от HSE (внешний кварц 8 МГц)
//// ---------------------------
void clock_config(void) {
    RCC->CR    |= (1 << 0);               // Включить HSI (внутренний RC, на всякий)
    RCC->CFGR   = 0;                      // Сброс конфигурации шины
    RCC->CR    &= ~((1 << 16) | (1 << 24)); // Выключить HSE и PLL перед настройкой
    RCC->CIR    = 0;                      // Отключить все прерывания тактирования

    RCC->CR    |= (1 << 16);              // Включить HSE (внешний кварц)
    while (!(RCC->CR & (1 << 17)));       // Дождаться стабилизации HSE

    FLASH->ACR |= (1 << 4);               // Включить предварительную выборку (Prefetch buffer)
    FLASH->ACR &= ~(0x7);                 // Сброс LATENCY
    FLASH->ACR |= (2 << 0);               // Установить задержку 2 цикла (72 МГц)

    RCC->CFGR |= (0 << 4);                // AHB = SYSCLK / 1
    RCC->CFGR |= (4 << 8);                // APB1 = AHB / 2
    RCC->CFGR |= (0 << 11);               // APB2 = AHB / 1
    RCC->CFGR |= (2 << 14);               // ADC = PCLK2 / 6

    RCC->CFGR |= (1 << 16);               // PLLSRC = HSE
    RCC->CFGR |= (7 << 18);               // PLLMUL = x9 (8 × 9 = 72 МГц)

    RCC->CR   |= (1 << 24);               // Включить PLL
    while (!(RCC->CR & (1 << 25)));       // Ждать готовности PLL

    RCC->CFGR &= ~((1 << 0) | (1 << 1));  // Сброс SW
    RCC->CFGR |=  (2 << 0);               // SW = 10 → выбрать PLL
    while ((RCC->CFGR & (3 << 2)) != (2 << 2)); // Ждать подтверждения SWS = PLL

    SystemCoreClock = SYSCLK_FREQ;       // Обновить переменную CMSIS
}

// ---------------------------
// Обработчик SysTick — инкрементирует счётчик миллисекунд
// ---------------------------
void SysTick_Handler(void) {
    msTicks++;
}

// ---------------------------
// Инициализация системного таймера SysTick для 1 мс тика
// ---------------------------
void initSysTick(void) {
    SysTick_Config(SystemCoreClock / 1000); // Каждую 1 мс
}

// ---------------------------
// Задержка в миллисекундах
// ---------------------------
void delay_ms(uint32_t ms) {
    uint32_t tgt = msTicks + ms;
    while (msTicks < tgt);
}

// ---------------------------
// Задержка в микросекундах с точностью по DWT
// ---------------------------
void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) __NOP(); // Примерно 1 мкс при 72 МГц
}
void tim2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (SystemCoreClock / 1000000) - 1; // 1 мкс / тик
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}
// ---------------------------
// Инициализация GPIO
// ---------------------------
void gpio_init(void) {
    RCC->APB2ENR |= (1 << 2) | (1 << 3);
               
	
    // Кнопка на PA10 — вход с подтяжкой вверх (pull-up)
    GPIOA->CRH &= ~(0xF << ((BUTTON_PIN - 8) * 4));
    GPIOA->CRH |=  (0x8 << ((BUTTON_PIN - 8) * 4));  // CNF=10, MODE=00
    GPIOA->ODR |=  (1 << BUTTON_PIN);               // Включаем подтяжку
	
	
	  GPIOA->CRL &= ~(0xF << (BURST_PIN * 4));   // очистить CNF4/MODE4
    GPIOA->CRL |=  (0x8 << (BURST_PIN * 4));   // CNF=10 (pull), MODE=00 (input)
    GPIOA->ODR &= ~(1 << BURST_PIN);           // подтяжка вниз

    // Светодиод на PA12 — выход push-pull
    GPIOA->CRH &= ~(0xF << ((LED_PIN - 8) * 4));
    GPIOA->CRH |=  (0x2 << ((LED_PIN - 8) * 4));     // CNF=00, MODE=10 (выход 2 МГц)
	  // PB2 BLINK
    GPIOB->CRL &= ~(0xF << (BLINK_PIN * 4));
    GPIOB->CRL |=  (0x2 << (BLINK_PIN * 4));
    GPIOB->BRR  =  (1 << BLINK_PIN);
}

// ---------------------------
// Инициализация ADC
// ---------------------------
void adc_init(void) {
    RCC->APB2ENR |= (1 << 9);        // Включить тактирование ADC1
    RCC->CFGR    |= (2 << 14);       // Делитель ADC = PCLK2 / 6
    ADC1->CR2     = (1 << 0);        // Включить ADC (ADON)
}

// ---------------------------
// Преобразование АЦП и возврат результата
// ---------------------------
uint16_t adc_read(void) {
    ADC1->SQR3 &= ~0x1F;                      // Очистить выбор канала
    ADC1->SQR3 |= (RSSI_CHAN & 0x1F);         // Установить канал PA0

    ADC1->CR2 |= (1 << 0);                    // Повторно включить ADC
    ADC1->CR2 |= (1 << 20);                   // Разрешить программный триггер
    ADC1->CR2 |= (1 << 22);                   // Старт преобразования

    while (!(ADC1->SR & (1 << 1)));           // Ждать окончания (EOC)
    return (uint16_t)(ADC1->DR & 0xFFFF);     // Вернуть результат
}

// ---------------------------
// Инициализация I2C1 для OLED SSD1306
// ---------------------------
void i2c1_init(void) {
    RCC->APB2ENR |= (1 << 3);        // Включить порт B
    RCC->APB1ENR |= (1 << 21);       // Включить I2C1

    // PB6=SCL, PB7=SDA — alternate function open-drain
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB->CRL |=  ((0xB << 24) | (0xB << 28));

    I2C1->CR1 = (1 << 15);          // Сброс I2C
    I2C1->CR1 = 0;

    I2C1->CR2  = (SystemCoreClock / 1000000);     // APB1 freq в МГц
    I2C1->CCR  = (SystemCoreClock / 100000) / 2;  // Делитель для 100 кГц
    I2C1->TRISE= (SystemCoreClock / 1000000) + 1; // Максимальный рост фронта
    I2C1->CR1 |= (1 << 0);          // Включить I2C
}

// ---------------------------
// Инициализация GPIO для SPI-передачи (битбэнг)
// ---------------------------
void spi_gpio_init(void) {
    RCC->APB2ENR |= (1 << 3);       // Включить порт B

    // Настроить PB12, PB13, PB15 как выход push-pull
    GPIOB->CRH &= ~((0xF << 16)|(0xF << 20)|(0xF << 28));
    GPIOB->CRH |=  ((0x2 << 16)|(0x2 << 20)|(0x2 << 28));

    GPIOB->BSRR = (1 << SPILE_PIN); // CS HIGH (неактивен)
}
// ---------------------------
// Измерение ширины BURST-сигнала от LM1881 (в микросекундах)
// ---------------------------
uint32_t pulse_width(void) {
    uint32_t start_cnt, timeout;

    // Ждём окончания предыдущего импульса, но не дольше 1 мс
    start_cnt = TIM2->CNT;
    timeout   = start_cnt + 1000;            // 1000 us таймаут
    while (GPIOA->IDR & (1 << BURST_PIN)) {
        if ((TIM2->CNT - start_cnt) > 1000) return 0;
    }

    // Ждём начала импульса (фронт)
    start_cnt = TIM2->CNT;
    timeout   = start_cnt + 1000;
    while (!(GPIOA->IDR & (1 << BURST_PIN))) {
        if ((TIM2->CNT - start_cnt) > 1000) return 0;
    }

    // Запоминаем время старта
    start_cnt = TIM2->CNT;

    // Ждём конца импульса
    timeout = start_cnt + 1000;
    while (GPIOA->IDR & (1 << BURST_PIN)) {
        if ((TIM2->CNT - start_cnt) > 1000) break;
    }

    // Вернём измеренную длительность, или 0 при таймауте
    return (TIM2->CNT - start_cnt);
}

// ---------------------------
// Перевод "сырых" значений АЦП в dBm
// ---------------------------
int32_t RSSI_dbmcalc(int raw) {
    return (int32_t)(0.468 * raw - 139);  // Калибровочная формула
}

// ---------------------------
// Вычисление команды частоты для SPI-модуля (например, RX5808)
// ---------------------------
int calc_freq_cmd(int f) {
    int N = ((f - 479) / 2) / 32;         // N = делитель PLL
    int A = ((f - 479) / 2) % 32;         // A = остаток
    return (N << 7) | (A & 0x1FFF);       // Сборка команды (формат PLL: 7 бит N + 13 бит A)
}

// ---------------------------
// Передача команды по SPI (битбэнг, вручную)
// ---------------------------
void sendSPICommand(uint8_t addr, uint8_t rw, uint32_t data) {
    uint32_t cmd = (addr & 0x0F) | ((rw & 1) << 4) | (data << 5); // Формируем 25-битную команду

    GPIOB->BRR  = (1 << SPILE_PIN);  // CS = LOW (начало передачи)
    delay_us(1);

    for (uint8_t i = 0; i < 25; i++) {   // Передаём побитно
        if (cmd & (1 << i))
            GPIOB->BSRR = (1 << SPIDATA_PIN); // Устанавливаем DATA в 1
        else
            GPIOB->BRR  = (1 << SPIDATA_PIN); // DATA = 0

        GPIOB->BSRR = (1 << SPICLK_PIN);      // Такт ↑
        delay_us(1);
        GPIOB->BRR  = (1 << SPICLK_PIN);      // Такт ↓
        delay_us(1);
    }

    GPIOB->BSRR = (1 << SPILE_PIN);   // CS = HIGH (конец передачи)
    delay_us(1);
}

// ---------------------------
// Установка рабочей частоты модуля (через SPI)
// ---------------------------
void set_freq(int f) {
    sendSPICommand(0x01, 1, calc_freq_cmd(f)); // Адрес 0x01, запись (rw=1), данные — PLL команда
}

// ---------------------------
// Отрисовка информации на экране: частота, RSSI и индикатор
// ---------------------------
//void SFrq_draw(int f, int rssi) {
//    char buf[32];
//    ssd1306_Fill(Black); // Очистка дисплея

//    sprintf(buf, "%d MHz", f); // Строка с частотой
//    ssd1306_SetCursor(0, 5);
//    ssd1306_WriteString(buf, Font_7x10, White);

//    sprintf(buf, "RSSI = %d dBm", rssi); // Строка с уровнем сигнала
//    ssd1306_SetCursor(0, 20);
//    ssd1306_WriteString(buf, Font_7x10, White);

//    // Рисуем рамку и уровень сигнала (полоску)
//    ssd1306_DrawRect(RSSI_X, RSSI_Y, RSSI_W, RSSI_H, White);
//    int w = rssi + RSSI_MINVAL; // Сдвиг в положительный диапазон
//    if (w < 0) w = 0;
//    if (w > RSSI_W) w = RSSI_W;
//    ssd1306_FillRect(RSSI_X + 1, RSSI_Y + 1, w, RSSI_H - 2, White);

//    ssd1306_UpdateScreen(); // Обновить дисплей
//}

// ---------------------------
// Основная функция
// ---------------------------
int main(void) {
    SystemInit();           // Инициализация системы (CMSIS)
		//RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               // Включить тактирование AFIO
		//AFIO->MAPR &= ~(7 << 24);                         // Очистить поле SWJ_CFG (биты 26:24)
		//AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;      // Отключить JTAG, оставить SWD
    clock_config();         // Настройка тактирования (72 МГц)
    initSysTick();          // SysTick (1 мс тики)
    tim2_init();            
    gpio_init();            // GPIO (кнопка, LED)
    adc_init();             // АЦП (RSSI)
    i2c1_init();            // I2C (дисплей)
    ssd1306_Init();         // Инициализация дисплея
    spi_gpio_init();        // SPI (модуль частоты)

    // Моргаем светодиодом 2 раза для индикации запуска
    GPIOA->BSRR = (1 << LED_PIN); delay_ms(100);
    GPIOA->BRR  = (1 << LED_PIN); delay_ms(100);
    GPIOA->BSRR = (1 << LED_PIN); delay_ms(100);
    GPIOA->BRR  = (1 << LED_PIN); delay_ms(100);
//		int prev_freq = -1;
//		int prev_rssi = -999;

    // Сначала устанавливаем 5110 МГц
    set_freq(5110);
    //SFrq_draw(5110, RSSI_dbmcalc(adc_read()));
		GPIOB->BRR = (1 << BLINK_PIN); 
    while (1) {
        int check_fpv = 0;         // Счётчик наличия видеосигнала
        freq += 5;                 // Шаг частоты +5 МГц
        if (freq > 6200) freq = 4850; // Циклично (перебор от 4850 до 6200 МГц)
        set_freq(freq); // Установить текущую частоту
//				int rssi = RSSI_dbmcalc(adc_read());
//				if (freq != prev_freq || ABS(rssi - prev_rssi) > 2) {
//				SFrq_draw(freq + 10, rssi);
//				prev_freq = freq;
//				prev_rssi = rssi;


        //SFrq_draw(freq + 10, RSSI_dbmcalc(adc_read())); // Показать частоту и RSSI
				// Моргание PB2 при смене частоты
        GPIOB->BSRR = (1 << BLINK_PIN);
        delay_ms(10);
        GPIOB->BRR  = (1 << BLINK_PIN);
				delay_ms(10);
        // 10 измерений burst-импульсов
        for (int i = 0; i < 10; i++) {
            uint32_t dur = pulse_width();
            if (dur > 59 && dur < 62) check_fpv++; // Если в пределах BURST, увеличиваем счётчик
        }

        if (check_fpv > 4) { // Если 5 и более BURST — сигнал найден
            while (GPIOA->IDR & (1 << BUTTON_PIN)) { // Пока кнопка НЕ нажата
               // SFrq_draw(freq + 10, RSSI_dbmcalc(adc_read())); // Показываем данные
                last_freq = freq;  // Сохраняем частоту
            }
            delay_ms(500); // Дебаунс
        }

        // Если кнопка нажата (сброс частоты назад)
        if (!(GPIOA->IDR & (1 << BUTTON_PIN))) {
            freq = last_freq - 10; // Шаг назад
            delay_ms(500);         // Задержка
        }
    }
	}
