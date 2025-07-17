#include "stm32f10x.h"
#include <stdio.h>
#include "lcd_i2c.h"

void delay(volatile uint32_t t) {
    while (t--) __NOP();
}

// === PWM ?? PA8: 72 MHz / (PSC+1) / (ARR+1) ===
void pwm_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;  // 50 ???
    GPIOA->CRH |= GPIO_CRH_CNF8_1;

    TIM1->PSC = 0;      
    TIM1->ARR = 8 - 1; 
    TIM1->CCR1 = 4;     

    TIM1->CCMR1 |= (6 << 4);       
    TIM1->CCER  |= TIM_CCER_CC1E;  
    TIM1->BDTR  |= TIM_BDTR_MOE;
    TIM1->CR1   |= TIM_CR1_CEN;
}

void adc_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->CRL &= ~(0xF << 0); 
    ADC1->SMPR2 |= 7 << 0;     
    ADC1->SQR3 = 0;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(10000);
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
}

uint32_t adc_read_mv(void) {
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint16_t raw = ADC1->DR;
    return (raw * 3300UL) / 4095;
}

int32_t mv_to_dbm(uint32_t mv, float intercept) {
    float v = mv / 1000.0f;
    float slope = -0.025f;
    return (int32_t)((v - intercept) / slope);
}

int main(void) {
    lcd_i2c_init();
    pwm_init();
    adc_init();

    lcd_i2c_clear();
    lcd_i2c_set_cursor(0, 0);
    lcd_i2c_print("Kalibrovka...");

    const int32_t ref_dbm = 0;
    uint32_t mv_now = adc_read_mv();
    float v_now = mv_now / 1000.0f;
    float slope = -0.025f;
    const float intercept = 0.5f; 
    delay(1000000);

    char line1[17];
    char line2[17];

    while (1) {
        uint32_t mv = adc_read_mv();
        int32_t dbm = mv_to_dbm(mv, intercept);

        snprintf(line1, 17, "U: %4lu mV       ", mv);
        snprintf(line2, 17, "Pwr: %4ld dBm    ", dbm);

        lcd_i2c_set_cursor(0, 0);
        lcd_i2c_print(line1);

        lcd_i2c_set_cursor(0, 1);
        lcd_i2c_print(line2);

        delay(1500000);
    }
}
