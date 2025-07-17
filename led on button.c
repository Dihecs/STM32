#include "stm32f10x.h"

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    // PA0 - кнопка с подтяжкой вверх
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;  // вход с подтяжкой
    GPIOA->ODR |= (1 << 0);         // включаем подтяжку

    // PB2 - выход для лампы (изначально выключена)
    GPIOB->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOB->CRL |= GPIO_CRL_MODE2_1; // выход 2 МГц
    GPIOB->ODR &= ~(1 << 2);        // выключаем лампу

    uint8_t button_state = 1;       // текущее состояние кнопки
    uint8_t last_button_state = 1;  // предыдущее состояние кнопки
    uint8_t lamp_state = 0;         // состояние лампы (0 - выкл, 1 - вкл)

    while (1) {
        button_state = (GPIOA->IDR & (1 << 0)) ? 1 : 0;
        
        // Обнаружение фронта нажатия (кнопка была отпущена, теперь нажата)
        if (button_state == 0 && last_button_state == 1) {
            lamp_state ^= 1; // инвертируем состояние лампы
            if (lamp_state) {
                GPIOB->ODR |= (1 << 2);  // включаем лампу
            } else {
                GPIOB->ODR &= ~(1 << 2); // выключаем лампу
            }
            
            // Простая защита от дребезга - ждем отпускания кнопки
            while (!(GPIOA->IDR & (1 << 0)));
        }
        
        last_button_state = button_state;
    }
}