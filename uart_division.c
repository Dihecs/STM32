#include "stm32f10x.h"

// Не забудь включить CR на отправке и CR+LF на приеме
#define RX_BUF_SIZE 64
char rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;

void uart_send_str(const char* str);
void parse_and_respond(const char* buf);

void rcc_enable_periph(void);
int main(void)
{

    rcc_enable_periph();
    // === ???????????? ===
    RCC->APB2ENR |= (1 << 14); // USART1
    RCC->APB2ENR |= (1 << 2);  // GPIOA

    // === ????????? GPIO ===
    // PA9 - TX (????. push-pull, 50 ???)
    GPIOA->CRH &= ~(0xF << 4);
    GPIOA->CRH |=  (0xB << 4);

    // PA10 - RX (input pull-up)
    GPIOA->CRH &= ~(0xF << 8);
    GPIOA->CRH |=  (0x4 << 8);   // Input with pull-up/down
    GPIOA->ODR  |= (1 << 10);    // pull-up

    // === ????????? USART1 ===
    USART1->BRR = 625; // 72 ??? / 9600 ???
    USART1->CR1 |= (1 << 2) | (1 << 3) | (1 << 5) | (1 << 13); // RX, TX, RXNEIE, UE

    // === ?????????? ?????????? USART1 ===
    NVIC_EnableIRQ(USART1_IRQn);

    uart_send_str("STM32 UART Calculator Ready\r\n");

    while (1)
    {
        // ???????? ???? ?????? ?? ?????? — ??? ? ??????????
    }
}

// === ?????????? ?? ?????? ????? ===
void USART1_IRQHandler(void)
{
    if (USART1->SR & (1 << 5)) // RXNE
    {
        char c = USART1->DR;

        if (c == '\r' || c == '\n')
        {
            rx_buf[rx_index] = '\0';
            if (rx_index > 0)
            {
                parse_and_respond(rx_buf);
                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < RX_BUF_SIZE - 1)
            {
                rx_buf[rx_index++] = c;
            }
        }
    }
}

// === ????????? ??????: ??????? ???? ????? ===
void parse_and_respond(const char* buf)
{
    int a = 0, b = 0;
    char out[64];

    if (sscanf(buf, "%d %d", &a, &b) == 2)
    {
        if (b == 0)
        {
            uart_send_str("Error: divide by zero\r\n");
        }
        else
        {
            snprintf(out, sizeof(out), "Result: %d\r\n", a / b);
            uart_send_str(out);
        }
    }
    else
    {
        uart_send_str("Usage: <a> <b>\r\n");
    }
}

// === ???????? ?????? ?? UART ===
void uart_send_str(const char* str)
{
    while (*str)
    {
        while (!(USART1->SR & (1 << 7))); // ????, ???? DR ???????????
        USART1->DR = *str++;
    }
}



void rcc_enable_periph(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}
