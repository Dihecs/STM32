 
#include "stm32f10x.h"
void clock_config(void);
int main(void)
{
	clock_config();
	RCC->APB2ENR |= 1<<14; // ???????? ???????????? UART1
	RCC->APB2ENR |= 1<<2; // ???????? ???????????? ????? A
	GPIOA->CRH |= 1<<9;	//????????????? PA10 ??? ???? ? ?????????
	GPIOA->CRH &= ~(1<<8);	//??????? ??????? ?? ?????????
	GPIOA->ODR |= 1<<10;//?????? ???????? PA10 ? ?????
	//A0,A2,A3 - ?????, ???????? ????, ??????? 50???
	GPIOA->CRL |= 3 | (3<<8) | (3<<12);
	GPIOA->ODR |= 1|4|8;//???????? ??????????
	USART1->BRR = 625; 
	//?????????????? UART(13), ?????(2), ?????????? ?? ??????(5)
	USART1->CR1 |= (1<<2) | (1<<5) | (1<<13);
	NVIC_EnableIRQ (USART1_IRQn); //???????? ??????????, ????????? ??????
	while(1){}
}	
void USART1_IRQHandler() //??????? ??????????? ??????????
{
	if( USART1->SR & ~(1<<5) ) // 5-RXNE
	{
		char symbol = USART1->DR;
		if(symbol=='r')
		{
			GPIOA->ODR &= ~1;
			GPIOA->ODR |= 4|8;
		}
		if(symbol=='g')
		{
			GPIOA->ODR &= ~4;
			GPIOA->ODR |= 1|8;
		}
		if(symbol=='b')
		{
			GPIOA->ODR &= ~8;
			GPIOA->ODR |= 1|4;
		}
	}
}	
void clock_config(void) {
    RCC->CR |= RCC_CR_HSION;
    RCC->CFGR = 0;
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_PLLON);
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

    RCC->CFGR |= RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
