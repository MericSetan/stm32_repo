/*
 * user_button_EXTI.c
 *
 *  Created on: Oct 31, 2023
 *      Author: meric
 */
//PA0    ----- with HAL -----
#include "stm32f4xx_hal.h"

void user_button_EXTI_init(void) {
	// 1- SET clock  (RCC_AHB1ENR)
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// ENABLE  SYSCFG CLCOK (for EXTI)
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	// 2- GPIO SETTINGS

	// select pin mode 00 for input Moder0(10) (GPIOx_MODER)
	GPIOA->MODER &= ~(1 << 0);
	GPIOA->MODER &= ~(1 << 1);

	// PULL-UP  PULL-DOWN
	GPIOA->PUPDR &= ~(1 << 0);
	GPIOA->PUPDR &= ~(1 << 1);

	// 3- EXTI SETTINGS

	// Select EXTI0   (GPIOA)  EXTICR1 0000: PA[x] pin
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA ;

	// EXTI0 Interrupt Unmask
	EXTI->IMR |= EXTI_IMR_IM0;

	// EXTI0 Interrupt Falling Edge
	EXTI->FTSR |= EXTI_FTSR_TR0;  // OR EXTI->FTSR |= (1<<0);

	// 4- NVIC SETTINGS
	// SET Priority
	NVIC_SetPriority(EXTI0_IRQn,2);

	// Enable Interrupt on NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);

}

/*
void EXTI0_IRQHandler(void){

   if((EXTI->PR &EXTI_PR_PR0) == EXTI_PR_PR0){
	   EXTI->PR |= EXTI_PR_PR0;
   }


}*/


