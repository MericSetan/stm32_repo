/*
 * user_button.c
 *
 *  Created on: Oct 27, 2023
 *      Author: meric
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>
// PA0
void user_button_init(){
	// PA0

	// SET clock  (RCC_AHB1ENR)
	RCC->AHB1ENR |= (1<<0);

	// select pin mode 00 for input Moder0(10) (GPIOx_MODER)
	GPIOA->MODER &= ~(1<<0);
	GPIOA->MODER &= ~(1<<1);

	// select output type (GPIOx_OTYPER) OT0(0) push-pull
	GPIOA->OTYPER &= ~(1<<0);

	// select speed  01 medium (GPIOx_OSPEEDR)    OSPEEDR0(10)
	GPIOA->OSPEEDR |= (1<<0);
	GPIOA->OSPEEDR &= ~(1<<1);

	// select pull-up/down 00 (GPIOx_PUPDR)  PUPDR0(10)
	GPIOA->PUPDR &= ~(1<<0);
	GPIOA->PUPDR &= ~(1<<1);


}

int32_t user_button_get_state(){
	if((GPIOA->IDR &(1<<0) )==1){
		return 1;
	}else{
		return 0;
	}

}
