/*
 * user_led.c
 *
 *  Created on: Oct 27, 2023
 *      Author: meric
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>

void user_led_init(void){


	// enable CLOCK for GPIOD  SET 3. pin
	RCC->AHB1ENR |= (1<<3);

	// for OUTPUT 01   MODER 12,13,14,15  (GPIOx_MODER)

	GPIOD->MODER |= (1<<24);  //green
	GPIOD->MODER &= ~(1<<25); //green

	GPIOD->MODER |= (1<<26);  //orange
	GPIOD->MODER &= ~(1<<27); //orange

	GPIOD->MODER |= (1<<28);  //red
	GPIOD->MODER &= ~(1<<29); //red

	GPIOD->MODER |= (1<<30);  //blue
	GPIOD->MODER &= ~(1<<31); //blue

	// select output type  0: Output push-pull (GPIOx_OTYPER)

	GPIOD->OTYPER &=~(1<<12);
	GPIOD->OTYPER &=~(1<<13);
	GPIOD->OTYPER &=~(1<<14);
	GPIOD->OTYPER &=~(1<<15);

	// select speed MEDİUM 01  (GPIOx_OSPEEDR)

	GPIOD->OSPEEDR |= (1<<24);  //green
	GPIOD->OSPEEDR &= ~(1<<25); //green

	GPIOD->OSPEEDR |= (1<<26);  //orange
	GPIOD->OSPEEDR &= ~(1<<27); //orange

	GPIOD->OSPEEDR |= (1<<28);  //red
	GPIOD->OSPEEDR &= ~(1<<29); //red

	GPIOD->OSPEEDR |= (1<<30);  //blue
	GPIOD->OSPEEDR &= ~(1<<31); //blue



}
void user_led_on(uint8_t led_pin){

	GPIOD->ODR |=(1<<led_pin);

}

void user_led_off(uint8_t led_pin ){

	GPIOD->ODR &=~(1<<led_pin);
}

void user_led_toggle(uint8_t led_pin){

	GPIOD->ODR ^= (1<<led_pin);
}



