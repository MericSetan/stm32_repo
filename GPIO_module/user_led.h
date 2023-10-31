/*
 * user_led.h
 *
 *  Created on: Oct 27, 2023
 *      Author: meric
 */

#ifndef GPIO_MODULE_USER_LED_H_
#define GPIO_MODULE_USER_LED_H_
#include <stdint.h>

void user_led_init(void);
void user_led_on(uint8_t);
void user_led_off(uint8_t);
void user_led_toggle(uint8_t);

#endif /* GPIO_MODULE_USER_LED_H_ */
