/*
 * user_button.h
 *
 *  Created on: Oct 27, 2023
 *      Author: meric
 */

#ifndef GPIO_MODULE_USER_BUTTON_H_
#define GPIO_MODULE_USER_BUTTON_H_
#include <stdint.h>
void user_button_init(void);
int32_t user_button_get_state();
#endif /* GPIO_MODULE_USER_BUTTON_H_ */
