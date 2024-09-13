/*
 * led_status.c
 *
 *  Created on: Sep 9, 2024
 *      Author: JHON GARCIA
 */
#include "main.h"
#include "led_status.h"

void heartbeat(void)
{
	static uint32_t heartbeat_tick = 0;
	if (heartbeat_tick < HAL_GetTick()) {
		heartbeat_tick = HAL_GetTick() + 250;
		HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port,GPIO_PIN_5);
	}
}
