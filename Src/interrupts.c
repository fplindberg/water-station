/*
 * interrupts.c
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#include "main.h"
#include "interrupts.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Handle GPIO EXTI interrupt
	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// Handle UART TX interrupt
	HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// Handle UART RX interrupt
	HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	// Handle UART Error interrupt
	while(1);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	// Handle RTC Alarm A interrupt
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	// Set global bmp280 read flag to get temp and pres from bmp280 sensor
	BMP280_READ_FLAG = 1;
}
