/*
 * interrupts.c
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#include "interrupts.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// Handle UART TX interrupt
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// Handle UART RX interrupt
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	// Handle UART Error interrupt
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	// Handle RTC Alarm A interrupt
}
