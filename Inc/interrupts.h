/*
 * interrupts.h
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

#endif /* INTERRUPTS_H_ */
