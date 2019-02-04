/*
 * esp8266_interface.c
 *
 *  Created on: 6 jan. 2019
 *      Author: fp
 */

#include <stdio.h>
#include "usart.h"
#include "esp8266_interface.h"

/**
 * @brief Get IP assigned to chip and print it on debug line
 * @return 0 for success, -1 for fail
 */
static int8_t ESP8266_Get_IP(void){
	return 0;
}

/**
 * @brief Get connection status for network or access point
 * @return 0 for success, -1 for fail
 */
static int8_t ESP8266_Get_Connection_Status(void){
	return 0;
}

int8_t ESP8266_Init(uint8_t mode, uint8_t mult_conn){
	return 0;
}

int8_t ESP8266_Join_AP(char *ssid, char *pwd){
	return 0;
}

int8_t ESP8266_Quit_AP(void){
	return 0;
}

int8_t ESP8266_Connect(char* protocol, uint8_t *ip, uint32_t port){
	return 0;
}

int8_t ESP8266_Close(uint8_t conn_id){
	return 0;
}

int8_t ESP8266_Send_Data(uint8_t conn_id, char* data){
	return 0;
}
