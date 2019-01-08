/*
 * esp8266_interface.h
 *
 *  Created on: 6 jan. 2019
 *      Author: fp
 */

#ifndef ESP8266_INTERFACE_H_
#define ESP8266_INTERFACE_H_

struct esp8266_dev{
	uint8_t mode;
	uint8_t multiple_connections;
	uint8_t active_connections[4][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
};

/**
 * @brief Initiate ESP8266 chip
 * @param mode Chip mode: 1 = Client, 2 = AP, 3 = Both
 * @param multiple_connections Enable multiple connections or not: 0 = Single connection, 1 = Multiple connections
 * @return 0 for success, not 0 for fail
 */
int8_t ESP8266_Init(uint8_t mode, uint8_t multiple_connections);

int8_t ESP8266_Join_AP(char *ssid, char *pwd);

int8_t ESP8266_Quit_AP(void);

int8_t ESP8266_Get_IP(uint8_t *ip);

int8_t ESP8266_Connect(char* protocol, uint8_t *ip, uint32_t port);

int8_t ESP8266_Close(uint8_t *ip);

int8_t ESP8266_Send_Data(uint8_t *ip, char* data);

#endif /* ESP8266_INTERFACE_H_ */
