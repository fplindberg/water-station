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
	char ssid[30];
	uint8_t active_connections[4][4];
};

/**
 * @brief Initiate ESP8266 chip and abstract layer
 * @param mode Chip mode: 1 = Client, 2 = AP, 3 = Both
 * @param mult_conn Enable multiple connections, 0 = Single, 1 = Multiple
 * @return 0 for success, -1 for fail
 */
int8_t ESP8266_Init(uint8_t mode, uint8_t mult_conn);

/**
 * @brief Establish connection to network or access point
 * @param ssid Network name to connect to
 * @param pwd Password for network name
 * @return 0 for success, -1 for fail
 */
int8_t ESP8266_Join_AP(char *ssid, char *pwd);

/**
 * @brief Break connection with network or access point
 * @return 0 for success, -1 for fail
 */
int8_t ESP8266_Quit_AP(void);

/**
 * @brief Establish connection to IP-address and port number using protocol
 * @param protocol Communication protocol to be used
 * @param ip IP-address to host to connect to
 * @param port Port-number to use for communication
 * @return Connection-ID for success, -1 for fail
 */
int8_t ESP8266_Connect(char* protocol, uint8_t *ip, uint32_t port);

/**
 * @brief Close connection with specified ID
 * @param conn_id Connection-ID of connection to close
 * @return 0 for success, -1 for fail
 */
int8_t ESP8266_Close(uint8_t conn_id);

/**
 * @brief Send data on connection with specified ID
 * @param conn_id Connection-ID of connection to send data to
 * @param data Data to send
 * @return 0 for success, -1 for fail
 */
int8_t ESP8266_Send_Data(uint8_t conn_id, char* data);

#endif /* ESP8266_INTERFACE_H_ */
