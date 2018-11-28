/*
 * bmp280_interface.h
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#ifndef BMP280_INTERFACE_H_
#define BMP280_INTERFACE_H_

#include "../Drivers/BMP280_driver-master/bmp280.h"

int8_t BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

void BMP280_Delay_ms(uint32_t period);

int8_t BMP280_Init(struct bmp280_dev *bmp);

int8_t BMP280_Read(struct bmp280_dev *bmp);

#endif /* BMP280_INTERFACE_H_ */
