/*
 * bmp280_interface.c
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#include <stdio.h>
#include "spi.h"
#include "bmp280_interface.h"

// Functions to pass to the bmp280 library for communication with the chip
int8_t BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len){
	// Drive bmp280 chip select pin low to start conversation
	HAL_GPIO_WritePin(SPI2_NSS_BMP280_GPIO_Port, SPI2_NSS_BMP280_Pin, GPIO_PIN_RESET);
	// Write register address to sensor
	HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 1);
	// Receive len amount of data and print it
	HAL_SPI_Receive(&hspi2, data, len, 1);
	// Drive bmp280 chip select pin high to stop conversation
	HAL_GPIO_WritePin(SPI2_NSS_BMP280_GPIO_Port, SPI2_NSS_BMP280_Pin, GPIO_PIN_SET);

	printf("Bytes: %d -->", len);
	// Print register address and received data
	printf(" w:0x%x", reg_addr);
	for(int i = 0; i < len; i++){
		printf(" r:0x%x", data[i]);
	}
	printf("\r\n");

	// Return 0 if successful
	return BMP280_OK;
}

int8_t BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len){
	printf("Bytes: %d -->", len);
	// Combine register address and data to array and print it
	uint8_t spi_data[len + 1];
	spi_data[0] = reg_addr;
	printf(" w:0x%x", reg_addr);
	for(int i = 0; i < len; i++){
		spi_data[i+1] = data[i];
		printf(" w:0x%x", data[i]);
	}
	printf("\r\n");

	// Drive bmp280 chip select pin low to start conversation
	HAL_GPIO_WritePin(SPI2_NSS_BMP280_GPIO_Port, SPI2_NSS_BMP280_Pin, GPIO_PIN_RESET);
	// Write register address and data to sensor
	HAL_SPI_Transmit(&hspi2, spi_data, (len + 1), 1);
	// Drive bmp280 chip select pin high to stop conversation
	HAL_GPIO_WritePin(SPI2_NSS_BMP280_GPIO_Port, SPI2_NSS_BMP280_Pin, GPIO_PIN_SET);

	// Return 0 if successful
	return BMP280_OK;
}

void BMP280_Delay_ms(uint32_t period){
	// Wait for period ms
	HAL_Delay(period);
}

int8_t BMP280_Init(struct bmp280_dev *bmp, struct bmp280_config *conf){
	printf("Initiating BMP280 sensor...\r\n");
	int8_t rslt;

	// Configure function pointers to read, write and delay functions
	bmp280_com_fptr_t spi_read_fptr = &BMP280_SPI_Read;
	bmp280_com_fptr_t spi_write_fptr = &BMP280_SPI_Write;
	bmp280_delay_fptr_t delay_ms_fptr = &BMP280_Delay_ms;

	// Give library SPI settings and functions for read, write and delay
	bmp->dev_id  =  0;
	bmp->intf = BMP280_SPI_INTF;
	bmp->read = spi_read_fptr;
	bmp->write = spi_write_fptr;
	bmp->delay_ms = delay_ms_fptr;

	rslt = bmp280_init(bmp);
	if (rslt != BMP280_OK){
		printf("Initiation failed!\r\n");
		return 0;
	}
	// Print sensor chip ID if initialization was successful
	printf("Device found with chip id 0x%x\r\n", bmp->chip_id);

	// Set configuration values and configure sensor
	rslt = bmp280_set_config(conf, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Config write error!\r\n");
		return 0;
	}
	printf("Config write ok!\r\n");

	return 1;
}

int8_t BMP280_Forced_Read(struct bmp280_dev *bmp, double temp, double pres){
	int8_t rslt;
	struct bmp280_uncomp_data ucomp_data;

	// Set sensor in forced mode to perform one reading
	rslt = bmp280_set_power_mode(BMP280_FORCED_MODE, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Power mode write error!\r\n");
		return 0;
	}
	printf("Power mode write ok!\r\n");

	// Get time it takes to read 1 data sample and wait for so long
	uint8_t meas_dur = bmp280_compute_meas_time(bmp);
	printf("Measurement duration: %dms\r\n", meas_dur);
	bmp->delay_ms(meas_dur);

	// Get data sample from sensor
	rslt = bmp280_get_uncomp_data(&ucomp_data, bmp);

	if(rslt == BMP280_OK){
		// Compute temperature and pressure from reading and print it
		temp = bmp280_comp_temp_double(ucomp_data.uncomp_temp, bmp);
		pres = bmp280_comp_pres_double(ucomp_data.uncomp_press, bmp);
		printf("T: %f\r\nP: %f\r\n", temp, pres);
	} else{
		printf("Get uncomputed data error!\r\n");
		return 0;
	}

	return 1;
}
