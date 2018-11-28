/*
 * bmp280_interface.c
 *
 *  Created on: 28 nov. 2018
 *      Author: fp
 */

#include <stdio.h>
#include "bmp280_interface.h"

// Functions to pass to the bmp280 library for communication with the chip
int8_t BMP280_SPI_Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len){
	uint8_t cnt = 0;
	printf("Bytes: %d -->", len);

	// Enable SPI peripheral
	LL_SPI_Enable(SPI2);
	while(!LL_SPI_IsEnabled(SPI2));

	// Write register address control byte
	LL_SPI_TransmitData8(SPI2, reg_addr);
	// Wait for data to be transmitted
	while(!LL_SPI_IsActiveFlag_TXE(SPI2));

	printf(" -w-0x%x", reg_addr);
	// Loop for len times to receive all data
	do{
		// Write dummy word to read bytes
		LL_SPI_TransmitData8(SPI2, (uint8_t)0xFF);
		// Wait for data to be transmitted
		while(!LL_SPI_IsActiveFlag_TXE(SPI2));
		// Wait for data to be received
		while(!LL_SPI_IsActiveFlag_RXNE(SPI2));
		// Read and store data
		data[cnt] = LL_SPI_ReceiveData8(SPI2);
		printf(" -r-0x%x", data[cnt]);
		cnt++;
	} while(len > cnt);
	printf("\r\n");

	// Wait for SPI to finish it's tasks
	while(LL_SPI_IsActiveFlag_BSY(SPI2));
	// Disable SPI peripheral
	LL_SPI_Disable(SPI2);

	// Return 0 if successful
	return BMP280_OK;
}

int8_t BMP280_SPI_Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len){
	uint8_t cnt = 0;
	printf("Bytes: %d -->", len);

	// Enable SPI peripheral
	LL_SPI_Enable(SPI2);
	while(!LL_SPI_IsEnabled(SPI2));

	// Write register address control byte
	LL_SPI_TransmitData8(SPI2, reg_addr);
	// Wait for data to be transmitted
	while(!LL_SPI_IsActiveFlag_TXE(SPI2));
	printf(" -w-0x%x", reg_addr);

	// Loop for len times to write all data
	do{
		// Write data to register address
		LL_SPI_TransmitData8(SPI2, data[cnt]);
		// Wait for data to be transmitted
		while(!LL_SPI_IsActiveFlag_TXE(SPI2));
		printf(" -w-0x%x", data[cnt]);
		cnt++;
	} while(len > cnt);
	printf("\r\n");

	// Wait for SPI to finish it's tasks
	while(LL_SPI_IsActiveFlag_BSY(SPI2));
	// Disable SPI peripheral
	LL_SPI_Disable(SPI2);

	// Return 0 if successful
	return BMP280_OK;
}

void BMP280_Delay_ms(uint32_t period){
	LL_mDelay(period);
}

int8_t BMP280_Init(struct bmp280_dev *bmp){
	printf("Initiating BMP280 sensor...\r\n");
	int8_t rslt;
	struct bmp280_config conf;
	bmp280_com_fptr_t spi_read_fptr = &BMP280_SPI_Read;
	bmp280_com_fptr_t spi_write_fptr = &BMP280_SPI_Write;
	bmp280_delay_fptr_t delay_ms_fptr = &BMP280_Delay_ms;

	/* Sensor interface over SPI with native chip select line */
	bmp->dev_id  =  0;
	bmp->intf = BMP280_SPI_INTF;
	bmp->read = spi_read_fptr;
	bmp->write = spi_write_fptr;
	bmp->delay_ms = delay_ms_fptr;

	rslt = bmp280_init(bmp);
	if (rslt != BMP280_OK){
		printf("Initiation failed!\r\n");
		Blink_error();
		return STATUS_ERROR;
	}
	/* Sensor chip ID will be printed if initialization was successful */
	printf("Device found with chip id 0x%x\r\n", bmp->chip_id);
	Blink_ok();

	// Get sensor configurations
	rslt = bmp280_get_config(&conf, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Config read error!\r\n");
		Blink_error();
		return STATUS_ERROR;
	}
	printf("Config read ok!\r\n");
	printf("OS_PRES: conf = %d, dev->conf = %d\r\n", conf.os_pres, bmp->conf.os_pres);
	printf("OS_TEMP: conf = %d, dev->conf = %d\r\n", conf.os_temp, bmp->conf.os_temp);
	printf("ODR: conf = %d, dev->conf = %d\r\n", conf.odr, bmp->conf.odr);
	printf("FILTER: conf = %d, dev->conf = %d\r\n", conf.filter, bmp->conf.filter);

	// Set configuration values and configure sensor
	conf.os_pres = BMP280_OS_1X;
	conf.os_temp = BMP280_OS_1X;
	conf.odr = BMP280_ODR_1000_MS;
	conf.filter = BMP280_FILTER_COEFF_2;
	rslt = bmp280_set_config(&conf, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Config write error!\r\n");
		Blink_error();
		return STATUS_ERROR;
	}
	printf("Config write ok!\r\n");
	printf("OS_PRES: conf = %d, dev->conf = %d\r\n", conf.os_pres, bmp->conf.os_pres);
	printf("OS_TEMP: conf = %d, dev->conf = %d\r\n", conf.os_temp, bmp->conf.os_temp);
	printf("ODR: conf = %d, dev->conf = %d\r\n", conf.odr, bmp->conf.odr);
	printf("FILTER: conf = %d, dev->conf = %d\r\n", conf.filter, bmp->conf.filter);

	// Set power mode after configuration is written to sensor
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Power mode write error!\r\n");
		Blink_error();
		return STATUS_ERROR;
	}

	printf("Power mode write ok!\r\n");
	printf("OS_PRES: conf = %d, dev->conf = %d\r\n", conf.os_pres, bmp->conf.os_pres);
	printf("OS_TEMP: conf = %d, dev->conf = %d\r\n", conf.os_temp, bmp->conf.os_temp);
	printf("ODR: conf = %d, dev->conf = %d\r\n", conf.odr, bmp->conf.odr);
	printf("FILTER: conf = %d, dev->conf = %d\r\n", conf.filter, bmp->conf.filter);

	// Get sensor configurations after setting them
	rslt = bmp280_get_config(&conf, bmp);
	if(rslt != BMP280_OK){
		// Error handling
		printf("Config read error!\r\n");
		Blink_error();
		return STATUS_ERROR;
	}
	printf("Config read ok!\r\n");
	printf("OS_PRES: conf = %d, dev->conf = %d\r\n", conf.os_pres, bmp->conf.os_pres);
	printf("OS_TEMP: conf = %d, dev->conf = %d\r\n", conf.os_temp, bmp->conf.os_temp);
	printf("ODR: conf = %d, dev->conf = %d\r\n", conf.odr, bmp->conf.odr);
	printf("FILTER: conf = %d, dev->conf = %d\r\n", conf.filter, bmp->conf.filter);

	return STATUS_OK;
}

static int8_t Read_bmp280(struct bmp280_dev *bmp, uint8_t samples){
	LL_GPIO_ResetOutputPin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	/* Code from library readme-file */
	int8_t rslt;
	struct bmp280_uncomp_data ucomp_data;

	uint8_t meas_dur = bmp280_compute_meas_time(bmp);

	// Variables to store read-out values
	int32_t temp32 = 0;
	uint32_t pres32 = 0;
	uint32_t pres64 = 0;
	double temp = 0;
	double pres = 0;
	//double avgtemp = 0;
	//double avgpres = 0;

	printf("Measurement duration: %dms\r\n", meas_dur);

	/* Loop to read out 10 samples of data */
	for (uint8_t i = 0; (i < samples) && (rslt == BMP280_OK); i++) {
	    bmp->delay_ms(meas_dur); /* Measurement time */

	    rslt = bmp280_get_uncomp_data(&ucomp_data, bmp);
	    /* Check if rslt == BMP280_OK, if not, then handle accordingly */
	    if(rslt == BMP280_OK){
	    	temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, bmp);
	    	pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, bmp);
	    	pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, bmp);
	    	temp = bmp280_comp_temp_double(ucomp_data.uncomp_temp, bmp);
	    	pres = bmp280_comp_pres_double(ucomp_data.uncomp_press, bmp);
	    } else{
	    	LL_GPIO_SetOutputPin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	    	return STATUS_ERROR;
	    }
	    //avgtemp += temp;
	    //avgpres += pres;
	    printf("UT: %d\r\nUP: %d\r\nT32: %d\r\nP32: %d\r\nP64: %d\r\nP64N: %d\r\nT: %f\r\nP: %f\r\n",
	    		ucomp_data.uncomp_temp, ucomp_data.uncomp_press, temp32, pres32, pres64, pres64 / 256, temp, pres);

	    bmp->delay_ms(1000); /* Sleep time between measurements = BMP280_ODR_1000_MS */
	}
	//avgtemp = avgtemp / samples;
	//avgpres = avgpres / samples;
	//printf("AVG Temp: %f\r\nAVG Pres: %f\r\n", avgtemp, avgpres);
	//printf("UT: %d\r\nUP: %d\r\nT32: %d\r\nP32: %d\r\nP64: %d\r\nP64N: %d\r\nT: %f\r\nP: %f\r\n",
	//	    		ucomp_data.uncomp_temp, ucomp_data.uncomp_press, temp32, pres32, pres64, pres64 / 256, temp, pres);
	LL_GPIO_SetOutputPin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
	return STATUS_OK;
}
