/*
 * MS5837.c
 *
 *  Created on: Feb 4, 2021
 *      Author: Zeynep Yurttas
 */


#include "MS5837.h"

extern I2C_HandleTypeDef hi2c3;
#define i2cHandler hi2c3

/****************************READ & WRITE FUNCTIONS*****************************************/

static HAL_StatusTypeDef I2C_Write(uint8_t addr) {
	if(HAL_I2C_Master_Transmit(&i2cHandler, MS5837_ADDR<<1, &addr, 1, 100) != HAL_OK)
		return MS5837_ERROR;

	return MS5837_OK;
}

static t_MS5837_Status I2C_Read(uint8_t addr,uint8_t *reg_data, uint8_t r_len){

	if(HAL_I2C_Master_Transmit(&i2cHandler, MS5837_ADDR<<1, &addr, 1, 100) != HAL_OK){
		return MS5837_ERROR;
	}

	HAL_Delay(20);

	if(HAL_I2C_Master_Receive(&i2cHandler, MS5837_ADDR<<1, reg_data, r_len, 100) == HAL_OK){
		return MS5837_OK;
	}

	return MS5837_ERROR;
}
/*******************************************************************************************/
static uint8_t crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}


/* Set fluid density to 1029 as default
 * Set  model to MS5837_30BA as default
 * Trigger a power-on reset right after power-on (this helps getting the right calibration values)
 * Read calibration data
 * Check if the values are transmitted correctly by using CRC calculation
 *
 *** After using this function, calibration data can be reached through MS5837->val.C ***
 *** This function returns communication result ***
 */
t_MS5837_Status MS5837_Init(MS5837_t *MS5837){
	MS5837->fluidDensity = 1029;
	MS5837->model = MS5837_30BA;

	/* Write one byte reset command */
	while( MS5837_Reset() != MS5837_OK ){
		MS5837_Reset();
	}

	HAL_Delay(20);
	while(1){
		/* Read calibration data from PROM */
		for(uint8_t i=0; i<7 ; i++){
			uint8_t buffer[2];
			I2C_Read(MS5837_PROM_READ + (i*2), buffer, 2);
			MS5837->val.C[i] = buffer[1] | (buffer[0] << 8);
		}
		uint8_t crcRead = MS5837->val.C[0] >> 12;
		uint8_t crcCalculated = crc4(MS5837->val.C);

		if ( crcCalculated == crcRead ) {
			return MS5837_OK;
		}
	}



}

/* Power-on reset function */
t_MS5837_Status MS5837_Reset(){
	if(I2C_Write(MS5837_RESET) == HAL_OK){
		HAL_Delay(20);
		return MS5837_OK;
	}
	return MS5837_ERROR;
}

/* This function reads ADC results (D1 & D2 values) */
static t_MS5837_Status MS5837_ADC(MS5837_t *MS5837){
	/* Send command to start calculation of D1 and D2 */
	if(I2C_Write(MS5837_CONVERT_D1_8192) != HAL_OK){
		return MS5837_ERROR;
	}

	HAL_Delay(20);

	uint8_t buffer[3];
	if(I2C_Read(MS5837_ADC_READ, buffer, 3) != MS5837_OK ){
		return MS5837_ERROR;
	}
	MS5837->val.D1 = buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);

	if(I2C_Write(MS5837_CONVERT_D2_8192) != HAL_OK){
		return MS5837_ERROR;
	}

	HAL_Delay(20);

	if(I2C_Read( MS5837_ADC_READ, buffer, 3) != MS5837_OK ){
		return MS5837_ERROR;
	}
	MS5837->val.D2 = buffer[2] | (buffer[1] << 8) | (buffer[0] << 16);
	return MS5837_OK;
}

void MS5837_30BA_Calc(MS5837_t *MS5837){
	int32_t dT, OFFi, Ti,SENSi = 0;
	int64_t SENS, OFF, OFF2, SENS2 = 0;

	MS5837_ADC(MS5837);

	dT = MS5837->val.D2 - ((uint32_t)MS5837->val.C[5] * 256l);
	OFF = (int64_t)MS5837->val.C[2] * 65536l + ((int64_t)dT * (int64_t)MS5837->val.C[4]) / 128l;
	SENS = (int64_t)MS5837->val.C[1] * 32768l + ((int64_t)dT * (int64_t)MS5837->val.C[3]) / 256l;

	MS5837->val.TEMP = 2000l + ((int64_t)dT * MS5837->val.C[6]) / 8388608LL;
	MS5837->val.P = (((int64_t)MS5837->val.D1 * SENS) / 2097152l - OFF) / 8192l;

	if((MS5837->val.TEMP/100)<20){
		Ti = (3*(int64_t)dT*(int64_t)dT)/8589934592LL;
		OFFi = (3*(MS5837->val.TEMP-2000l)*(MS5837->val.TEMP-2000l))/2;
		SENSi = (5*(MS5837->val.TEMP-2000l)*(MS5837->val.TEMP-2000l))/8;

		if((MS5837->val.TEMP/100) < -15){
			OFFi = OFFi + 7 * (MS5837->val.TEMP + 1500l) * (MS5837->val.TEMP + 1500l);
			SENSi = SENSi + 4 * (MS5837->val.TEMP + 1500l) * (MS5837->val.TEMP + 1500l);
		}
	}
	else{
		Ti = (2*(int64_t)dT*(int64_t)dT)/137438953472LL;
		OFFi = ((MS5837->val.TEMP-2000l)*(MS5837->val.TEMP-2000l))/16;
		SENSi = 0;
	}

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	MS5837->val.TEMP = (MS5837->val.TEMP - Ti);

	MS5837->val.P = ((MS5837->val.D1 * SENS2) / 2097152l - OFF2)/ 8192l;

	MS5837->temperature = (float) MS5837->val.TEMP / 100.0f;             // result of temperature in deg C
	MS5837->pressure = (float) MS5837->val.P / 10.0f;                 // BAR30 result of pressure in mBar

}

void MS5837_02BA_Calc(MS5837_t *MS5837){
	int32_t dT, OFFi, Ti,SENSi = 0;
	int64_t SENS, OFF, OFF2, SENS2 = 0;

	MS5837_ADC(MS5837);

	dT = MS5837->val.D2 - ((uint32_t)MS5837->val.C[5] * 256l);
	OFF = (int64_t)MS5837->val.C[2] * 131072l + ((int64_t)dT * (int64_t)MS5837->val.C[4]) / 64l;
	SENS = (int64_t)MS5837->val.C[1] * 65536l + ((int64_t)dT * (int64_t)MS5837->val.C[3]) / 128l;

	MS5837->val.TEMP = 2000l + ((int64_t)dT * MS5837->val.C[6]) / 8388608l;
	MS5837->val.P = (((int64_t)MS5837->val.D1 * SENS) / 2097152l - OFF) / 32768l;

	if((MS5837->val.TEMP/100)<20){
		Ti = (11*(int64_t)dT*(int64_t)dT)/(34359738368LL);
		OFFi = (31*(MS5837->val.TEMP-2000l)*(MS5837->val.TEMP-2000l))/8;
		SENSi = (63*(MS5837->val.TEMP-2000l)*(MS5837->val.TEMP-2000l))/32;
	}

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	MS5837->val.TEMP = (MS5837->val.TEMP - Ti);
	MS5837->val.P =(((MS5837->val.D1*SENS2)/2097152l-OFF2)/32768l);

	MS5837->temperature = (float) MS5837->val.TEMP / 100.0f;           // result of temperature in deg C
	MS5837->pressure = (float) MS5837->val.P / 100.0f;                 // BAR02 result of pressure in mBar

}

float MS5837_Depth(MS5837_t *MS5837) {
	//return (P_MS5837/100)*1.019716;
	return ((MS5837->pressure*100)-101300)/(MS5837->fluidDensity*9.80665);

}

float MS5837_Altitude(MS5837_t *MS5837) {
	//return (P_MS5837/100)*1.019716;
	return (1-pow((MS5837->pressure/1013.25),0.190284))*145366.45*0.3048;

}
