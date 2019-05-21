
#include "UM7_reg.h"


// Function that will use SPI to write data to a register
uint8_t UM7_Write_Data(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t length, uint8_t reg)
{
	uint8_t dummy = 0;
	HAL_GPIO_WritePin(GPIOB, SPI1_CHIPSEL_Pin, GPIO_PIN_RESET);

	// Create request for the register value
	uint8_t requestWrite[2] = {UM7_WRITE_BIT, reg};
	// Send the register that we want to write from
	HAL_SPI_Transmit(hspi, &requestWrite[0], 1, 10);
	HAL_SPI_Transmit(hspi, &requestWrite[1], 1, 10);
	
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
	//HAL_SPI_Receive(hspi, &dummy, 1, 10);
		
	// Loop through 4 times to get each byte of the register data
	for (int8_t i = length - 1; i >= 0; i--)
	{
		HAL_SPI_Transmit(hspi, &data[i], 1, 10);
		//HAL_SPI_Receive(&hspi1, &dummy, 1, 10);
	}
	
	HAL_GPIO_WritePin(GPIOB, SPI1_CHIPSEL_Pin, GPIO_PIN_SET);
	return 0;
}

// Function that will use SPI to read data from 1 register at a time 
uint8_t UM7_Read_Data(SPI_HandleTypeDef *hspi, uint8_t *data, uint8_t length, uint8_t reg)
{
	uint8_t dummy = 0;
	HAL_GPIO_WritePin(GPIOB, SPI1_CHIPSEL_Pin, GPIO_PIN_RESET);

	// Create request for the register value
	uint8_t requestData[2] = {UM7_READ_BIT, reg};
	// Send the register that we want to read from
	HAL_SPI_Transmit(hspi, &requestData[0], 1, 10);
	HAL_SPI_Transmit(hspi, &requestData[1], 1, 10);
	
	while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY){};
	//HAL_SPI_Receive(hspi, &dummy, 1, 10);
	// Loop through 4 times to get each byte of the register data
	for (int8_t i = length - 1; i >= 0; i--)
	{
		HAL_SPI_Receive(hspi, &data[i], 1, 10);
		HAL_SPI_Transmit(hspi, 0x00, 1, 10);
	}
		//HAL_SPI_Receive(hspi, &dummy, 1, 10);
	HAL_GPIO_WritePin(GPIOB, SPI1_CHIPSEL_Pin, GPIO_PIN_SET);
	return 0;
}

// set mode
void UM7_Init(SPI_HandleTypeDef *hspi, uint8_t gyrRate, uint8_t magRate)
{
	// Write the frequency to read gyroscope data
	uint8_t procRate[4]  = {0x0, magRate, gyrRate, 0x0};
	uint8_t rawRate[4]   = {0x0, magRate, gyrRate, 0x0};
	UM7_Write_Data(hspi,  rawRate, 4, CREG_COM_RATES1);
	HAL_Delay(200);
	UM7_Write_Data(hspi, procRate, 4, CREG_COM_RATES3);
	return;
}

// Get Sensor Data
// get magnetometer data (microteslas)
void get_mag_data(SPI_HandleTypeDef *hspi, float *data)
{
	// Read the x, y, and z data from the magnetometer
	uint8_t byteData[12] = {0};
	UM7_Read_Data(hspi, &byteData[0], 4, DREG_MAG_PROC_X);
	UM7_Read_Data(hspi, &byteData[4], 4, DREG_MAG_PROC_Y);
	UM7_Read_Data(hspi, &byteData[8], 4, DREG_MAG_PROC_Z);
	
	uint32_t *outData = (uint32_t *) data;
	
	// Reassemble the data into floating point values
	for (uint8_t i = 0; i < 3; i++)
	{
		outData[i] = (byteData[i*4 + 3] << 24) |	(byteData[i*4 + 2] << 16) 
								| (byteData[i*4 + 1] << 8) | (byteData[i*4 + 0] << 0);
	}
	
	return;
}

// get gyroscope data
void get_gyr_data(SPI_HandleTypeDef *hspi, float *data)
{
	// Read the x, y, and z data from the magnetometer
	uint8_t byteData[12] = {0};
	UM7_Read_Data(hspi, &byteData[0], 4, DREG_GYRO_PROC_X);
	UM7_Read_Data(hspi, &byteData[4], 4, DREG_GYRO_PROC_Y);
	UM7_Read_Data(hspi, &byteData[8], 4, DREG_GYRO_PROC_Z);
	
	uint32_t *outData = (uint32_t *) data;
	
	// Reassemble the data into floating point values
	for (uint8_t i = 0; i < 3; i++)
	{
		outData[i] = (byteData[i*4 + 3] << 24) |	(byteData[i*4 + 2] << 16) 
								| (byteData[i*4 + 1] << 8) | (byteData[i*4 + 0] << 0);
		
		// Convert to rad/s
		data[i] = data[i]*PI/180.0;
	}

	return;
}


void get_raw_data(SPI_HandleTypeDef *hspi, int16_t *data)
{
	// Read the x, y, and z data from the magnetometer
	uint8_t byteData[12] = {0};
	UM7_Read_Data(hspi, &byteData[0], 4, DREG_MAG_RAW_Z);
	
	// Reassemble the data into floating point values
	data[0] = (byteData[3] << 8) |	(byteData[2] << 0);
	
	
	return;
}


void get_gyr_offsets(SPI_HandleTypeDef *hspi, uint8_t *data)
{
	// Read the x, y, and z offset values for the gyroscope
	UM7_Read_Data(hspi, &data[0], 4,  CREG_GYRO_TRIM_X);
	
	UM7_Read_Data(hspi, &data[4], 4,  CREG_GYRO_TRIM_Y);
	
	UM7_Read_Data(hspi, &data[8], 4,  CREG_GYRO_TRIM_Z);
	
	return;
}

void get_mag_offsets(SPI_HandleTypeDef *hspi, uint8_t *data)
{
	// Read the x, y, and z offset values for the magnetometer
	UM7_Read_Data(hspi, &data[0], 4,  CREG_MAG_BIAS_X);
	
	UM7_Read_Data(hspi, &data[4], 4,  CREG_MAG_BIAS_Y);
	
	UM7_Read_Data(hspi, &data[8], 4,  CREG_MAG_BIAS_Z);
	
	return;
}
