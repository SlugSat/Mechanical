#include "BNO055_IMU.h"

// broken at the moment but not super important for prototyping
uint8_t built_in_self_test(I2C_HandleTypeDef *hi2c)
{
	IMU_Reg_t reg = OPR_MODE_ADDR;
	IMU_Op_Mode_t mode = OPERATION_MODE_CONFIG;
	
	// set mode to CONFIG_MODE
	write_byte(hi2c, reg, mode);
	
	// set self test bit in system trigger register
	reg = SYS_TRIGGER_ADDR;
	write_byte(hi2c, reg, 0x01);
	
	// read from self test register to get result
	reg = SELFTEST_RESULT_ADDR;
	uint8_t readTest =  read_byte(hi2c, &reg);
	
	// check if test worked
	if (readTest == 0x0F)
	{
		return TRUE;
	}
	return FALSE;	
}

// IMU initialization
void IMU_init(I2C_HandleTypeDef *hi2c, IMU_Op_Mode_t sensor_mode)
{
	// first check if device is ready and exists
	while (HAL_I2C_IsDeviceReady(hi2c, IMU_ADDRESS_ALT, 2, 10) != HAL_OK);
	
	// set to config mode in case the device is not in default config mode
	set_mode(hi2c, OPERATION_MODE_CONFIG);
	
	// reset IMU
	write_byte(hi2c, SYS_TRIGGER_ADDR, 0x20);

	// poll device and wait until it is ready again
	while (HAL_I2C_IsDeviceReady(hi2c, IMU_ADDRESS_ALT, 2, 10) != HAL_OK);
	// toggle on board LED to indicate successful reset
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // turn on
	
	HAL_Delay(100);
	
	// get power on self test results
	IMU_Reg_t self_test_reg = SELFTEST_RESULT_ADDR; 
	uint8_t self_test_result = read_byte(hi2c, &self_test_reg);
	
	self_test_result &= 0x0F;
	if (self_test_result == 0x0F)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Turn off
	}
	
	// set to normal power mode
	write_byte(hi2c, PWR_MODE_ADDR, POWER_MODE_NORMAL);
	HAL_Delay(10);
	
	// re-init sys_trigger reg to 0
	write_byte(hi2c, SYS_TRIGGER_ADDR, 0);
	
	HAL_Delay(100);
	
	// set fusion mode and wait until fully calibrated
	set_mode(hi2c, OPERATION_MODE_NDOF_FMC_OFF); // set temporary fusion mode
	while(!is_calibrated(hi2c)); // wait until calibrated
	
	set_mode(hi2c, OPERATION_MODE_CONFIG); // set CONFIG mode
		
	HAL_Delay(50);
	
	set_mode(hi2c, sensor_mode); // set sensor mode

}

void get_calib_status(I2C_HandleTypeDef *hi2c, uint8_t *sys, uint8_t *gyro, uint8_t *acc, uint8_t *mag)
{
	// set register to read from as the calibration status address
	IMU_Reg_t calibration_status_reg = CALIB_STAT_ADDR;
	
	// read from the address and save value
	uint8_t calib_stat = read_byte(hi2c, &calibration_status_reg);
	
	// set the appropriate nibbles as the calibration
	// status of the given sensor and the system overall
	if (sys != NULL) {
    *sys = (calib_stat >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calib_stat >> 4) & 0x03;
  }
  if (acc != NULL) {
    *acc = (calib_stat >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calib_stat & 0x03;
	}
}

uint8_t is_calibrated(I2C_HandleTypeDef *hi2c)
{
	// initialize calibration status nibbles
	uint8_t sys, gyr, acc, mag;
	sys = acc = gyr = mag = 0;
	
	// get calibration status nibbles
	get_calib_status(hi2c, &sys, &acc, &gyr, &mag);
	
	// check if the system is calibrated
	if (sys < 3 || gyr < 3 || acc < 3 || mag < 3)
	{
		return FALSE;
	}
	return TRUE;
}


// set mode
void set_mode(I2C_HandleTypeDef *hi2c, IMU_Op_Mode_t op_mode)
{
	IMU_Reg_t reg = OPR_MODE_ADDR;
		
	// set mode to CONFIG_MODE
	write_byte(hi2c, reg, op_mode);
	
	return;
}

// Get Sensor Data
// get magnetometer data (microteslas)
void get_mag_data(I2C_HandleTypeDef *hi2c, float *data)
{
	IMU_Reg_t reg = MAG_DATA_X_LSB_ADDR;
	
	uint8_t rx_data[RCV_DATA_LEN] = {0};
	int16_t raw_data[RAW_DATA_LEN] = {0};
	
	read_bytes(hi2c, reg, rx_data, RCV_DATA_LEN);
	
	raw_data[0] = ((int16_t)rx_data[1] << 8) | (int16_t)rx_data[0];
	raw_data[1] = ((int16_t)rx_data[3] << 8) | (int16_t)rx_data[2];
	raw_data[2] = ((int16_t)rx_data[5] << 8) | (int16_t)rx_data[4];
	
	for (int i = 0; i < RAW_DATA_LEN; i++)
	{
		data[i] = (float)raw_data[i]/16.0;
	}

	return;
}

// get gyroscope data
void get_gyr_data(I2C_HandleTypeDef *hi2c, float *data)
{
	IMU_Reg_t reg = GYRO_DATA_X_LSB_ADDR;
	
	uint8_t rx_data[RCV_DATA_LEN] = {0};
	int16_t raw_data[RAW_DATA_LEN] = {0};
	
	read_bytes(hi2c, reg, rx_data, RCV_DATA_LEN);
	
	raw_data[0] = ((int16_t)rx_data[1] << 8) | (int16_t)rx_data[0];
	raw_data[1] = ((int16_t)rx_data[3] << 8) | (int16_t)rx_data[2];
	raw_data[2] = ((int16_t)rx_data[5] << 8) | (int16_t)rx_data[4];
	
	for (int i = 0; i < RAW_DATA_LEN; i++)
	{
		data[i] = (float)raw_data[i]/16.0;
	}

	return;
}

// get accelerometer data
void get_acc_data(I2C_HandleTypeDef *hi2c, float *data)
{
	// set register appropriately
	IMU_Reg_t reg = ACCEL_DATA_X_LSB_ADDR;
	
	// initialize necessary data arrays
	uint8_t rx_data[RCV_DATA_LEN] = {0}; // recieve data from registers
	int16_t raw_data[RAW_DATA_LEN] = {0}; // concatenated raw data
	
	read_bytes(hi2c, reg, rx_data, RCV_DATA_LEN);
	
	raw_data[0] = ((int16_t)rx_data[1] << 8) | (int16_t)rx_data[0];
	raw_data[1] = ((int16_t)rx_data[3] << 8) | (int16_t)rx_data[2];
	raw_data[2] = ((int16_t)rx_data[5] << 8) | (int16_t)rx_data[4];
	
	// convert raw data into units of m/s^2
	for (int i = 0; i < RAW_DATA_LEN; i++)
	{
		data[i] = (float)raw_data[i]/100.0;
	}

	return;
}

void get_gyr_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data)
{
	// set register appropriately
	IMU_Reg_t reg = ACCEL_OFFSET_X_LSB_ADDR;
	
	// read offsets
	read_bytes(hi2c, reg, data, RCV_DATA_LEN);
	
	return;
}

void get_mag_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data)
{
	// set register appropriately
	IMU_Reg_t reg = MAG_OFFSET_X_LSB_ADDR;
	
	// read offsets
	read_bytes(hi2c, reg, data, RCV_DATA_LEN);
	
	return;
}

void get_MG_offsets(I2C_HandleTypeDef *hi2c, uint8_t *gyr_offset, uint8_t *mag_offset)
{
	get_gyr_offsets(hi2c, gyr_offset);
	get_mag_offsets(hi2c, mag_offset);
	return;
}

void set_gyr_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data)
{
	IMU_Reg_t reg;
	
	for (int i = 0, reg = GYRO_OFFSET_X_LSB_ADDR; i < RCV_DATA_LEN; i++, reg++)
	{
		write_byte(hi2c, reg, data[i]);
	}
	return;
}

void set_mag_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data)
{
	IMU_Reg_t reg;
	
	for (int i = 0, reg = MAG_OFFSET_X_LSB_ADDR; i < RCV_DATA_LEN; i++, reg++)
	{
		write_byte(hi2c, reg, data[i]);
	}
	return;
}


//=================================================================
// 										I2C Communication Methods
//-----------------------------------------------------------------
//=================================================================
//													WRITE BYTE
//=================================================================
void write_byte(I2C_HandleTypeDef *hi2c, IMU_Reg_t reg, uint8_t dat) 
{
	// tx data array with register address to write to and data to write
	uint8_t tx_data[2] = {(uint8_t)reg, dat};

	// transmit the register address and data
	HAL_I2C_Master_Transmit(hi2c, IMU_ADDRESS_ALT, tx_data, sizeof(tx_data), 10);	HAL_Delay(30);
	return;
}

//=================================================================
//													READ BYTE
//=================================================================
uint8_t read_byte(I2C_HandleTypeDef *hi2c, IMU_Reg_t *reg)
{
	// transmit register address to read from
	uint8_t rx_data = 0;
	HAL_I2C_Master_Transmit(hi2c, IMU_ADDRESS_ALT, (uint8_t *) reg, sizeof(uint8_t), 10);
		
	// read data from register
	HAL_I2C_Master_Receive(hi2c, IMU_ADDRESS_ALT, &rx_data, sizeof(rx_data), 10);
	
	return rx_data;
}
//=================================================================


//=================================================================
//													READ BYTES
//=================================================================
void read_bytes(I2C_HandleTypeDef *hi2c, IMU_Reg_t reg, uint8_t *rx_data, uint16_t len)
{
	// transmit register address to read from
	HAL_I2C_Master_Transmit(hi2c, IMU_ADDRESS_ALT, (uint8_t*)&reg, sizeof(uint8_t), 10);
	
	// read data from registers
	HAL_I2C_Master_Receive(hi2c, IMU_ADDRESS_ALT, rx_data, len, 10);
	HAL_Delay(1);
	return;
}
//=================================================================
//-----------------------------------------------------------------
