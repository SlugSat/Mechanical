#ifndef BNO055_IMU_H
#define BNO055_IMU_H

#include "main.h"

// =================================================================
// pre-processor defines
// =================================================================

// true and false defines
#define TRUE 1
#define FALSE 0

// slave addresses for the BNO055 IMU
#define IMU_ADDRESS_DEF (0x29 << 1)
#define IMU_ADDRESS_ALT (0x28 << 1) // recognizable address by STM32

// size definitions
#define RCV_DATA_LEN 6
#define RAW_DATA_LEN 3
// =================================================================


// =================================================================
// type definitions for registers and special bytes
// =================================================================
typedef enum
{
	/* Page id register definition */
	PAGE_ID_ADDR                                     = 0x07,

	/* PAGE0 REGISTER DEFINITION START*/
	CHIP_ID_ADDR                                     = 0x00,
	ACCEL_REV_ID_ADDR                                = 0x01,
	MAG_REV_ID_ADDR                                  = 0x02,
	GYRO_REV_ID_ADDR                                 = 0x03,
	SW_REV_ID_LSB_ADDR                               = 0x04,
	SW_REV_ID_MSB_ADDR                               = 0x05,
	BL_REV_ID_ADDR                                   = 0X06,

	/* Accel data register */
	ACCEL_DATA_X_LSB_ADDR                            = 0x08,
	ACCEL_DATA_X_MSB_ADDR                            = 0x09,
	ACCEL_DATA_Y_LSB_ADDR                            = 0x0A,
	ACCEL_DATA_Y_MSB_ADDR                            = 0x0B,
	ACCEL_DATA_Z_LSB_ADDR                            = 0x0C,
	ACCEL_DATA_Z_MSB_ADDR                            = 0x0D,

	/* Mag data register */
	MAG_DATA_X_LSB_ADDR                              = 0x0E,
	MAG_DATA_X_MSB_ADDR                              = 0x0F,
	MAG_DATA_Y_LSB_ADDR                              = 0x10,
	MAG_DATA_Y_MSB_ADDR                              = 0x11,
	MAG_DATA_Z_LSB_ADDR                              = 0x12,
	MAG_DATA_Z_MSB_ADDR                              = 0x13,

	/* Gyro data registers */
	GYRO_DATA_X_LSB_ADDR                             = 0x14,
	GYRO_DATA_X_MSB_ADDR                             = 0x15,
	GYRO_DATA_Y_LSB_ADDR                             = 0x16,
	GYRO_DATA_Y_MSB_ADDR                             = 0x17,
	GYRO_DATA_Z_LSB_ADDR                             = 0x18,
	GYRO_DATA_Z_MSB_ADDR                             = 0x19,

	/* Euler data registers */
	EULER_H_LSB_ADDR                                 = 0x1A,
	EULER_H_MSB_ADDR                                 = 0x1B,
	EULER_R_LSB_ADDR                                 = 0x1C,
	EULER_R_MSB_ADDR                                 = 0x1D,
	EULER_P_LSB_ADDR                                 = 0x1E,
	EULER_P_MSB_ADDR                                 = 0x1F,

	/* Quaternion data registers */
	QUATERNION_DATA_W_LSB_ADDR                       = 0x20,
	QUATERNION_DATA_W_MSB_ADDR                       = 0x21,
	QUATERNION_DATA_X_LSB_ADDR                       = 0x22,
	QUATERNION_DATA_X_MSB_ADDR                       = 0x23,
	QUATERNION_DATA_Y_LSB_ADDR                       = 0x24,
	QUATERNION_DATA_Y_MSB_ADDR                       = 0x25,
	QUATERNION_DATA_Z_LSB_ADDR                       = 0x26,
	QUATERNION_DATA_Z_MSB_ADDR                       = 0x27,

	/* Linear acceleration data registers */
	LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0x28,
	LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0x29,
	LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0x2A,
	LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0x2B,
	LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0x2C,
	LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0x2D,

	/* Gravity data registers */
	GRAVITY_DATA_X_LSB_ADDR                          = 0x2E,
	GRAVITY_DATA_X_MSB_ADDR                          = 0x2F,
	GRAVITY_DATA_Y_LSB_ADDR                          = 0x30,
	GRAVITY_DATA_Y_MSB_ADDR                          = 0x31,
	GRAVITY_DATA_Z_LSB_ADDR                          = 0x32,
	GRAVITY_DATA_Z_MSB_ADDR                          = 0x33,

	/* Temperature data register */
	TEMP_ADDR                                        = 0x34,

	/* Status registers */
	CALIB_STAT_ADDR                                  = 0x35,
	SELFTEST_RESULT_ADDR                             = 0x36,
	INTR_STAT_ADDR                                   = 0x37,

	SYS_CLK_STAT_ADDR                                = 0x38,
	SYS_STAT_ADDR                                    = 0x39,
	SYS_ERR_ADDR                                     = 0x3A,

	/* Unit selection register */
	UNIT_SEL_ADDR                                    = 0x3B,
	DATA_SELECT_ADDR                                 = 0x3C,

	/* Mode registers */
	OPR_MODE_ADDR                                    = 0x3D,
	PWR_MODE_ADDR                                    = 0x3E,

	SYS_TRIGGER_ADDR                                 = 0x3F,
	TEMP_SOURCE_ADDR                                 = 0x40,

	/* Axis remap registers */
	AXIS_MAP_CONFIG_ADDR                             = 0x41,
	AXIS_MAP_SIGN_ADDR                               = 0x42,

	/* SIC registers */
	SIC_MATRIX_0_LSB_ADDR                            = 0x43,
	SIC_MATRIX_0_MSB_ADDR                            = 0x44,
	SIC_MATRIX_1_LSB_ADDR                            = 0x45,
	SIC_MATRIX_1_MSB_ADDR                            = 0x46,
	SIC_MATRIX_2_LSB_ADDR                            = 0x47,
	SIC_MATRIX_2_MSB_ADDR                            = 0x48,
	SIC_MATRIX_3_LSB_ADDR                            = 0x49,
	SIC_MATRIX_3_MSB_ADDR                            = 0x4A,
	SIC_MATRIX_4_LSB_ADDR                            = 0x4B,
	SIC_MATRIX_4_MSB_ADDR                            = 0x4C,
	SIC_MATRIX_5_LSB_ADDR                            = 0x4D,
	SIC_MATRIX_5_MSB_ADDR                            = 0x4E,
	SIC_MATRIX_6_LSB_ADDR                            = 0x4F,
	SIC_MATRIX_6_MSB_ADDR                            = 0x50,
	SIC_MATRIX_7_LSB_ADDR                            = 0x51,
	SIC_MATRIX_7_MSB_ADDR                            = 0x52,
	SIC_MATRIX_8_LSB_ADDR                            = 0x53,
	SIC_MATRIX_8_MSB_ADDR                            = 0x54,

	/* Accelerometer Offset registers */
	ACCEL_OFFSET_X_LSB_ADDR                                 = 0x55,
	ACCEL_OFFSET_X_MSB_ADDR                                 = 0x56,
	ACCEL_OFFSET_Y_LSB_ADDR                                 = 0x57,
	ACCEL_OFFSET_Y_MSB_ADDR                                 = 0x58,
	ACCEL_OFFSET_Z_LSB_ADDR                                 = 0x59,
	ACCEL_OFFSET_Z_MSB_ADDR                                 = 0x5A,

	/* Magnetometer Offset registers */
	MAG_OFFSET_X_LSB_ADDR                                   = 0x5B,
	MAG_OFFSET_X_MSB_ADDR                                   = 0x5C,
	MAG_OFFSET_Y_LSB_ADDR                                   = 0x5D,
	MAG_OFFSET_Y_MSB_ADDR                                   = 0x5E,
	MAG_OFFSET_Z_LSB_ADDR                                   = 0x5F,
	MAG_OFFSET_Z_MSB_ADDR                                   = 0x60,

	/* Gyroscope Offset register s*/
	GYRO_OFFSET_X_LSB_ADDR                                  = 0x61,
	GYRO_OFFSET_X_MSB_ADDR                                  = 0x62,
	GYRO_OFFSET_Y_LSB_ADDR                                  = 0x63,
	GYRO_OFFSET_Y_MSB_ADDR                                  = 0x64,
	GYRO_OFFSET_Z_LSB_ADDR                                  = 0x65,
	GYRO_OFFSET_Z_MSB_ADDR                                  = 0x66,

	/* Radius registers */
	ACCEL_RADIUS_LSB_ADDR                                   = 0x67,
	ACCEL_RADIUS_MSB_ADDR                                   = 0x68,
	MAG_RADIUS_LSB_ADDR                                     = 0x69,
	MAG_RADIUS_MSB_ADDR                                     = 0x6A

} IMU_Reg_t;

typedef enum
{
	POWER_MODE_NORMAL                                       = 0x00,
	POWER_MODE_LOWPOWER                                     = 0x01,
	POWER_MODE_SUSPEND                                      = 0x02
} IMU_Power_Mode_t;

typedef enum
{
	/* Operation mode settings*/
	OPERATION_MODE_CONFIG                                   = 0x00,
	OPERATION_MODE_ACCONLY                                  = 0x01,
	OPERATION_MODE_MAGONLY                                  = 0x02,
	OPERATION_MODE_GYRONLY                                  = 0x03,
	OPERATION_MODE_ACCMAG                                   = 0x04,
	OPERATION_MODE_ACCGYRO                                  = 0x05,
	OPERATION_MODE_MAGGYRO                                  = 0x06,
	OPERATION_MODE_AMG                                      = 0x07,
	OPERATION_MODE_IMUPLUS                                  = 0x08,
	OPERATION_MODE_COMPASS                                  = 0x09,
	OPERATION_MODE_M4G                                      = 0x0A,
	OPERATION_MODE_NDOF_FMC_OFF                             = 0x0B,
	OPERATION_MODE_NDOF                                     = 0x0C
} IMU_Op_Mode_t;
// =================================================================


// =================================================================
// Configuration and Initialization Methods
// =================================================================

// broken at the moment due to timing issues
uint8_t built_in_self_test(I2C_HandleTypeDef *hi2c);

// initialization method that checks self test register before
// changing to a sensor mode provided by user
void IMU_init(I2C_HandleTypeDef *hi2c, IMU_Op_Mode_t sensor_mode);

// Change IMU mode
// NOTE: must be first in config mode before changing to sensor
//			 output mode
void set_mode(I2C_HandleTypeDef *hi2c, IMU_Op_Mode_t op_mode);

// =================================================================


// =================================================================
// Accessors of sensor data methods
// =================================================================

// gets magnetometer data where data is a float array of size 3
// that represents the magnetic fields vector [micro-teslas]
void get_mag_data(I2C_HandleTypeDef *hi2c, float *data);

// gets gyroscope data where data is a float array of size 3
// that represents the rotation vector []
void get_gyr_data(I2C_HandleTypeDef *hi2c, float *data);

// gets accelerometer data where data is a float array of size 3
// that represents the acceleration vector [m/s^2]
void get_acc_data(I2C_HandleTypeDef *hi2c, float *data);

// =================================================================


// =================================================================
// Calibration methods
// NOTE: must be in config mode to read/set offset registers
// =================================================================

// Accessors for offsets of sensors
void get_MG_offsets(I2C_HandleTypeDef *hi2c, uint8_t *gyr_offset, uint8_t *mag_offset);
void get_gyr_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data);
void get_mag_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data);

// Mutators for offsets of sensors
void set_MG_offsets(I2C_HandleTypeDef *hi2c, uint8_t *gyr_offset, int16_t *mag_offset);
void set_gyr_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data);
void set_mag_offsets(I2C_HandleTypeDef *hi2c, uint8_t *data);

// Accessor for calibration status
uint8_t is_calibrated(I2C_HandleTypeDef *hi2c);
void get_calib_status(I2C_HandleTypeDef *hi2c, uint8_t *sys, uint8_t *gyro, uint8_t *acc, uint8_t *mag);

// =================================================================


// =================================================================
// I2C Communication Methods
// =================================================================

// Writes single byte to the specified register
void write_byte(I2C_HandleTypeDef *hi2c, IMU_Reg_t reg, uint8_t dat);

// Reads single byte from the specified register
uint8_t read_byte(I2C_HandleTypeDef *hi2c, IMU_Reg_t *reg);

// Reads multiple bytes starting from the specified register up to
// the registor of address (register_addr + len)
void read_bytes(I2C_HandleTypeDef *hi2c, IMU_Reg_t reg, uint8_t *rx_data, uint16_t len);

// =================================================================

#endif
