/*
 * mpu6050.h
 * MPU6050 Accelerometer and Gyroscope Driver over I2C
 *  Created on: Nov 1, 2023
 *      Author: Chamath Suraweera
 *
 * Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * Register map: https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 */

#ifndef MPU6050_I2C_DRIVER_H
#define MPU6050_I2C_DRIVER_H

#include "stm32h7xx_hal.h" // Need this for I2C communications via HAL

/* DEFINES */
#define MPU_ADDR       (0x68 << 1) // Datasheet says 0x68; left shift by 1 due to 7-bit addressing


/* REGISTERS: Page 3 */
#define SELF_TEST_X         0x0D
#define SELF_TEST_Y         0x0E
#define SELF_TEST_Z         0x0F
#define SELF_TEST_A         0x10
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define MOT_THR             0x1F
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1		    0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNT_H        0x72
#define FIFO_COUNT_L        0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

typedef enum {
  GYRO_FSR_250  = 0,
  GYRO_FSR_500  = 1,
  GYRO_FSR_1000 = 2,
  GYRO_FSR_2000 = 3
} Gyro_FSR_SEL_TypeDef;

typedef enum {
  ACCEL_FSR_2g  = 0,
  ACCEL_FSR_4g  = 1,
  ACCEL_FSR_8g  = 2,
  ACCEL_FSR_16g = 3
} Accel_FSR_SEL_TypeDef;

typedef struct MPU6050 {
  // Info used for I2C communication
  I2C_HandleTypeDef* i2c_handle;
  uint8_t MPU6050_addr;

  // Configuration information
  uint8_t gyro_smplrt; // 8kHz or 1kHz dependi ng on DLPF_CFG
  uint8_t dlpf;
  Gyro_FSR_SEL_TypeDef gyro_FSR; // Explicit number of the FSR (eg: 250 deg/s)
  Accel_FSR_SEL_TypeDef accel_FSR; // Just the coefficient (eg: accel_FSR = 2 means 2g = 2 x 9.8)

  // Actual data stored away
  float ax, ay, az; // 16 bit integer
  float gx, gy, gz; // 16 bit integer with 3 entries for x, y and z rotational velocities
  float temp_C; // 16 bit integer

  // Any data for calibration
  float ax_bias, ay_bias, az_bias;
  float gx_bias, gy_bias, gz_bias;

} MPU6050;

/* Function prototypes here */

/* Initialisation functions */
HAL_StatusTypeDef MPU6050_wakeup(MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_set_pwr_mgmt(MPU6050* mpu6050, int dev_rst, int sleep, int cycle, int temp_dis, int clksel);
HAL_StatusTypeDef MPU6050_set_dlpf(uint8_t* dlpf, MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_set_sample_rate(uint8_t* freq_ptr, MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_set_gyro_FSR(Gyro_FSR_SEL_TypeDef setting, MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_set_accel_FSR(Accel_FSR_SEL_TypeDef setting, MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_FIFO_enable(MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_FIFO_reset(MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_init(MPU6050* mpu6050_ptr,I2C_HandleTypeDef* hi2c, uint8_t* dlpf, uint8_t* smpl_frq, Gyro_FSR_SEL_TypeDef gyro_setting, Accel_FSR_SEL_TypeDef accel_setting);
HAL_StatusTypeDef MPU6050_INT_enable(MPU6050* mpu6050);

/* Data Acquisition functions */
HAL_StatusTypeDef MPU6050_read_gyro_reg(MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_read_accel_reg(MPU6050* mpu6050);
HAL_StatusTypeDef MPU6050_read_temp_reg(MPU6050* mpu6050);

uint8_t MPU6050_readGyro_DMA(MPU6050* mpu6050);
uint8_t MPU6050_readAccel_DMA(MPU6050* mpu6050);
uint8_t MPU6050_readTemp_DMA(MPU6050* mpu6050);

void MPU6050_readGyro_DMA_Complete(MPU6050* mpu6050);
void MPU6050_readAccel_DMA_Complete(MPU6050* mpu6050);
void MPU6050_readTemp_DMA_Complete(MPU6050* mpu6050);


/* Low-level functions */
HAL_StatusTypeDef MPU6050_readRegister(MPU6050* mpu6050, uint8_t reg, uint8_t* data);
  /*
  Reads 1 byte from the specified register using I2C configuration from the MPU6050 instance
  Info is stored away into the data array
  */


HAL_StatusTypeDef MPU6050_writeRegister(MPU6050* mpu6050, uint8_t reg, uint8_t* data);
  /*
  Writes 1 byte from the data array into the specified register reg using I2C configuration from the MPU6050 instance
  */


HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* mpu6050, uint8_t reg, uint8_t* data, uint8_t length);
  /*
  Reads 1 byte from the specified register using I2C configuration from the MPU6050 instance
  Info is stored away into the data array
  */
  // NOTE: Length is the number of bytes we wanna read from the register -> 1 means read that register. 2 would mean reading this register then the next one


#endif /* INC_MPU6050_H_ */
