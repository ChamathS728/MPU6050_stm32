/*
 * mpu6050.c
 * MPU6050 Accelerometer and Gyroscope Driver over I2C
 *  Created on: Nov 1, 2023
 *      Author: Chamath Suraweera
 *
 * Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * Register map: https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 */

#include "mpu6050.h"


/* Initialisation Functions */
HAL_StatusTypeDef MPU6050_wakeup(MPU6050* mpu6050) {
  // Write 0's to the PWR_MGMT_1 register to wake it up
  // It sets clock source as internal 8MHz clock, and it is woken up
  // uint8_t zero = 0;
  // HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &zero, sizeof(zero), TIMEOUT_DEFAULT);

  HAL_StatusTypeDef res = MPU6050_writeRegister(mpu6050, PWR_MGMT_1, 0);
  return res;
}

HAL_StatusTypeDef MPU6050_set_pwr_mgmt(MPU6050* mpu6050, int dev_rst, int sleep, int cycle, int temp_dis, int clksel) {
  // Assumption is that the first 4 inputs are 1 bit (0 or 1), and clksel is 3 bit (0 to 7)
  assert(dev_rst >= 0 && dev_rst <= 1);
  assert(sleep >= 0 && sleep <= 1);
  assert(cycle >= 0 && cycle <= 1);
  assert(temp_dis >= 0 && temp_dis <= 1);
  assert(clksel >= 0 && clksel <= 7);

  // Create bit string to put into the register
  uint8_t res =  0 | (dev_rst << 7) | (sleep << 6) | (cycle << 5) | (temp_dis << 3) | clksel;

  // Write to the register over I2C
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &res, sizeof(res), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, PWR_MGMT_1, &res);

  return result;
}

HAL_StatusTypeDef MPU6050_set_dlpf(uint8_t* dlpf, MPU6050* mpu6050) {
  /*
  Just set DLPF_CFG = 001 (so register )
  */

  // Ensure that dlpf is a number between 0 and 7
  assert((*dlpf >= 0) && (*dlpf <= 7));
  // assert(*dlpf >= 0);
  // assert(*dlpf <= 7);

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, DLPF, I2C_MEMADD_SIZE_8BIT, dlpf, sizeof(*dlpf), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, PWR_MGMT_1, dlpf);


  // Store the dlpf number in the struct
  mpu6050->dlpf = *dlpf;
  return result;
}

// REVIEW - Maths behind the smplrt_div_input may not be good due to division.
HAL_StatusTypeDef MPU6050_set_sample_rate(uint8_t* freq_ptr, MPU6050* mpu6050) {
  /*
  When DLPF_CFG = 0 or 7, gyro output rate = 8kHz
    otherwise, gyro output rate = 1kHz

   Accelerometer sample rate is fixed at 1kHz always

  freq_ptr is a pointer to the sample rate freq in kHz as uint8

  We need to calculate what needs to go into the SMPLRT_DIV register to get the desired frequency
  This also assumes that the gyro output rate is at 1kHz
  */

  uint8_t smplrt_div_input = (mpu6050->gyro_smplrt)/(*freq_ptr) - 1;

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &smplrt_div_input, sizeof(smplrt_div_input), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, SMPLRT_DIV, &smplrt_div_input);

  // Store the new sample rate divider in the struct
  mpu6050->gyro_smplrt = *freq_ptr;
  return result;
}

// REVIEW - Consider writing it so that we pull the current data in the register, AND it with 1110 0111, then OR it with (number << 3) so that we keep self-test
HAL_StatusTypeDef MPU6050_set_gyro_FSR(Gyro_FSR_SEL_TypeDef setting, MPU6050* mpu6050) {
  uint8_t pData;

  switch (setting) {
    // For each possibility, set the correct data to be written and store the setting away
    // Left shift data bits by 3 so that we only overwrite bits 4 and 3
    case GYRO_FSR_250:
      pData = (0 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_250;
      break;
    case GYRO_FSR_500:
      pData = (1 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_500;
      break;
    case GYRO_FSR_1000:
      pData = (2 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_1000;
      break;
    case GYRO_FSR_2000:
      pData = (3 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_2000;
      break;
  }

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &pData, sizeof(pData), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, GYRO_CONFIG, &pData);

  return result;
}

// REVIEW - Consider writing it so that we pull the current data in the register, AND it with 1110 0111, then OR it with (number << 3) so that we keep self-test
HAL_StatusTypeDef MPU6050_set_accel_FSR(Accel_FSR_SEL_TypeDef setting, MPU6050* mpu6050) {
  uint8_t pData;

  switch (setting) {
    case ACCEL_FSR_2g:
      pData = (0 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_2g;
      break;
    case ACCEL_FSR_4g:
      pData = (1 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_4g;
      break;
    case ACCEL_FSR_8g:
      pData = (2 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_8g;
      break;
    case ACCEL_FSR_16g:
      pData = (3 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_16g;
      break;
  }

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &pData, sizeof(pData), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, ACCEL_CONFIG, &pData);

  return result;
}


HAL_StatusTypeDef MPU6050_FIFO_enable(MPU6050* mpu6050) {
  // Write a 1 to bit 6 of register 0x6A
  uint8_t res = (1 << 6);

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &res, sizeof(res), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, USER_CTRL, &res);
  return result;
}

HAL_StatusTypeDef MPU6050_FIFO_reset(MPU6050* mpu6050) {
  // Assumes that MPU6050 FIFO buffer is enabled. FIFO enable bit is driven low ONLY when MPU is power cycled (turned off and back on)
  // Write a 2 to bit 2 of register 0x6A -> this gets written to 0 once FIFO is reset anyway
  uint8_t res = (1 << 2);

  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &res, sizeof(res), TIMEOUT_DEFAULT);
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, USER_CTRL, &res);

  return result;
}

HAL_StatusTypeDef MPU6050_init(MPU6050* mpu6050_ptr,I2C_HandleTypeDef* hi2c, uint8_t* dlpf, uint8_t* smpl_frq, Gyro_FSR_SEL_TypeDef gyro_setting, Accel_FSR_SEL_TypeDef accel_setting) {
  /*
  Pass in a pointer to an MPU6050 instance; it will initialise it for us

  */
    // Initialise what we can
    mpu6050_ptr->i2c_handle = hi2c;
    mpu6050_ptr->MPU6050_addr = MPU_ADDR;

    mpu6050_ptr->ax = 0.0;
    mpu6050_ptr->ay = 0.0;
    mpu6050_ptr->az = 0.0;

    mpu6050_ptr->gx = 0.0;
    mpu6050_ptr->gy = 0.0;
    mpu6050_ptr->gz = 0.0;

    mpu6050_ptr->temp_C = 0.0;

    mpu6050_ptr->ax_bias = 0.0;
    mpu6050_ptr->ay_bias = 0.0;
    mpu6050_ptr->az_bias = 0.0;

    mpu6050_ptr->gx_bias = 0.0;
    mpu6050_ptr->gy_bias = 0.0;
    mpu6050_ptr->gz_bias = 0.0;




    // Check that we have the correct device by checking its address
    uint8_t check = 0;
    HAL_StatusTypeDef result = MPU6050_readRegister(mpu6050_ptr, WHO_AM_I, &check);

    if (result == HAL_OK && check == 0x68) { //check == 0x68
        // Device is identified as the MPU6050 yay -> Wake it up
        MPU6050_wakeup(mpu6050_ptr);

        // Configure DLPF_CFG and store away settings
        MPU6050_set_dlpf(dlpf, mpu6050_ptr);

        // Configure the gyro sample rate and store away settings
        MPU6050_set_sample_rate(smpl_frq, mpu6050_ptr);

        // Set the full scale range for the gyroscope and accelerometer
        MPU6050_set_gyro_FSR(gyro_setting, mpu6050_ptr);
        MPU6050_set_accel_FSR(accel_setting, mpu6050_ptr);
    }

  return result;
}

/* Data Acquisition Functions */

HAL_StatusTypeDef MPU6050_read_gyro_reg(MPU6050* mpu6050) {
  // Initialise temporary variables
  int16_t gx_raw = 0;
  int16_t gy_raw = 0;
  int16_t gz_raw = 0;

  uint8_t num_of_bytes = 6;
  int8_t g_raw[num_of_bytes];

  // Read X gyro registers and store the raw gyroscope value away
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &g_raw_H, sizeof(g_raw_H), TIMEOUT_DEFAULT);

  // Read all gyroscope registers starting from GYRO_XOUT_H and ending at GYRO_ZOUT_L
  HAL_StatusTypeDef result = MPU6050_readRegisters(mpu6050, GYRO_XOUT_H, g_raw, num_of_bytes);

  // Get the raw x, y and z values
  gx_raw = (g_raw[0] << 8) | (g_raw[1]);
  gy_raw = (g_raw[2] << 8) | (g_raw[3]);
  gz_raw = (g_raw[4] << 8) | (g_raw[5]);

  // Rescale raw readings according to gyro FSR
  switch(mpu6050->gyro_FSR) {
    case GYRO_FSR_250:
      mpu6050->gx = ((float) gx_raw) / 131.0;
      mpu6050->gy = ((float) gy_raw) / 131.0;
      mpu6050->gz = ((float) gz_raw) / 131.0;
      break;
    case GYRO_FSR_500:
      mpu6050->gx = ((float) gx_raw) / 65.5;
      mpu6050->gy = ((float) gy_raw) / 65.5;
      mpu6050->gz = ((float) gz_raw) / 65.5;
      break;
    case GYRO_FSR_1000:
      mpu6050->gx = ((float) gx_raw) / 32.8;
      mpu6050->gy = ((float) gy_raw) / 32.8;
      mpu6050->gz = ((float) gz_raw) / 32.8;
      break;
    case GYRO_FSR_2000:
      mpu6050->gx = ((float) gx_raw) / 16.4;
      mpu6050->gy = ((float) gy_raw) / 16.4;
      mpu6050->gz = ((float) gz_raw) / 16.4;
      break;
  }

  return result;
}

HAL_StatusTypeDef MPU6050_read_accel_reg(MPU6050* mpu6050) {
  // Initialise temporary variables
  int16_t ax_raw = 0;
  int16_t ay_raw = 0;
  int16_t az_raw = 0;

  uint8_t num_of_bytes = 6;
  int8_t a_raw[num_of_bytes];

  // Read all gyroscope registers starting from GYRO_XOUT_H and ending at GYRO_ZOUT_L
  HAL_StatusTypeDef result = MPU6050_readRegisters(mpu6050, ACCEL_XOUT_H, a_raw, num_of_bytes);

  // Get the raw x, y and z values
  ax_raw = (a_raw[0] << 8) | (a_raw[1]);
  ay_raw = (a_raw[2] << 8) | (a_raw[3]);
  az_raw = (a_raw[4] << 8) | (a_raw[5]);

  // Rescale them according to accel FSR
  switch(mpu6050->accel_FSR) {
    case ACCEL_FSR_2g:
      mpu6050->ax = ((float) ax_raw) / 16384.0 * 9.8;
      mpu6050->ay = ((float) ay_raw) / 16384.0 * 9.8;
      mpu6050->az = ((float) az_raw) / 16384.0 * 9.8;
      break;
    case ACCEL_FSR_4g:
      mpu6050->ax = ((float) ax_raw) / 8192.0 * 9.8;
      mpu6050->ay = ((float) ay_raw) / 8192.0 * 9.8;
      mpu6050->az = ((float) az_raw) / 8192.0 * 9.8;
      break;
    case ACCEL_FSR_8g:
      mpu6050->ax = ((float) ax_raw) / 4096.0 * 9.8;
      mpu6050->ay = ((float) ay_raw) / 4096.0 * 9.8;
      mpu6050->az = ((float) az_raw) / 4096.0 * 9.8;
      break;
    case ACCEL_FSR_16g:
      mpu6050->ax = ((float) ax_raw) / 2048.0 * 9.8;
      mpu6050->ay = ((float) ay_raw) / 2048.0 * 9.8;
      mpu6050->az = ((float) az_raw) / 2048.0 * 9.8;
      break;
  }

  return result;
}

HAL_StatusTypeDef MPU6050_read_temp_reg(MPU6050* mpu6050) {
  int16_t raw_temp = 0;

  uint8_t num_of_bytes = 2;
  int8_t temp[2];

  HAL_StatusTypeDef result = MPU6050_readRegisters(mpu6050, TEMP_OUT_H, temp, num_of_bytes);
  raw_temp = (temp[0] << 8) | (temp[1]);

  // Fix the readings to be in celsius
  mpu6050->temp_C = ((float) raw_temp)/340.0 + 36.53;

  return HAL_OK;
}

uint8_t MPU6050_readGyro_DMA(MPU6050* mpu6050) {
  // Declare a static rxBuf made of uint8_t since it'll store register contents

  // Call HAL_I2C_Mem_Read_DMA with necessary values (check number of bytes!)
  
  return 1;
}

uint8_t MPU6050_readAccel_DMA(MPU6050* mpu6050) {
  // Declare a static rxBuf made of uint8_t since it'll store register contents

  // Call HAL_I2C_Mem_Read_DMA with necessary values (check number of bytes!)
    
  return 1;
}

uint8_t MPU6050_readTemp_DMA(MPU6050* mpu6050) {
  // Declare a static rxBuf made of uint8_t since it'll store register contents

  // Call HAL_I2C_Mem_Read_DMA with necessary values (check number of bytes!)
    
  return 1;
}

void MPU6050_readGyro_DMA_Complete(MPU6050* mpu6050) {
  // Take raw data from registers and make signed 16-bit integers

  // Scale it according to gyro_FSR

  // Store it away in the MPU6050 instance
  
}

void MPU6050_readAccel_DMA_Complete(MPU6050* mpu6050) {
  // Take raw data from registers and make signed 16-bit integers

  // Scale it according to accel_FSR + scale it to be in m/s^2

  // Store it away in the MPU6050 instance

}

void MPU6050_readTemp_DMA_Complete(MPU6050* mpu6050) {
  // Take raw data from registers and make signed 16-bit integers

  // Fix it to be in celsius

  // Store it away in the MPU6050 instance

}

// REVIEW - Consider rewriting to enable any of the bits to be switched
HAL_StatusTypeDef MPU6050_INT_enable(MPU6050* mpu6050) {
  // Write 0x01 to the INT_ENABLE register to enable the DATA_RDY bit -> only generate INT when data is in registers
  HAL_StatusTypeDef result = MPU6050_writeRegister(mpu6050, INT_ENABLE, 0x01);
  return result;
}



/* Low Level Functions */
HAL_StatusTypeDef MPU6050_readRegister(MPU6050* mpu6050, uint8_t reg, uint8_t* data) {
  /*
  Reads 1 byte from the specified register using I2C configuration from the MPU6050 instance
  Info is stored away into the data array
  NOTE - Pick the function
  */
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read_DMA(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1);
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read_IT(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1);
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1, HAL_MAX_DELAY);
  return result;
}

HAL_StatusTypeDef MPU6050_writeRegister(MPU6050* mpu6050, uint8_t reg, uint8_t* data) {
  /*
  Writes 1 byte from the data array into the specified register reg using I2C configuration from the MPU6050 instance
  */
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write_DMA(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1);
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Write_IT(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1);
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) 1, HAL_MAX_DELAY);
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_readRegisters(MPU6050* mpu6050, uint8_t reg, uint8_t* data, uint8_t length) {
  /*
  Reads 1 byte from the specified register using I2C configuration from the MPU6050 instance
  Info is stored away into the data array
  */
  // NOTE: Length is the number of bytes we wanna read from the register -> 1 means read that register. 2 would mean reading this register then the next one
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read_DMA(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) length);
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read_IT(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) length);
  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(mpu6050->i2c_handle, mpu6050->MPU6050_addr, reg, I2C_MEMADD_SIZE_8BIT, data, (uint16_t) length, HAL_MAX_DELAY);
  return HAL_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
  // Check if the pin that just went high was the interrupt pin

  // If it was, then call MPU6050_readGyro_DMA, MPU6050_readAccel_DMA and MPU6050_readTemp_DMA
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  // We have finished writing something via HAL_I2C_MemWrite -> Do something

  // Check that this handle matches our designated I2C bus (hi2c->Instance == I2C4)

  // If so, 
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  // We have finished receiving via HAL_I2C_MemRead -> Do something

  // Check that this handle matches our designated I2C bus (hi2c->Instance == I2C4)

  // If so, then call MPU6050_readGyro_DMA_Complete, MPU6050_readAccel_DMA_Complete and MPU6050_readTemp_DMA_Complete
}