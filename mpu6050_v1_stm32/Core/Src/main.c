/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT_DEFAULT 1000

// Linear acceleration registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

//Angular acceleration registers
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define WHO_AM_I		  0x75
#define MPU_ADDR     (0x68 << 1) // Datasheet says 0x68; left shift by 1 due to 7-bit addressing

#define PWR_MGMT_1		0x6B
#define SMPLRT_DIV		0x19 // Sample rate divider. Sample rate = GyroscopeOutputRate/(1 + SMPLRT_DIV)
                    		 // GyroscopeOutputRate = 8kHz if DLPF_CFG = 0 or 7 | 1kHz if DLPF_CFG is 1 to 6
#define CONFIG			  0x1A
#define GYRO_CONFIG		0x1B
#define ACCEL_CONFIG	0x1C
#define INT_ENABLE		0x38
#define DLPF          0x1A

#define FIFO_EN       0x23
#define USER_CTRL     0x6A

#define FIFO_COUNT_H  0x72
#define FIFO_COUNT_L  0x73
#define FIFO_R_W      0x74

#define FIFO_BRST_LEN 0x06
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Define any structs here:

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
  int16_t ax, ay, az; // 16 bit integer
  int16_t gx, gy, gz; // 16 bit integer with 3 entries for x, y and z rotational velocities
  int16_t temp; // 16 bit integer
  uint8_t gyro_smplrt; // 8kHz or 1kHz dependi ng on DLPF_CFG
  uint8_t dlpf;
  Gyro_FSR_SEL_TypeDef gyro_FSR; // Explicit number of the FSR (eg: 250 deg/s)
  Accel_FSR_SEL_TypeDef accel_FSR; // Just the coefficient (eg: accel_FSR = 2 means 2g = 2 x 9.8)
} MPU6050;


// NOTE - Instantiate one of each thing here


//typedef struct MPU6050 MPU6050;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef MPU6050_wakeup(void) {
  // Write 0's to the PWR_MGMT_1 register to wake it up
  // It sets clock source as internal 8MHz clock, and it is woken up
  uint8_t zero = 0;
  HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, PWR_MGMT_1, sizeof(PWR_MGMT_1), &zero, sizeof(zero), TIMEOUT_DEFAULT);

  return res;
}

HAL_StatusTypeDef MPU6050_set_pwr_mgmt(int dev_rst, int sleep, int cycle, int temp_dis, int clksel) {
  // Assumption is that the first 4 inputs are 1 bit (0 or 1), and clksel is 3 bit (0 to 7)
  assert(dev_rst >= 0 && dev_rst <= 1);
  assert(sleep >= 0 && sleep <= 1);
  assert(cycle >= 0 && cycle <= 1);
  assert(temp_dis >= 0 && temp_dis <= 1);
  assert(clksel >= 0 && clksel <= 7);

  // Create bit string to put into the register
  uint8_t res =  0 | (dev_rst << 7) | (sleep << 6) | (cycle << 5) | (temp_dis << 3) | clksel;
  
  // Write to the register over I2C
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, PWR_MGMT_1, sizeof(PWR_MGMT_1), &res, sizeof(res), TIMEOUT_DEFAULT);

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

  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, DLPF, sizeof(DLPF), dlpf, sizeof(*dlpf), TIMEOUT_DEFAULT);
  
  // Store the dlpf number in the struct
  mpu6050->dlpf = *dlpf;
  return result;
}

// REVIEW - Maths behind the smplrt_div_input may not be good due to division.
HAL_StatusTypeDef MPU6050_set_sample_rate(uint8_t* freq_ptr, MPU6050* mpu650_obj) {
  /*
  When DLPF_CFG = 0 or 7, gyro output rate = 8kHz  
    otherwise, gyro output rate = 1kHz

   Accelerometer sample rate is fixed at 1kHz always

  freq_ptr is a pointer to the sample rate freq in kHz as uint8

  We need to calculate what needs to go into the SMPLRT_DIV register to get the desired frequency
  This also assumes that the gyro output rate is at 1kHz
  */
  
  uint8_t smplrt_div_input = (mpu650_obj->gyro_smplrt)/(*freq_ptr) - 1;

  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, SMPLRT_DIV, sizeof(SMPLRT_DIV), &smplrt_div_input, sizeof(smplrt_div_input), TIMEOUT_DEFAULT);
  
  // Store the new sample rate divider in the struct
  mpu650_obj->gyro_smplrt = *freq_ptr;
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
    case GYRO_FSR_500:
      pData = (1 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_500;
    case GYRO_FSR_1000:
      pData = (2 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_1000;
    case GYRO_FSR_2000:
      pData = (3 << 3);
      mpu6050->gyro_FSR = GYRO_FSR_2000;
  }
  
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, GYRO_CONFIG, sizeof(GYRO_CONFIG), &pData, sizeof(pData), TIMEOUT_DEFAULT);
  return result;
}

// REVIEW - Consider writing it so that we pull the current data in the register, AND it with 1110 0111, then OR it with (number << 3) so that we keep self-test
HAL_StatusTypeDef MPU6050_set_accel_FSR(Accel_FSR_SEL_TypeDef setting, MPU6050* mpu6050) {
  uint8_t pData;

  switch (setting) {
    case ACCEL_FSR_2g:
      pData = (0 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_2g;
    case ACCEL_FSR_4g:
      pData = (1 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_4g;
    case ACCEL_FSR_8g:
      pData = (2 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_8g;
    case ACCEL_FSR_16g:
      pData = (3 << 3);
      mpu6050->accel_FSR = ACCEL_FSR_16g;
  }
  
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, ACCEL_CONFIG, sizeof(ACCEL_CONFIG), &pData, sizeof(pData), TIMEOUT_DEFAULT);
  return result;  
}


HAL_StatusTypeDef MPU6050_FIFO_enable(void) {
  // Write a 1 to bit 6 of register 0x6A
  uint8_t res = (1 << 6);

  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, USER_CTRL, sizeof(USER_CTRL), &res, sizeof(res), TIMEOUT_DEFAULT);
  return result;
}

HAL_StatusTypeDef MPU6050_FIFO_reset(void) {
  // Assumes that MPU6050 FIFO buffer is enabled. FIFO enable bit is driven low ONLY when MPU is power cycled (turned off and back on)
  // Write a 2 to bit 2 of register 0x6A -> this gets written to 0 once FIFO is reset anyway
  uint8_t res = (1 << 2);

  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, USER_CTRL, sizeof(USER_CTRL), &res, sizeof(res), TIMEOUT_DEFAULT);
  return result;  
}

HAL_StatusTypeDef MPU6050_init(MPU6050* mpu6050_ptr, uint8_t* dlpf, uint8_t* smpl_frq, Gyro_FSR_SEL_TypeDef gyro_setting, Accel_FSR_SEL_TypeDef accel_setting) {
  // Check that we have the correct device by checking its address
  uint8_t check;
  // uint8_t check[4];

  HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, WHO_AM_I, sizeof(WHO_AM_I), &check, sizeof(check), TIMEOUT_DEFAULT);
  // HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, WHO_AM_I, 1, check, 1, 1000);

  if (result == HAL_OK && check == 0xD0) { //check == 0x68
    // Device is identified as the MPU6050 yay -> Wake it up
    MPU6050_wakeup();

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

HAL_StatusTypeDef MPU6050_read_gyro(MPU6050* mpu6050) {
  // Set the FIFO ENABLE register so that we can read gyro 
  uint8_t res = (7 << 4); // Set bits 6,5,4 to 1 by shifting 7 = b111 by 4 bits to the left
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, FIFO_EN, sizeof(FIFO_EN), &res, sizeof(res), TIMEOUT_DEFAULT);
  
  // Check FIFO_COUNT to see if we have enough bytes to read (always read high register first then low)
  uint16_t fifo_count_H;
  uint16_t fifo_count_L;

  // Read high and low addresses then concatenate them together properly
  result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_COUNT_H, sizeof(FIFO_COUNT_H), &fifo_count_H, sizeof(fifo_count_H), TIMEOUT_DEFAULT);
  result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_COUNT_L, sizeof(FIFO_COUNT_L), &fifo_count_L, sizeof(fifo_count_L), TIMEOUT_DEFAULT);
    
  uint16_t fifo_count = (fifo_count_H << 8) | fifo_count_L;

  // if (fifo_count != FIFO_BRST_LEN) {
  //   return HAL_ERROR;
  // }

  // for loop 6 times to get readings from H and L registers for X Y and Z axes each
  uint8_t fifo_readings[FIFO_BRST_LEN];

  for (int i = 0; i < FIFO_BRST_LEN; ++i) {
    result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_R_W, sizeof(FIFO_R_W), fifo_readings + i, sizeof(*(fifo_readings + i)), TIMEOUT_DEFAULT);
  }

  // Now that we have all the readings, chuck them into the struct
  int16_t temp_gx = (int16_t) fifo_readings[0] << 8 | (int16_t) fifo_readings[1];
  int16_t temp_gy = (int16_t) fifo_readings[2] << 8 | (int16_t) fifo_readings[3];
  int16_t temp_gz = (int16_t) fifo_readings[4] << 8 | (int16_t) fifo_readings[5];

  // Rescale them according to gyro FSR
  switch(mpu6050->gyro_FSR) {
    case GYRO_FSR_250:
      mpu6050->gx = temp_gx / 131.0;
      mpu6050->gy = temp_gy / 131.0;
      mpu6050->gz = temp_gz / 131.0;
    case GYRO_FSR_500:
      mpu6050->gx = temp_gx / 65.5;
      mpu6050->gy = temp_gy / 65.5;
      mpu6050->gz = temp_gz / 65.5;
    case GYRO_FSR_1000:
      mpu6050->gx = temp_gx / 32.8;
      mpu6050->gy = temp_gy / 32.8;
      mpu6050->gz = temp_gz / 32.8;
    case GYRO_FSR_2000:
      mpu6050->gx = temp_gx / 16.4;
      mpu6050->gy = temp_gy / 16.4;
      mpu6050->gz = temp_gz / 16.4;
  }

  return result;
}

HAL_StatusTypeDef MPU6050_read_accel(MPU6050* mpu6050) {

  // Set the FIFO ENABLE register so that we can read accelerometer 
  uint8_t res = (1 << 3); // Set bit 3 to 1 by shifting 1 by 3 bits to the left
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, FIFO_EN, sizeof(FIFO_EN), &res, sizeof(res), TIMEOUT_DEFAULT);
  

  // for loop 6 times to get readings from H and L registers for X Y and Z axes each
  uint8_t fifo_readings[FIFO_BRST_LEN];

  for (int i = 0; i < FIFO_BRST_LEN; ++i) {
    result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_R_W, sizeof(FIFO_R_W), fifo_readings + i, sizeof(*(fifo_readings + i)), TIMEOUT_DEFAULT);
  }

  // Now that we have all the readings, chuck them into the struct
  int16_t temp_ax = (int16_t) fifo_readings[0] << 8 | (int16_t) fifo_readings[1];
  int16_t temp_ay = (int16_t) fifo_readings[2] << 8 | (int16_t) fifo_readings[3];
  int16_t temp_az = (int16_t) fifo_readings[4] << 8 | (int16_t) fifo_readings[5];

  // Rescale them according to gyro FSR
  switch(mpu6050->accel_FSR) {
    case ACCEL_FSR_2g:
      mpu6050->ax = temp_ax / 16384.0;
      mpu6050->ay = temp_ay / 16384.0;
      mpu6050->az = temp_az / 16384.0;
    case ACCEL_FSR_4g:
      mpu6050->ax = temp_ax / 8192.0;
      mpu6050->ay = temp_ay / 8192.0;
      mpu6050->az = temp_az / 8192.0;
    case ACCEL_FSR_8g:
      mpu6050->ax = temp_ax / 4096.0;
      mpu6050->ay = temp_ay / 4096.0;
      mpu6050->az = temp_az / 4096.0;
    case ACCEL_FSR_16g:
      mpu6050->ax = temp_ax / 2048.0;
      mpu6050->ay = temp_ay / 2048.0;
      mpu6050->az = temp_az / 2048.0;
  }

  return result;
}

HAL_StatusTypeDef MPU6050_read_temp(MPU6050* mpu6050) {
  // Set the FIFO ENABLE register so that we can read temperature 
  uint8_t res = (1 << 7); // Set bit 7 to 1 by shifting 1 by 7 bits to the left
  HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c4, MPU_ADDR, FIFO_EN, sizeof(FIFO_EN), &res, sizeof(res), TIMEOUT_DEFAULT);

  // Read buffer twice to get high and low registers
  uint16_t temp_high;
  uint16_t temp_low;

  result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_R_W, sizeof(FIFO_R_W), &temp_high, sizeof(temp_high), TIMEOUT_DEFAULT);
  result = HAL_I2C_Mem_Read(&hi2c4, MPU_ADDR, FIFO_R_W, sizeof(FIFO_R_W), &temp_low, sizeof(temp_low), TIMEOUT_DEFAULT);

  // Now that we have all the readings, chuck them into the struct
  int16_t temp = (int16_t) temp_high << 8 | (int16_t) temp_low;

  mpu6050->temp = temp / 340.0 + 36.53; 

  return result;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

  // Run the MPU init
  MPU6050 mpu6050;
  
  // Wake up the MPU6050 as well
  MPU6050_wakeup();

  // Initialise the MPU6050
  uint8_t dlpf = 1;
  uint8_t smplfrq = 1;
  MPU6050_init(&mpu6050, &dlpf, &smplfrq, GYRO_FSR_250, ACCEL_FSR_2g);
  MPU6050_set_pwr_mgmt(0, 0, 0, 0, 1);

  // Enable the FIFO buffer
  MPU6050_FIFO_enable();
  // Reset it as well so that it can be burst read or something -> https://stackoverflow.com/questions/60419390/mpu-6050-correctly-reading-data-from-the-fifo-register
  MPU6050_FIFO_reset();
  MPU6050_FIFO_enable();

  uint16_t gyro_buff[3];
  uint16_t accel_buff[3];
  uint16_t temp_buff;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read data
    MPU6050_read_gyro(&mpu6050);
    HAL_Delay(100); // Delays in between just for timing??
    MPU6050_read_accel(&mpu6050);
    HAL_Delay(100); // Delays in between just for timing??
    MPU6050_read_temp(&mpu6050);
    HAL_Delay(100); // Delays in between just for timing??
    

    // Stuff below here for printing to serial
    gyro_buff[0] = mpu6050.gx;
    gyro_buff[1] = mpu6050.gy;
    gyro_buff[2] = mpu6050.gz;

    accel_buff[0] = mpu6050.ax;
    accel_buff[1] = mpu6050.ay;
    accel_buff[2] = mpu6050.az;

    temp_buff = mpu6050.temp;    

    HAL_UART_Transmit(&huart3, gyro_buff, 3, TIMEOUT_DEFAULT);
    HAL_Delay(100); // Delays in between just for timing??
    HAL_UART_Transmit(&huart3, accel_buff, 3, TIMEOUT_DEFAULT);
    HAL_Delay(100); // Delays in between just for timing??
    HAL_UART_Transmit(&huart3, &temp_buff, 1, TIMEOUT_DEFAULT);
    HAL_Delay(100); // Delays in between just for timing??

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20B0CCFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
