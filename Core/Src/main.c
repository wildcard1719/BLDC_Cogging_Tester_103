/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_ENCODER 0x36
#define ADDR_REG_ANGLE_H 0x0C
#define ADDR_REG_ANGLE_L 0x0D

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t T = 0;
uint16_t T_prev = 0;
uint16_t dt = 0;

uint16_t T_uart = 0;
uint16_t Period_uart = 2000;  // [us]

uint32_t T_cmd = 0;
uint32_t Period_cmd = 250000;  // [us]


uint16_t encoder_T = 0;
uint16_t encoder_T_prev = 0;
uint16_t encoder_dt = 0;

const uint16_t encoder_offset = 410;
const uint8_t right_step = 180;

uint8_t i2c1_buf_H = 0;
uint8_t i2c1_buf_L = 0;
uint16_t encoder_value = 0;
uint16_t encoder_value_prev = 0;
int16_t encoder_speed = 0;
int16_t encoder_speed_prev = 0;
int16_t encoder_acc = 0;
uint16_t encoder_value_filtered = 0;
int16_t encoder_speed_filtered = 0;
double encoder_speed_filtered_dt = 0;
double encoder_speed_filtered_dt_prev = 0;
double encoder_accel_filtered_dt = 0;

int16_t encoder_value_multi_turn = 0;
int16_t encoder_turn = 0;

volatile uint8_t i2c1_go = 1;
volatile uint8_t i2c1_proc = 0;
uint8_t i2c1_err = 0;

volatile uint8_t uart1_go = 1;

volatile uint8_t adc1_go = 1;
volatile uint8_t adc1_err = 0;
volatile uint16_t current_value = 0;
double current_value_lpf = 0;
double current_offset = 100;

double lpf_current_alpha = 1;
double lpf_current_output_prev = 0;

double lpf_speed_alpha = 0.05;
double lpf_speed_output_prev = 0;



//////////  PID GAIN  //////////
float motor_current_gain_P = 0.2;
float motor_current_gain_I = 0;
float motor_speed_gain_P = 32000;
float motor_speed_gain_I = 1.4;
float motor_speed_gain_A = 0;
float motor_position_gain_P = 0.00003;
float motor_position_gain_I = 0;
float motor_position_gain_D = 0.002;
////////////////////////////////

uint16_t motor_position = 0;
uint16_t motor_position_prev = 0;
double motor_speed = 0;
double motor_speed_prev = 0;
double motor_speed_filtered = 0;
double motor_acc = 0;
double motor_current = 0;

double motor_current_cmd = 0;
double motor_current_error = 0;
double motor_current_term_P = 0;
double motor_current_term_I = 0;
double motor_current_term_I_max = 6000;
double motor_current_control_out = 0;

double motor_speed_cmd = 0;
double motor_speed_cmd_prev = 0;
double motor_accel_cmd;
double motor_accel_error;
double motor_speed_error = 0;
double motor_speed_term_P = 0;
double motor_speed_term_I = 0;
double motor_speed_term_I_max = 2000;
double motor_speed_term_A = 0;
double motor_speed_control_out = 0;
int8_t motor_speed_control_dir = 1;


uint16_t motor_position_cmd = 0;
int16_t motor_position_error = 0;
int16_t motor_position_error_prev = 0;
double motor_position_term_P = 0;
double motor_position_term_I = 0;
double motor_position_term_I_max = 1000;
double motor_position_term_D = 0;
double motor_position_control_out = 0;

int8_t motor_position_error_sign = 1;
int8_t motor_position_error_sign_prev = 1;

uint16_t motor_speed_plot = 0;
uint16_t motor_speed_cmd_plot = 0;
uint16_t motor_current_plot = 0;
uint16_t motor_current_cmd_plot = 0;





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
    	i2c1_go = 1;
    	if(i2c1_proc == 1){ i2c1_proc = 2; }
    	if(i2c1_proc == 3){
    		i2c1_proc = 0;
    	}
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		uart1_go = 1;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(hadc->Instance == ADC1){
		adc1_go = 1;
		current_value = HAL_ADC_GetValue(hadc);
	}
}

void get_accel_cmd(){
	motor_accel_cmd = (double)motor_speed_cmd - (double)motor_speed_cmd_prev;
	motor_speed_cmd_prev = motor_speed_cmd;

	motor_speed_cmd_prev = motor_speed_cmd;
	motor_accel_error = motor_accel_cmd - encoder_accel_filtered_dt;
}


double lpf_current(double input, double alpha){
	double output = (alpha*input) + ((1.0-alpha)*lpf_current_output_prev);
	lpf_current_output_prev = output;
	return output;
}

double lpf_speed(double input, double alpha){
	double output = (alpha*input) + ((1.0-alpha)*lpf_speed_output_prev);
	lpf_speed_output_prev = output;
	return output;
}


void dt_update(){
	T_prev = T;
	T = TIM1->CNT;
	dt = T - T_prev;
}

void encoder_dt_update(){
	encoder_T_prev = encoder_T;
	encoder_T = TIM1->CNT;
	encoder_dt = encoder_T - encoder_T_prev;
}


void encoder_read(){
	if(i2c1_proc == 0){
		encoder_dt_update();
		encoder_value_prev = encoder_value;
		encoder_value = ((i2c1_buf_H << 8) | i2c1_buf_L);
		encoder_speed_prev = encoder_speed;
		encoder_speed = encoder_value - encoder_value_prev;
		encoder_acc = encoder_speed - encoder_speed_prev;
		if(abs(encoder_acc) < 100){
			encoder_value_filtered = encoder_value;
			encoder_speed_filtered = encoder_speed;
			encoder_speed_filtered_dt = (double)encoder_speed_filtered / (double)encoder_dt;// * (double)1000.0;
		}
		//if(encoder_value > 3800 && encoder_value_prev < 200){ encoder_turn--; }
		//if(encoder_value < 200 && encoder_value_prev > 3800){ encoder_turn++; }
		//encoder_value_multi_turn = encoder_turn * 4096 + encoder_value_filtered;
		encoder_value_multi_turn = encoder_value_filtered;

		if(HAL_I2C_Mem_Read_IT(&hi2c1, (ADDR_ENCODER << 1), ADDR_REG_ANGLE_H, 1, &i2c1_buf_H, 1U) == HAL_OK){ i2c1_proc = 1; }
		else{ i2c1_err = 1; }
	}
	if(i2c1_proc == 2){
		if(HAL_I2C_Mem_Read_IT(&hi2c1, (ADDR_ENCODER << 1), ADDR_REG_ANGLE_L, 1, &i2c1_buf_L, 1U) == HAL_OK){ i2c1_proc = 3; }
		else{ i2c1_err = 1;}
	}
}


void spwm(uint16_t idx_1, uint8_t power){
	idx_1 %= 720;

	uint16_t idx_2 = idx_1 + 240;
	uint16_t idx_3 = idx_1 + 480;
	idx_2 %= 720;
	idx_3 %= 720;
	float u = ((float)spwm_arr[idx_1] * (float)power / 255.0);
	float v = ((float)spwm_arr[idx_2] * (float)power / 255.0);
	float w = ((float)spwm_arr[idx_3] * (float)power / 255.0);

	TIM2->CCR2 = (uint16_t)(u + 0.5f);
	TIM2->CCR3 = (uint16_t)(v + 0.5f);
	TIM2->CCR4 = (uint16_t)(w + 0.5f);
}


void torque(int16_t v){
	if(v > 255){ v = 255; }
	if(v < -255){ v = -255; }

	uint16_t encoder = encoder_value_filtered + encoder_offset;
	encoder %= (uint16_t)(4096.0/7.0);
	int16_t accelation = (int16_t)((float)encoder * 720.0 / 585.0);
	if(v >= 0){ accelation += 180; }
	else { accelation -= 180; }
	if(accelation < 0){ accelation += 720; }
	accelation %= 720;
	spwm(accelation, abs(v));
}


void motor_pid(){
	motor_position_prev = motor_position;
	motor_speed_prev = motor_speed;
	motor_position = encoder_value_filtered;
	motor_speed = lpf_speed((double)(encoder_speed_filtered_dt), lpf_speed_alpha);


	motor_position_error_prev = motor_position_error;
	motor_position_error = motor_position_cmd - motor_position;
	motor_position_term_P = motor_position_error * motor_position_gain_P;
	motor_position_term_I += motor_position_error * motor_position_gain_I * (float)dt;
	if(motor_position_term_I > motor_position_term_I_max){ motor_position_term_I = motor_position_term_I_max; }
	if(motor_position_term_I < -motor_position_term_I_max){ motor_position_term_I = -motor_position_term_I_max; }
	motor_position_term_D = -motor_speed * motor_position_gain_D ;
	motor_position_control_out = motor_position_term_P + motor_position_term_I + motor_position_term_D;
	motor_speed_cmd = motor_position_control_out;

	motor_position_error_sign_prev = motor_position_error_sign;
	if(motor_position_error > 0){ motor_position_error_sign = 1; }
	if(motor_position_error < 0){ motor_position_error_sign = -1; }



	motor_speed_error = motor_speed_cmd - motor_speed;
	motor_speed_term_P = motor_speed_error  * motor_speed_gain_P;

	motor_speed_term_I += motor_speed_error * motor_speed_gain_I * (float)dt;

	if(motor_speed_term_I > motor_speed_term_I_max){ motor_speed_term_I = motor_speed_term_I_max; }
	if(motor_speed_term_I < -motor_speed_term_I_max){ motor_speed_term_I = -motor_speed_term_I_max; }

	motor_speed_control_out = motor_speed_term_P + motor_speed_term_I;
	if (motor_speed_control_out >= 0){ motor_speed_control_dir = 1; }
	else { motor_speed_control_dir = -1; }
	motor_current_cmd = abs((int16_t)motor_speed_control_out);

	motor_current = current_value_lpf;
	motor_current_error = (motor_current_cmd - motor_current);
	motor_current_term_P = motor_current_error * motor_current_gain_P;
	motor_current_term_I += motor_current_error * motor_current_gain_I * (float)dt;
	if(motor_current_term_I > motor_current_term_I_max){ motor_current_term_I = motor_current_term_I_max; }
	if(motor_current_term_I < -motor_current_term_I_max){ motor_current_term_I = -motor_current_term_I_max; }
	motor_current_control_out = motor_current_term_P + motor_current_term_I;
	if(motor_current_control_out < 0){ motor_current_control_out = 0; }

	motor_speed_plot = (uint16_t)(motor_speed * 60000);
	motor_speed_cmd_plot = (uint16_t)(motor_speed_cmd * 60000);
	motor_current_plot = (uint16_t)motor_current;
	motor_current_cmd_plot = (uint16_t)motor_current_cmd;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_ADC_Start_IT(&hadc1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  dt_update();
	  motor_pid();
	  torque((int16_t)motor_current_control_out * motor_speed_control_dir);
	  if (i2c1_go == 1){ i2c1_go = 0; encoder_read(); }
	  if (i2c1_err == 1){ i2c1_err = 0; encoder_read(); }
	  if (adc1_go == 1){
		  adc1_go = 0;
		  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK){ adc1_err = 1; }
	  }


	  current_value_lpf = lpf_current((double)(current_value - current_offset), lpf_current_alpha);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  T_cmd += dt;
	  if(T_cmd >= Period_cmd){
		  T_cmd -= Period_cmd;

		  if(motor_position_cmd == 1000){ motor_position_cmd = 1500; }
		  else if(motor_position_cmd == 1500){ motor_position_cmd = 900; }
		  else if(motor_position_cmd == 900){ motor_position_cmd = 3000; }
		  else if(motor_position_cmd == 3000){ motor_position_cmd = 2500; }
		  else if(motor_position_cmd == 2500){ motor_position_cmd = 3100; }
		  else if(motor_position_cmd == 3100){ motor_position_cmd = 1000; }
		  else{ motor_position_cmd = 1000; }
/*
		  if(motor_speed_cmd == 0){ motor_speed_cmd = 0.03; }
		  else if(motor_speed_cmd == 0.03){ motor_speed_cmd = 0.0005; }
		  else if(motor_speed_cmd == 0.0005){ motor_speed_cmd = 0; }
		  else{ motor_speed_cmd = 0; }*/
	  }

	  T_uart += dt;
	  if(T_uart >= Period_uart){
		  T_uart -= Period_uart;
		  if(uart1_go == 1){
			  uart1_go = 0;
			  uint8_t buf[50] = { '\0', };
			  uint8_t str_1[5]; uint8_t str_2[5]; uint8_t str_3[5]; uint8_t str_4[5];
			  int16_t plot_min = -10; int16_t plot_max = 10; uint8_t str_eof[2] = "\r\n";
/*
			  			  sprintf(buf, "%d,%d,%d,%d\r\n", plot_min, plot_max, motor_speed_plot, motor_speed_cmd_plot);


			  sprintf(buf, "%d,%d,%d,%d\r\n", motor_speed_plot, motor_speed_cmd_plot,
			  					  motor_current_plot, motor_current_cmd_plot);
*/

			  sprintf(buf, "%d,%d,%d,%d,%d,%d\r\n", motor_speed_plot, motor_speed_cmd_plot,
			  					  motor_current_plot, motor_current_cmd_plot
			  					  ,motor_position, motor_position_cmd);

			  HAL_UART_Transmit_IT(&huart1, buf, 50);
	  	}
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
