/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "general.h"
//#include "retarget.h"
#include <fastmath.h>
#include <string.h>
#include <stdlib.h>
#include <lwip/udp.h>
#include <lwip/ip_addr.h>

#include "ssd1306.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct udp_pcb *upcb;
struct ip4_addr dst_addr;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for task2 */
osThreadId_t task2Handle;
const osThreadAttr_t task2_attributes = {
  .name = "task2",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024 * 4
};
/* Definitions for speed_control */
osThreadId_t speed_controlHandle;
const osThreadAttr_t speed_control_attributes = {
  .name = "speed_control",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 1024 * 4
};
/* USER CODE BEGIN PV */

const float adc_2_volt = 3.3 / 0xfff; // V/code
const float isense_gain = 1.0 / 0.066; // A/V
const float vsense_gain = (218. + 4710.) / 218.;

#define STATE_OFF 0
#define STATE_RUN 1
#define STATE_OC 2
#define STATE_OV 3

#define OC_LIM 10.0  // Amps

volatile struct data_struct_t {
	float adc1;
	float adc2;
	float adc3;
	float ia;		// Measured A Phase Current
	float ib;		// Measured B Phase Current
	float ic;		// Calculated C Phase Current
	float vbus;		// Measured Bus Voltage
	float ids;		// Current D Axis
	float iqs;		// Current Q Axis
	float vds;		// Voltage D Axis
	float vqs;		// Voltage Q Axis
	float flds;		// Flux Linkage D Axis
	float flqs;		// Flux Linkage Q Axis
	float fls;  	// Flux vector norm
	float flux_ang; // Aagle of the d&q flux vectors
	float rs;		// Stator Resistance
	float van;		// Voltage from A to Neutral
	float vbn;		// Voltage from B to Neutral
	float vcn;		// Voltage from C to Neutral
	float te; 		// Electromagnetic Torque
	float torque_goal;
	float flux_goal;
	float speed_rpm;
	float test;
	int run_state;	// Run Mode
	int flux_sector;// Sector from angle.
	int sw;			// Switch state (0-5)
} data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTask2(void *argument);
void SpeedControl(void *argument);

/* USER CODE BEGIN PFP */

void print_float(float val, char *buf) {

	char *val_sign = (val < 0) ? "-" : "";
	float abs_val = (val < 0) ? -val : val;

	int val_int = abs_val;                  // Get the integer (678).
	float val_frac = abs_val - abs_val;      // Get fraction (0.0123).
	int val_int2 = trunc(val_frac * 1000);  // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	snprintf(buf, 8, "%s%d.%01d", val_sign, val_int, val_int2);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const float one_over_sqrt3 = 1.0 / sqrtf(3);
const float one_over_three = 1.0 / 3.0;
const float two_over_three = 2.0 / 3.0;
const float rad_2_deg = 180.0 / M_PI;
const float dt = 1.0 / 80e3;
#define poles 8
const float Ke = 0.036;

float torque_hyst = 0.001;
float flux_hyst = 0.0005;

const float deg_per_sec_to_rpm = 60.0 / 360.0;

const uint16_t switch_vector_lookup[6][6] = {
		{2, 3, 4, 5, 6, 1},
		{0, 7, 0, 7, 0, 7},
		{6, 1, 2, 3, 4, 5},
		{3, 4, 5, 6, 1, 2},
		{7, 0, 7, 0, 7, 0},
		{5, 6, 1, 2, 3, 4},
};

#define SPEED_FILT_LEN 64
#define STEPS_PER_SPEED_UPDATE 8 // 10kHz
static float speed_filt_arr[SPEED_FILT_LEN];
static float speed_filt_sum = 0;
static int speed_filt_idx = 0;

void FastControl(void) {
	// Hysteresis state variables
	static int dflux = 1;
	static int dtorque = 1;
	static float last_ang = 0;
	static int steps_per_angle_update = 1;
	static int run_speed_calc = 0;
	static float torque_err_last = 0;

	// Neutral voltage
	float vn;

	DBG_0_ON();

	// https://www.mathworks.com/help/physmod/sps/powersys/ug/simulating-an-ac-motor-drive.html#f4-7795

	data.adc1 = ADC1->DR * adc_2_volt;
	data.adc2 = ADC2->DR * adc_2_volt;
	data.adc3 = ADC3->DR * adc_2_volt;
	data.vbus = data.adc1 * vsense_gain;
	data.ia = -(data.adc2 - 1.65) * isense_gain;
	data.ib = -(data.adc3 - 1.65) * isense_gain;
	data.ic = 0 - data.ia - data.ib; // ia + ib + ic = 0

	if (data.ia > OC_LIM) {
		data.run_state = STATE_OC;
	}
	if (data.ib > OC_LIM) {
		data.run_state = STATE_OC;
	}
	if (data.ic > OC_LIM) {
		data.run_state = STATE_OC;
	}

	// Page 27
	// COmpute D&Q Currents
	data.iqs = two_over_three * data.ia - one_over_three * data.ib - one_over_three * data.ic;
	data.ids = one_over_sqrt3 * data.ic - one_over_sqrt3 * data.ib;

	// From the switch state the last cycle - compute vd and vq
	switch(data.sw){
		case 0:
			// AL BL CL - Braking Low
			data.van = 0.0;
			data.vbn = 0.0;
			data.vcn = 0.0;
			break;
		case 1:
			//  AH BL CL
			vn = data.vbus * one_over_three;
			data.van = data.vbus - vn;
			data.vbn = -vn;
			data.vcn = -vn;
			break;
		case 2:
			//  AH BH CL
			vn = data.vbus * two_over_three;
			data.van = data.vbus - vn;
			data.vbn = data.vbus - vn;
			data.vcn = -vn;
			break;
		case 3:
			//  AL BH CL
			vn = data.vbus * one_over_three;
			data.van = -vn;
			data.vbn = data.vbus - vn;
			data.vcn = -vn;
			break;
		case 4:
			//  AL BH CH
			vn = data.vbus * two_over_three;
			data.van = -vn;
			data.vbn = data.vbus - vn;
			data.vcn = data.vbus - vn;
			break;
		case 5:
			//  AL BL CH
			vn = data.vbus * one_over_three;
			data.van = -vn;
			data.vbn = -vn;
			data.vcn = data.vbus - vn;
			break;
		case 6:
			//  AH BL CH
			vn = data.vbus * two_over_three;
			data.van = data.vbus - vn;
			data.vbn = -vn;
			data.vcn = data.vbus - vn;
			break;
		case 7:
			// AH BH CH - Braking High
			data.van = 0.0;
			data.vbn = 0.0;
			data.vcn = 0.0;
			break;
		default:
			data.van = 0.0;
			data.vbn = 0.0;
			data.vcn = 0.0;
			break;
	}

	// Compute D&Q Voltages
	data.vqs = two_over_three * data.van - one_over_three * data.vbn - one_over_three * data.vcn;
	data.vds = one_over_sqrt3 * data.vcn - one_over_sqrt3 * data.vbn;

	// Compute D&Q flux linkages by integration + the vector norm
	data.flds += (data.vds - data.rs * data.ids) * dt;
	data.flqs += (data.vqs - data.rs * data.iqs) * dt;
	data.fls = hypotf(data.flds, data.flqs);

	// From the flux calculate the sector we're in
	data.flux_ang = atan2f(-data.flds, data.flqs) * rad_2_deg;

	// Compute electromagnetic torque
	data.te = 0.75 * poles * (data.flds * data.iqs - data.flqs * data.ids);

	// Compute hysteresis state switches
	float torque_err = data.torque_goal - data.te;
	float flux_err = data.flux_goal - data.fls;
	//if(torque_err > torque_hyst){dtorque = -1;}
	//if(torque_err < -torque_hyst){dtorque = 1;}

	if(torque_err > 0){
		if(torque_err > torque_hyst){dtorque = -1;}
		else if(torque_err > torque_err_last){dtorque = 0;}
		else{dtorque = -1;}
	}else{
		if(torque_err < torque_hyst){dtorque = 1;}
		else if(torque_err < torque_err_last){dtorque = 0;}
		else{dtorque = 1;}
	}

	torque_err_last = torque_err;

	if(flux_err > flux_hyst){dflux = 1;}
	if(flux_err < -flux_hyst){dflux = -1;}

	// Figure out the first index for the switch table from the hysteresis comparators
	int sw_row;
	if(dflux == 1){
		if(dtorque == 1){sw_row = 0;}
		else if(dtorque == 0){sw_row = 1;}
		else{sw_row = 2;}
	}
	else{
		if(dtorque == 1){sw_row = 3;}
		else if(dtorque == 0){sw_row = 4;}
		else{sw_row = 5;}
	}

	// Next figure out the switch state from the flux angle.
	if(data.flux_ang >= -30 && data.flux_ang < 30){data.flux_sector = 1;}
	else if(data.flux_ang >= 30 && data.flux_ang < 90){data.flux_sector = 2;}
	else if(data.flux_ang >= 90 && data.flux_ang < 150){data.flux_sector = 3;}
	else if(data.flux_ang >= 150 && data.flux_ang < 180){data.flux_sector = 4;}
	else if(data.flux_ang >= -180 && data.flux_ang < -150){data.flux_sector = 4;}
	else if(data.flux_ang >= -150 && data.flux_ang < -90){data.flux_sector = 5;}
	else if(data.flux_ang >= -90 && data.flux_ang < -30){data.flux_sector = 6;}
	else {data.flux_sector = 1;}

	int sw_col = data.flux_sector - 1;

	// Set the switch state
	// TODO add a limiter function here.
	data.sw = switch_vector_lookup[sw_row][sw_col];

	// Resistance corrector
	// TODO
	data.rs = 0.72/2.0;

	// Estimate the speed
	// Wraparound
	if(last_ang < -90 && data.flux_ang > 90){
		last_ang += 360.0;
	}
	if(last_ang > 90 && data.flux_ang < -90){
		last_ang -= 360.0;
	}
	if(run_speed_calc++ >= STEPS_PER_SPEED_UPDATE){
		DBG_1_ON();
		run_speed_calc = 1;
		float deg_per_sec = (data.flux_ang - last_ang) / (dt*STEPS_PER_SPEED_UPDATE);
  	    float inst_rpm = -deg_per_sec * deg_per_sec_to_rpm * 2.0 / poles;
	    data.test = inst_rpm;
	    inst_rpm = lim_float(inst_rpm, -10000, 10000);
	    last_ang = data.flux_ang;
    	// Filter for speed
	    speed_filt_sum += inst_rpm;
	    speed_filt_sum -= speed_filt_arr[speed_filt_idx];
	    speed_filt_arr[speed_filt_idx] = inst_rpm;
	    speed_filt_idx++;
	    if(speed_filt_idx >= SPEED_FILT_LEN){ speed_filt_idx = 0;}

	    data.speed_rpm = speed_filt_sum / SPEED_FILT_LEN;
	    DBG_1_OFF();
	}else{
		steps_per_angle_update++;
	}


	// Switch to set switch states
	switch(data.sw){
		case 0:
			// AL BL CL - Brake
			AH_OFF();
			BH_OFF();
			CH_OFF();
			delay200ns();
			AL_ON();
			BL_ON();
			CL_ON();
			break;
		case 1:
			//  AH BL CL - V1
			AL_OFF();
			BH_OFF();
			CH_OFF();
			delay200ns();
			AH_ON();
			BL_ON();
			CL_ON();
			break;
		case 2:
			//  AH BH CL - V2
			AL_OFF();
			BL_OFF();
			CH_OFF();
			delay200ns();
			AH_ON();
			BH_ON();
			CL_ON();
			break;
		case 3:
			//  AL BH CL - V3
			AH_OFF();
			BL_OFF();
			CH_OFF();
			delay200ns();
			AL_ON();
			BH_ON();
			CL_ON();
			break;
		case 4:
			//  AL BH CH - V4
			AH_OFF();
			BL_OFF();
			CL_OFF();
			delay200ns();
			AL_ON();
			BH_ON();
			CH_ON();
			break;
		case 5:
			//  AL BL CH - V5
			AH_OFF();
			BH_OFF();
			CL_OFF();
			delay200ns();
			AL_ON();
			BL_ON();
			CH_ON();
			break;
		case 6:
			//  AH BL CH - V6
			AL_OFF();
			BH_OFF();
			CL_OFF();
			delay200ns();
			AH_ON();
			BL_ON();
			CH_ON();
			break;
		case 7:
			// AH BH CH - Brake
			AL_OFF();
			BL_OFF();
			CL_OFF();
			delay200ns();
			AH_ON();
			BH_ON();
			CH_ON();
			break;
		default:
			// All Off
			AL_OFF();
			AH_OFF();
			BL_OFF();
			BH_OFF();
			CL_OFF();
			CH_OFF();
			break;
	}

	DBG_0_OFF();
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);

	GeneralInit();

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_Delay(250);
	ssd1306_SetCursor(0, 23);
	ssd1306_WriteString("Initialized", Font_11x18, White);
	ssd1306_UpdateScreen();
	HAL_Delay(250);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	//RetargetInit(&huart3);

	//NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	//osTimerStart(timer_100msHandle, 1);
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of task2 */
  task2Handle = osThreadNew(StartTask2, NULL, &task2_attributes);

  /* creation of speed_control */
  speed_controlHandle = osThreadNew(SpeedControl, NULL, &speed_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.Timing = 0x6000030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1350;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|GAH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|dbg3_Pin|GCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_1|GPIO_PIN_2|LD3_Pin 
                          |dbg2_Pin|GBH_Pin|LINE_37_Pin|GPIO_PIN_6 
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GBL_GPIO_Port, GBL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|GCH_Pin|GAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 GAH_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GAH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 dbg3_Pin GCL_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|dbg3_Pin|GCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB1 PB2 LD3_Pin 
                           dbg2_Pin GBH_Pin LINE_37_Pin PB6 
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_1|GPIO_PIN_2|LD3_Pin 
                          |dbg2_Pin|GBH_Pin|LINE_37_Pin|GPIO_PIN_6 
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GBL_Pin */
  GPIO_InitStruct.Pin = GBL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GBL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin GCH_Pin GAL_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin|GCH_Pin|GAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_PMC_I2C_PB7_FMP);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used 
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */

	struct pbuf *p;

	osDelay(100);
	data.run_state = STATE_RUN;

	while (1) {
		//DBG_4_ON();

		IP4_ADDR(&dst_addr, 192, 168, 1, 44);
		upcb = udp_new();
		udp_bind(upcb, IP_ADDR_ANY, 4444);
		udp_connect(upcb, &dst_addr, 4444);

		p = pbuf_alloc(PBUF_TRANSPORT, sizeof(data), PBUF_RAM);

		__disable_irq();
		memcpy(p->payload, &data, sizeof(data));
		__enable_irq();

		udp_send(upcb, p);

		pbuf_free(p); //De-allocate packet buffer
		udp_disconnect(upcb);
		udp_remove(upcb);

		LED_B_TOG();

		//DBG_4_OFF();

		vTaskDelay(1);

	}
	osThreadTerminate(NULL);
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask2 */

void hack_format(float val, char *sign, int *int_part, int *rem_part) {
	*sign = (val < 0) ? '-' : ' ';
	*int_part = fabsf(val);
	float frac = fabsf(val) - *int_part;
	*rem_part = truncf(frac * 100);
}

/**
 * @brief Function implementing the task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
  /* USER CODE BEGIN StartTask2 */
	/* Infinite loop */
	while (1) {
		//DBG_3_ON();
		char buf[32];
		int n;

		float ia = data.ia;
		float ib = data.ib;
		float ic = data.ic;
		float ids = data.ids;
		float iqs = data.iqs;
		float fls = data.fls*1000.;
		float te = data.torque_goal*1000.;
		float vbus = data.vbus;
		int state = data.run_state;
		int rpm = data.speed_rpm;

		char sign = ' ';
		int int_part = 0;
		int rem_part = 0;

		// Task for I2C Display
		ssd1306_Fill(Black);

		ssd1306_SetCursor(0, 0);
		hack_format(ia, &sign, &int_part, &rem_part);
		n = sprintf(buf, "ia%c%2d.%02d", sign, int_part, rem_part);
		hack_format(ids, &sign, &int_part, &rem_part);
		n = sprintf(buf + n, "  id%c%2d.%02d", sign, int_part, rem_part);
		ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_SetCursor(0, 12);
		hack_format(ib, &sign, &int_part, &rem_part);
		n = sprintf(buf, "ib%c%2d.%02d", sign, int_part, rem_part);
		hack_format(iqs, &sign, &int_part, &rem_part);
		n = sprintf(buf + n, "  iq%c%2d.%02d", sign, int_part, rem_part);
		ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_SetCursor(0, 24);
		hack_format(ic, &sign, &int_part, &rem_part);
		n = sprintf(buf, "ic%c%2d.%02d", sign, int_part, rem_part);
		hack_format(fls, &sign, &int_part, &rem_part);
		n = sprintf(buf + n, "  fs%c%2d.%02d", sign, int_part, rem_part);
		ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_SetCursor(0, 36);
		hack_format(vbus, &sign, &int_part, &rem_part);
		n = sprintf(buf, "vb %2d.%02d", int_part, rem_part);
		hack_format(te, &sign, &int_part, &rem_part);
		n = sprintf(buf + n, "  te%c%2d.%02d", sign, int_part, rem_part);
		ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_SetCursor(0, 48);
		switch (state) {
		case STATE_OFF:
			ssd1306_WriteString("OFF    ", Font_7x10, White);
			break;
		case STATE_RUN:
			ssd1306_WriteString("RUN     ", Font_7x10, White);
			break;
		case STATE_OC:
			ssd1306_WriteString("OVERCURR", Font_7x10, White);
			break;
		case STATE_OV:
			ssd1306_WriteString("OVERVOLT", Font_7x10, White);
			break;
		}
		sprintf(buf, "rpm %d", rpm);
        ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_UpdateScreen();
		//DBG_3_OFF();
		LED_G_TOG();


		/*
		AL_OFF();
		AH_OFF();
		BL_OFF();
		BH_OFF();
		CL_OFF();
		CH_OFF();
		delay200ns();
		switch(i++){
			case 0:
				AL_ON();
				break;
			case 1:
				AH_ON();
				break;
			case 2:
				BL_ON();
				break;
			case 3:
				BH_ON();
				break;
			case 4:
				CL_ON();
				break;
			case 5:
				CH_ON();
				break;
		}
		if(i > 5){
			i = 0;
		}*/



		vTaskDelay(20);
	}

	osThreadTerminate(NULL);
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_SpeedControl */
/**
* @brief Function implementing the speed_control thread.
* @param argument: Not used
* @retval None
*/
const float speed_dt = 0.001;
const float speed_goal = 4000;
const float speed_p_gain = 1e-5;
const float speed_i_gain = 1e-7;
static float speed_integrator;

/* USER CODE END Header_SpeedControl */
void SpeedControl(void *argument)
{
  /* USER CODE BEGIN SpeedControl */
  /* Infinite loop */
  for(;;)
  {

	data.flux_goal = sqrtf(2. / 3.) * Ke * 2.0 / poles;
	flux_hyst = data.flux_goal / 20;
	//data.flux_goal = 0.1;
	float speed_err = lim_float(speed_goal - data.speed_rpm, -500, 500);
	speed_integrator += lim_float(speed_err * speed_i_gain, -10000, 10000);
	float pterm = speed_err * speed_p_gain;
	float control = lim_float(pterm + speed_integrator, 0, 0.1);
	data.torque_goal = lim_float(control, data.torque_goal-0.001, data.torque_goal+0.001); // Slew limit
	//data.torque_goal = 0.015;


    osDelay(1);
  }
  /* USER CODE END SpeedControl */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
