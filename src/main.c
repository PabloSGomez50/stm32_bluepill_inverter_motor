/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @autor          : Ariel Tulian
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
#include "stdio.h"
#include "Anglas_OLED_SSD1306.h"
//#include "Anglas_ENCODER_ROT.h"
#include "Anglas_IMAGENES.h"
#include <stdbool.h>
#include <string.h>
#define FLASH_PAGE_ADDRESS 0x0801F800
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define TIM4CLK 72000000
#define NS 1300

float F_SIGNAL = 1.1;
float F_SIGNAL_F = 50.0;
float F_SIGNAL_A= 50.0;
float F_SIGNAL_G= 50.0;
uint32_t F_PORTADORA = 4000; //1100 minimo

uint32_t spwm_table_R[NS];
uint32_t spwm_table_S[NS];
uint32_t spwm_table_T[NS];
uint32_t DestAddress = (uint32_t) &(TIM1->CCR1);
uint32_t DestAddress2 = (uint32_t) &(TIM1->CCR2);
uint32_t DestAddress3 = (uint32_t) &(TIM1->CCR3);
uint32_t TIM4_Ticks;
uint32_t TIM1_Ticks;

char buff[32];

uint8_t Menu;
uint8_t subMenu;
bool ingresoMenu=0;
uint32_t cuentaAnterior=500;

uint32_t cuentaMenu;
uint32_t cuentaSubMenu;
uint32_t cuentaOled;
bool pulso=0;
bool pulso2=0;
uint32_t F_PORTADORA_MENU;
uint32_t F_SIGNAL_MENU_E;
float F_SIGNAL_MENU;
uint32_t INDICE_MENU_E;
float INDICE_MENU;
float M_a = 1.0;  // indice de modulación inicial
float frecMax = 99.9;
bool MI_Invertida=0;
uint32_t rampaUp=15;
uint32_t RAMPA_UP_MENU;
uint32_t rampaDown=15;
uint32_t RAMPA_DOWN_MENU;
uint32_t rampaGral;
bool Marcha_GRAL=0;
bool bandera=1;
int32_t encoder_position = 0;
int32_t last_encoder_value = 0;
int32_t cuenta=0;
int habCuenta=0;
int contador=0;
int banderaM=0;

#define SAMPLE_RATE 100           // Frecuencia de muestreo (10 kHz)
#define CYCLE_SAMPLES (SAMPLE_RATE / 5) // Muestras por ciclo (50 Hz)


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  //HAL_TIM_Base_Start_IT(&htim3);

  F_PORTADORA = Flash_Read(FLASH_PAGE_ADDRESS);
  if(F_PORTADORA>18000){
	  	HAL_FLASH_Unlock();

	  	FLASH_EraseInitTypeDef eraseInitStruct;
	  	uint32_t pageError;

	  	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  	eraseInitStruct.PageAddress = FLASH_PAGE_ADDRESS;
	  	eraseInitStruct.NbPages = 1;

	  	if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
	  	 	Error_Handler();
	  	}
	  	Flash_Write(FLASH_PAGE_ADDRESS, 4000);  // F_PORTADORA
	  	Flash_Write_Float(FLASH_PAGE_ADDRESS + 4, 50.0);  // Indice M
	  	Flash_Write_Float(FLASH_PAGE_ADDRESS + 8, 50.0);  // Indice M
	  	Flash_Write(FLASH_PAGE_ADDRESS + 12, 0);  // Marcha
	  	Flash_Write(FLASH_PAGE_ADDRESS + 16, 10);  // Rampa UP
	  	Flash_Write(FLASH_PAGE_ADDRESS + 20, 10);  // Rampa DOWN
	  	Flash_Write_Float(FLASH_PAGE_ADDRESS + 24, 3.0);  // Amper

	  	HAL_FLASH_Lock();
	  	F_PORTADORA = Flash_Read(FLASH_PAGE_ADDRESS);
  }
  F_SIGNAL_G = Flash_Read_Float(FLASH_PAGE_ADDRESS + 4);

  frecMax = Flash_Read_Float(FLASH_PAGE_ADDRESS + 8);
  MI_Invertida = Flash_Read(FLASH_PAGE_ADDRESS + 12);
  rampaUp = Flash_Read(FLASH_PAGE_ADDRESS + 16);
  rampaDown = Flash_Read(FLASH_PAGE_ADDRESS + 20);

  if(F_SIGNAL_G>frecMax){
  	  F_SIGNAL_G=frecMax;
  }
  F_SIGNAL_A=F_SIGNAL_G;
  F_SIGNAL_F=F_SIGNAL_G;
  
  if (F_SIGNAL_F < 0.1f) {
    F_SIGNAL_F = 1.0f;
  }

  TIM4_Ticks = TIM4CLK / (NS * F_SIGNAL_F);
  TIM1_Ticks = TIM4CLK / F_PORTADORA;
  TIM4->ARR=TIM4_Ticks-1;
  TIM1->ARR=TIM1_Ticks-1;
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  M_a=F_SIGNAL_F*0.02;
  if(F_SIGNAL_F>50.0){
	  M_a=1;
  }
  Inicializar();
  Paro_SPWM();

  HAL_Delay(1000);
  //OLED_Imagen(house);
  while (1)
  {
	  if(HAL_GPIO_ReadPin(GPIOA, BT_Pin)){
		  if(cuentaMenu>1 && cuentaMenu<8){
			  Marcha_GRAL=!Marcha_GRAL;
		  }
		  cuentaMenu=0;
	  } else {
		  cuentaMenu++;

		  if(cuentaMenu==9){
			Menu=0;//TIM2->CNT;
			//cuentaAnterior=TIM2->CNT;
			OLED_Clear();
		  	cuentaMenu=11;
		  }
		  while(cuentaMenu>9){
			  Menu = DETECTA_ENCODER(cuentaAnterior,Menu,7);
			  cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
			  pulso=0;
			  while(subMenu==3){
				if(Marcha_GRAL==1){
					OLED();
					F_SIGNAL_A=0.0;
					Rampa();
					OLED_Clear();
				}
				Marcha_GRAL=0;
			  	switch(cuentaSubMenu){
			  		case 0:
			  			F_PORTADORA_MENU =2000;
			  			break;
			  		case 1:
			  			F_PORTADORA_MENU =4000;
			  			break;
			  		case 2:
			  			F_PORTADORA_MENU =8000;
			  			break;
			  		case 3:
			  			F_PORTADORA_MENU =16000;
			  			break;
			  	}
			  	OLED_MENU_Portadora();
			  	cuentaSubMenu= DETECTA_ENCODER(cuentaAnterior,cuentaSubMenu,4);
			  	cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);

			  	if(!HAL_GPIO_ReadPin(GPIOA, BT_Pin)){
			  		if(pulso==1){
			  			subMenu=1;
			  			F_PORTADORA=F_PORTADORA_MENU;
			  			TIM1_Ticks = TIM4CLK / F_PORTADORA;
			  			Paro_SPWM();
			  			TIM1->ARR=TIM1_Ticks-1;
			  			Inicializar();
			  			if(F_SIGNAL_F >=1.0){
			  				Start_SPWM();
			  			}
			  			//Grabar_Todo();
			  			pulso=0;
			  		}
			  	}else{
			  		pulso=1;
			  	}

			  }

			  while(subMenu==4){

				OLED_MENU_Modulante();

				F_SIGNAL_MENU_E= DETECTA_ENCODER(cuentaAnterior,F_SIGNAL_MENU_E, (uint32_t) (frecMax*10));
			  	cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
			  	if(F_SIGNAL_MENU_E<10){
			  		F_SIGNAL_MENU_E=10;
			  	}
			  	F_SIGNAL_MENU=(float)F_SIGNAL_MENU_E/10.0;
			  	if(HAL_GPIO_ReadPin (GPIOA, BT_Pin)==0){
			  		if(pulso==1){
			  			subMenu=1;
			  			F_SIGNAL_A=F_SIGNAL_MENU;
			  			F_SIGNAL_G= F_SIGNAL_A;
			  			//Grabar_Todo();
			  			pulso=0;
			  		}
			  	}else{
			  		pulso=1;
			  	}

			  }

			  while(subMenu==5){

				  OLED_MENU_Modulacion();

				  INDICE_MENU_E= DETECTA_ENCODER(cuentaAnterior,INDICE_MENU_E,1000);
				  cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
				  INDICE_MENU=INDICE_MENU_E/10;
				  if(INDICE_MENU==100.0){
					  INDICE_MENU=99.9;
				  }
				  if(HAL_GPIO_ReadPin (GPIOA, BT_Pin)==0){
			  		if(pulso==1){
			  			subMenu=1;
			  			frecMax=INDICE_MENU;
			  			if(F_SIGNAL_F >=0.1){
			  				Start_SPWM();
			  			}
			  			pulso=0;
			  		}
				  }else{
			  		pulso=1;
				  }

			  }
			  ///MI_Invertida
			  while(subMenu==6){

				  OLED_MENU_Marcha();
				  MI_Invertida= DETECTA_ENCODER(cuentaAnterior,MI_Invertida,2);
				  cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
				  if(HAL_GPIO_ReadPin (GPIOA, BT_Pin)==0){
			  		if(pulso==1){
			  			subMenu=1;
			  			pulso=0;
			  		}
				  }else{
			  		pulso=1;
				  }
			  }

			  ///
			  while(subMenu==7){

			  	OLED_MENU_RampaUP();

			  	RAMPA_UP_MENU= DETECTA_ENCODER(cuentaAnterior,RAMPA_UP_MENU,15);
			  	cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
			  	if(HAL_GPIO_ReadPin (GPIOA, BT_Pin)==0){
			  		if(pulso==1){
			  			subMenu=1;
			  			rampaUp=RAMPA_UP_MENU;
			  			pulso=0;
			  		}
			  	}else{
			  		pulso=1;
			  	}

			  }

			  while(subMenu==8){
				  OLED_MENU_RampaDW();

				  RAMPA_DOWN_MENU= DETECTA_ENCODER(cuentaAnterior,RAMPA_DOWN_MENU,15);
				  cuentaAnterior=__HAL_TIM_GET_COUNTER(&htim2);
				  if(HAL_GPIO_ReadPin (GPIOA, BT_Pin)==0){
			 		if(pulso==1){
			 			subMenu=1;
			 			rampaDown=RAMPA_DOWN_MENU;
			 			pulso=0;
			 		}
			 	  }else{
			 		pulso=1;
			 	  }
			  }
			  if(subMenu==0){
			  	subMenu=1;
			  }
			  OLED_MENU();
			  if(!HAL_GPIO_ReadPin(GPIOA, BT_Pin)){
				  switch(Menu){
				  	  case 0:
				  		  if(subMenu==2){
				  			OLED_Clear();
				  			subMenu=3;
				  			switch(F_PORTADORA){
				  				case 2000:
				  					cuentaSubMenu =0;
				  					break;
				  				case 4000:
				  					cuentaSubMenu =1;
				  					break;
				  				case 8000:
				  					cuentaSubMenu =2;
				  					break;
				  				case 16000:
				  					cuentaSubMenu =3;
				  					break;
				  			}
				  		  }
				  		  break;
				  	  case 1:
				  		  OLED_Clear();
				  		  F_SIGNAL_MENU_E=F_SIGNAL_G*10;
				  		  subMenu=4;
				  		  break;
				  	  case 2:
				  		  OLED_Clear();
				  		  INDICE_MENU_E=(uint32_t) (frecMax*10);
				  		  subMenu=5;
				  		  break;
				  	  case 3:
				  		  OLED_Clear();
				  		  subMenu=6;
				  		  break;
				  	  case 4:
				  		  OLED_Clear();
				  		  RAMPA_UP_MENU=rampaUp;
				  		  subMenu=7;
				  		  break;
				  	  case 5:
				  		  OLED_Clear();
				  		  RAMPA_DOWN_MENU=rampaDown;
				  		  subMenu=8;
				  		  break;
				  	  case 7:
						  // Opcion volver
				  		  OLED_Clear();
				  	      Menu=0;
				  	  	  cuentaMenu=0;
				  	  	  subMenu=0;
				  	  	  Grabar_Todo();
				  	  	  break;
				  }
			  }else{
				  if(subMenu==1){
					  subMenu=2;
				  }
			  }
		  }
	  }

	//   if(MI_Invertida==1){
	// 	  if(HAL_GPIO_ReadPin(GPIOA, MI_Pin) == 1){
	// 		  Marcha_GRAL=0;
	// 	  }else{
	// 		  Marcha_GRAL=1;
	// 	  }
	//   }

	  if(Marcha_GRAL==1){
		// Ramp up
		F_SIGNAL_A= F_SIGNAL_G;
	  }else{
		// Ramp down
		F_SIGNAL_A=0.0;
	  }
	  OLED();
	  Rampa();
  }
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 107;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MI_Pin */
  GPIO_InitStruct.Pin = MI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_Pin */
  GPIO_InitStruct.Pin = BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Genero tabla SPWM
void generate_spwm_table(void) {
	float amplitude = (float)(M_a*((TIM1_Ticks-150)/2)); // Amplitud de la señal SPWM
	for (int i = 0; i < NS; i++) {
		// Calcula el ángulo para la posición actual
		float angle = 2 * 3.14 * i / NS;
		float angle120 = 2*3.14*(i+2*NS/3)/NS;
		float angle240 = 2*3.14*(i+NS/3)/NS;
		float correccion=0.003;
		// Limito rango
		if(TIM1_Ticks<10000){
			correccion=0.015;
		}
		// Calcula el valor SPWM usando la función seno
		float value = amplitude*(2-(1.00 + sin(angle)))+(correccion*TIM1_Ticks)+25;
		float value2 = amplitude*(2-(1.00 + sin(angle120)))+(correccion*TIM1_Ticks)+25;
		float value3 = amplitude*(2-(1.00 + sin(angle240)))+(correccion*TIM1_Ticks)+25;
		// Almacena el valor en la tabla
		if (MI_Invertida) {
			spwm_table_R[i] = (uint32_t)value;
			spwm_table_S[i] = (uint32_t)value3;
			spwm_table_T[i] = (uint32_t)value2;
		} else {
			spwm_table_R[i] = (uint32_t)value;
			spwm_table_S[i] = (uint32_t)value2;
			spwm_table_T[i] = (uint32_t)value3;
		}
	}
}

void OLED(void) {
	if(F_SIGNAL_F!=F_SIGNAL_A){
		if(habCuenta==1){
			OLED_Print_Text(5,5,3,"UP    ");
			habCuenta=3;
		}
		if(habCuenta==2){
			OLED_Print_Text(5,5,3,"DWN   ");
			habCuenta=3;
		}
	} else {
		snprintf(buff,sizeof(buff),
			"%2ldkHz/%02d.%1dHz",
			F_PORTADORA/1000,
			(uint16_t)(F_SIGNAL_G),
			(uint16_t)(F_SIGNAL_G * 10) % 10
		);
		OLED_Print_Text(0,5,2,buff);

		OLED_Print_Text(2,0,2,"Amp.:");
		snprintf(buff,sizeof(buff),"%02d.%1d%%",
			(uint16_t)(M_a * 100),
			(uint16_t)(M_a * 1000) % 10
		);
		OLED_Print_Text(2,50,2,buff);

		if(F_SIGNAL_F<10.0){
			OLED_Print_Text(5,65,3," ");
		}
		snprintf(buff,sizeof(buff),"%02d.%1d",(uint16_t)(F_SIGNAL_F / 10), (uint16_t)F_SIGNAL_F % 10);
		OLED_Print_Text(5,5,3,buff);
		OLED_Print_Text(5,90,3,"Hz");
		HAL_Delay(50);
	}

}

uint32_t DETECTA_ENCODER(uint32_t cuentaPasada,uint32_t valorSalida,uint32_t valMax){
	int32_t Valor=valorSalida;
	if(__HAL_TIM_GET_COUNTER(&htim2) == cuentaPasada)
		return Valor;

	if(valMax==35){
		if(__HAL_TIM_GET_COUNTER(&htim2)<cuentaPasada){
			Valor = Valor + 1;
			if(Valor>valMax){
				Valor=valMax;
			}
		}
		if(__HAL_TIM_GET_COUNTER(&htim2)>cuentaPasada){
			Valor = Valor - 1;
			if(Valor<0){
				Valor=0;
			}
		}
	}else{
		if(valMax>20){
					if(__HAL_TIM_GET_COUNTER(&htim2)<cuentaPasada){
					Valor = Valor + 5;
						if(Valor>valMax){
							Valor=valMax;
						}
					}
					if(__HAL_TIM_GET_COUNTER(&htim2)>cuentaPasada){
						Valor = Valor - 5;
						if(Valor<0){
							Valor=0;
						}
					}
				} else{
					if(valMax>10){
						if(__HAL_TIM_GET_COUNTER(&htim2)<cuentaPasada){
							Valor = Valor + 1;
							if(Valor>valMax){
								Valor=valMax;
							}
						}
						if(__HAL_TIM_GET_COUNTER(&htim2)>cuentaPasada){
							Valor = Valor - 1;
							if(Valor<0){
								Valor=0;
							}
						}
					} else{
						if(__HAL_TIM_GET_COUNTER(&htim2)-1<cuentaPasada){
							Valor = (valorSalida + 1) % valMax;
						}
						if(__HAL_TIM_GET_COUNTER(&htim2)+1>cuentaPasada){
							Valor = (valorSalida-1+valMax) % valMax;
						}
					}
				}
	}
	HAL_Delay(1);
	return Valor;
}

void OLED_MENU(void) {

	switch(Menu){
		case 0:
			OLED_Print_Text(0,0,2,">Set portadora");//
			OLED_Print_Text(2,0,2,"Set modulante ");//
			OLED_Print_Text(4,0,2,"Set frec. Max");//
			OLED_Print_Text(6,0,2,"Set marcha ");//
			break;
		case 1:
			OLED_Print_Text(0,0,2,"Set portadora ");//
			OLED_Print_Text(2,0,2,">Set modulante");//
			OLED_Print_Text(4,0,2,"Set frec. Max ");//
			OLED_Print_Text(6,0,2,"Set marcha ");//
			break;
		case 2:
			OLED_Print_Text(0,0,2,"Set portadora ");//
			OLED_Print_Text(2,0,2,"Set modulante ");//
			OLED_Print_Text(4,0,2,">Set frec. Max");//
			OLED_Print_Text(6,0,2,"Set marcha ");//
			break;
		case 3:
			OLED_Print_Text(0,0,2,"Set portadora ");//
			OLED_Print_Text(2,0,2,"Set modulante ");//
			OLED_Print_Text(4,0,2,"Set frec. Max ");//
			OLED_Print_Text(6,0,2,">Set marcha");//
			break;
		case 4:
			OLED_Print_Text(0,0,2,">Set ramp. UP");//
			OLED_Print_Text(2,0,2,"Set ramp. DW ");//
			OLED_Print_Text(4,0,2,"              ");//
			OLED_Print_Text(6,0,2,"Volver        ");//
			break;
		case 5:
			OLED_Print_Text(0,0,2,"Set ramp. UP  ");//
			OLED_Print_Text(2,0,2,">Set ramp. DW");//
			OLED_Print_Text(4,0,2,"              ");//
			OLED_Print_Text(6,0,2,"Volver        ");//
			break;
		case 6:
			Menu++;
			break;
		case 7:
			OLED_Print_Text(0,0,2,"Set ramp. UP  ");//
			OLED_Print_Text(2,0,2,"Set ramp. DW ");//
			OLED_Print_Text(4,0,2,"              ");//
			OLED_Print_Text(6,0,2,">Volver       ");//
			break;
	}
	//Se anula visualización pantalla rampa por exceso de tiempo, se probo con una interrupción por TIM3 pero no funciono bien el PWM, considerar separar o 2 micros o 2 nucleos dichas operaciones, priorizar hardware

}

void OLED_MENU_Portadora(void) {
	OLED_Print_Text(0,0,2,"SET PORTADORA:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	sprintf(buff,"%2ld Hz  ",(F_PORTADORA_MENU));//
	OLED_Print_Text(4,0,2,buff);
}

void OLED_MENU_Modulante(void) {
	OLED_Print_Text(0,0,2,"SET MODULANTE:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	sprintf(buff,"%03d.%1d Hz", (uint16_t)(F_SIGNAL_MENU_E / 10), (uint16_t)F_SIGNAL_MENU_E % 10);//sprintf(buff,"%2.1f Hz",(F_SIGNAL_MENU));//
	OLED_Print_Text(4,0,2,buff);
}

void OLED_MENU_Modulacion(void) {
	OLED_Print_Text(0,0,2,"SET F. MAX.:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	sprintf(buff,"%03d.%1d Hz  ", (uint16_t)(INDICE_MENU_E / 10), (uint16_t)INDICE_MENU_E % 10);
	OLED_Print_Text(4,0,2,buff);
}

void OLED_MENU_Marcha(void) {
	OLED_Print_Text(0,0,2,"SET INV. MAR.:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	if(MI_Invertida==1){
		OLED_Print_Text(4,0,2,"True ");
	}else{
		OLED_Print_Text(4,0,2,"False");
	}
}

void OLED_MENU_RampaUP(void) {
	OLED_Print_Text(0,0,2,"SET RAMPA UP:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	OLED_Print_Text(2,0,1,"Rango: (0-15)S");
	sprintf(buff,"%2ld Seg  ",(RAMPA_UP_MENU));//
	OLED_Print_Text(4,0,2,buff);
}

void OLED_MENU_RampaDW(void) {
	OLED_Print_Text(0,0,2,"SET RAMPA DW:"); //4 Páginas, columna 127 máx, tmaño 1 1 pag // tamaño 2 2 pagina // tamaño 3 3 páginas
	OLED_Print_Text(2,0,1,"Rango: (0-15)S");
	sprintf(buff,"%2ld Seg  ",(RAMPA_DOWN_MENU));//
	OLED_Print_Text(4,0,2,buff);
}

void Paro_SPWM(void){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
}

void Start_SPWM(void){
	TIM1->BDTR = (TIM1->BDTR & 0xFFFFFF00) | 90; //Ayuda no se quemen las salidas complementarias al parar configuro el OSSI y el OSSR
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
}

float calcularSalto(uint32_t nivelRampa, float deltaF) {
    float divisor;
    if (nivelRampa == 0) return 0; // sin rampa
    if (nivelRampa <= 1) divisor = 5;
    else if (nivelRampa <= 3) divisor = 10;
    else if (nivelRampa <= 5) divisor = 20;
    else if (nivelRampa <= 12) divisor = 50;
    else divisor = 100;

    return fabs(deltaF) / divisor;
}

void Rampa(void){
	uint32_t newrampUp=rampaUp;
	uint32_t newrampDW=rampaDown;
	float deltaF=F_SIGNAL_F-F_SIGNAL_A;
	// uint8_t rampaUp = deltaF < 0 ? 1 : 0;
	uint8_t isRampUp = deltaF < 0 ? 1 : 0;
	uint32_t nivelRampa = isRampUp ? newrampUp : newrampDW;
	float salto = calcularSalto(nivelRampa, deltaF);

	uint32_t last_time = HAL_GetTick();

	if(F_SIGNAL_A-1.0==0.0){
		newrampUp=0;
		salto=0;
	}
	while(F_SIGNAL_F!=F_SIGNAL_A){

		uint32_t delay_ms = (nivelRampa == 0) ? 0 : (nivelRampa * 1000) / ((fabs(deltaF)) / salto);

		if (nivelRampa == 0) {
			F_SIGNAL_F = F_SIGNAL_A;
		} else {
			F_SIGNAL_F += isRampUp ? salto : -salto;
		}

		  while (HAL_GetTick() - last_time < delay_ms) { // 1000 ms

		  }
		  last_time = HAL_GetTick();

		  if (habCuenta == 0) habCuenta = isRampUp ? 1 : 2;

		  if (isRampUp && F_SIGNAL_F > F_SIGNAL_A + 0.1) F_SIGNAL_F = F_SIGNAL_A;
		  if (!isRampUp && F_SIGNAL_F < F_SIGNAL_A - 0.1) F_SIGNAL_F = F_SIGNAL_A;

		  if (F_SIGNAL_F > 50.0) {
			  M_a = 1;
		  } else if (F_SIGNAL_F > 0.9) {
		      M_a = F_SIGNAL_F * 0.02;
		      Generar();
		  }
	  if(F_SIGNAL_F > 0.9 && banderaM==0){
		  Start_SPWM();
		  banderaM=1;
	  }else if(F_SIGNAL_F <0.9){
		  Paro_SPWM();
	  }


	  TIM4_Ticks = TIM4CLK / (NS * F_SIGNAL_F);
	  TIM4->ARR=TIM4_Ticks-1;

	  if(HAL_GPIO_ReadPin(GPIOA, BT_Pin)==0 && pulso2==1){
		  pulso2=0;
		  break;
	  }else{
		  pulso2=1;
	  }
	}
	habCuenta=0;
	banderaM=0;
}

void Grabar_Todo(void){

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t pageError;

	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitStruct.PageAddress = FLASH_PAGE_ADDRESS;
	eraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
	 	Error_Handler();
	}
	Flash_Write(FLASH_PAGE_ADDRESS, F_PORTADORA);  // F_PORTADORA
	Flash_Write_Float(FLASH_PAGE_ADDRESS + 4, F_SIGNAL_G);  // �?ndice M
	Flash_Write_Float(FLASH_PAGE_ADDRESS + 8, frecMax);  // �?ndice M
	Flash_Write(FLASH_PAGE_ADDRESS + 12, MI_Invertida);  // Marcha
	Flash_Write(FLASH_PAGE_ADDRESS + 16, rampaUp);  // Rampa UP
	Flash_Write(FLASH_PAGE_ADDRESS + 20, rampaDown);  // Rampa DOWN

	HAL_FLASH_Lock();
}

void Flash_Write(uint32_t address, uint32_t data) {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
    HAL_FLASH_Lock();
}

float Flash_Read_Float(uint32_t address) {
    return *(float*)address;
}

void Flash_Write_Float(uint32_t address, float data) {
	uint32_t tmp;
    memcpy(&tmp, &data, sizeof(tmp));
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, tmp);
    HAL_FLASH_Lock();
}

uint32_t Flash_Read(uint32_t address) {
    return *(uint32_t*)address;
}

void Flash_Erase(uint32_t page_address) {
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError;

    HAL_FLASH_Unlock();

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = page_address;
    eraseInitStruct.NbPages = 1;

    HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    HAL_FLASH_Lock();
}

void Generar(void){
	generate_spwm_table();

	HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)spwm_table_R, DestAddress, NS);
	HAL_DMA_Start_IT(&hdma_tim4_ch2, (uint32_t)spwm_table_S, DestAddress2, NS);
	HAL_DMA_Start_IT(&hdma_tim4_ch3, (uint32_t)spwm_table_T, DestAddress3, NS);
}

void Inicializar(void){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);

	generate_spwm_table();

	Paro_SPWM();

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3);

	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC3);
}

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
