/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

enum submenu_index {
  MENU_IDLE = 0,
  MENU_INICIO = 1,
  MENU_PORTADORA = 3,
  MENU_MODULANTE,
  MENU_F_MAX,
  MENU_MARCHA,
  MENU_RAMP_UP,
  MENU_RAMP_DW,
} submenu_index_enum;


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Inicializar(void);
void generate_spwm_table(void);
void OLED(void);
void OLED_MENU(void);
void OLED_MENU_Portadora(void);
void OLED_MENU_Modulante(void);
void OLED_MENU_Modulacion(void);
void OLED_MENU_Marcha(void);
void OLED_MENU_RampaUP(void);
void OLED_MENU_RampaDW(void);
void Paro_SPWM(void);
void Start_SPWM(void);
void Rampa(void);
void Generar(void);
float leer_adc(void);
void OLED_MENU_Amper(void);
void OLED_ERROR(void);
uint32_t DETECTA_ENCODER(uint32_t cuentaPasada,uint32_t valorSalida,uint32_t valMax);
float scale_adc_value(uint16_t adc_value);
float ema_filter(float current_value, float previous_value, float alpha);

//
void Flash_Write(uint32_t address, uint32_t data);
void Flash_Write_Float(uint32_t address, float data);
uint32_t Flash_Read(uint32_t address);
float Flash_Read_Float(uint32_t address);
void Flash_Erase(uint32_t page_address);
void Grabar_Todo(void);
void Error_Handler(void);
//
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MI_Pin GPIO_PIN_3
#define MI_GPIO_Port GPIOA
#define BT_Pin GPIO_PIN_4
#define BT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
