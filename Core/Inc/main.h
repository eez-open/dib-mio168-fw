/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIN4_Pin GPIO_PIN_2
#define DIN4_GPIO_Port GPIOE
#define DIN3_Pin GPIO_PIN_3
#define DIN3_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_4
#define DIN2_GPIO_Port GPIOE
#define DIN1_Pin GPIO_PIN_5
#define DIN1_GPIO_Port GPIOE
#define DIN0_Pin GPIO_PIN_6
#define DIN0_GPIO_Port GPIOE
#define USEL1_4_Pin GPIO_PIN_13
#define USEL1_4_GPIO_Port GPIOC
#define USEL10_4_Pin GPIO_PIN_14
#define USEL10_4_GPIO_Port GPIOC
#define ISEL_3_Pin GPIO_PIN_15
#define ISEL_3_GPIO_Port GPIOC
#define USEL1_3_Pin GPIO_PIN_0
#define USEL1_3_GPIO_Port GPIOF
#define USEL10_3_Pin GPIO_PIN_1
#define USEL10_3_GPIO_Port GPIOF
#define USEL100_3_Pin GPIO_PIN_2
#define USEL100_3_GPIO_Port GPIOF
#define ISEL_S_4_Pin GPIO_PIN_3
#define ISEL_S_4_GPIO_Port GPIOF
#define ISEL_R_4_Pin GPIO_PIN_4
#define ISEL_R_4_GPIO_Port GPIOF
#define ISEL10_S_4_Pin GPIO_PIN_5
#define ISEL10_S_4_GPIO_Port GPIOF
#define ISEL10_R_4_Pin GPIO_PIN_1
#define ISEL10_R_4_GPIO_Port GPIOC
#define ISEL_LOW_4_Pin GPIO_PIN_2
#define ISEL_LOW_4_GPIO_Port GPIOC
#define ISEL_MID_4_Pin GPIO_PIN_3
#define ISEL_MID_4_GPIO_Port GPIOC
#define DOUT0_Pin GPIO_PIN_0
#define DOUT0_GPIO_Port GPIOA
#define DOUT1_Pin GPIO_PIN_1
#define DOUT1_GPIO_Port GPIOA
#define DOUT2_Pin GPIO_PIN_2
#define DOUT2_GPIO_Port GPIOA
#define DOUT3_Pin GPIO_PIN_3
#define DOUT3_GPIO_Port GPIOA
#define DOUT_EN_Pin GPIO_PIN_4
#define DOUT_EN_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_5
#define PWM1_GPIO_Port GPIOA
#define DIN7_Pin GPIO_PIN_6
#define DIN7_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOA
#define DOUT4_Pin GPIO_PIN_4
#define DOUT4_GPIO_Port GPIOC
#define DOUT5_Pin GPIO_PIN_5
#define DOUT5_GPIO_Port GPIOC
#define DOUT6_Pin GPIO_PIN_0
#define DOUT6_GPIO_Port GPIOB
#define DOUT7_Pin GPIO_PIN_1
#define DOUT7_GPIO_Port GPIOB
#define DAC_CS_1_Pin GPIO_PIN_11
#define DAC_CS_1_GPIO_Port GPIOF
#define DAC_CLR_1_Pin GPIO_PIN_12
#define DAC_CLR_1_GPIO_Port GPIOF
#define DAC_CLR_SEL_1_Pin GPIO_PIN_13
#define DAC_CLR_SEL_1_GPIO_Port GPIOF
#define DAC_CS_2_Pin GPIO_PIN_14
#define DAC_CS_2_GPIO_Port GPIOF
#define DAC_CLR_2_Pin GPIO_PIN_15
#define DAC_CLR_2_GPIO_Port GPIOF
#define DAC_CLR_SEL_2_Pin GPIO_PIN_0
#define DAC_CLR_SEL_2_GPIO_Port GPIOG
#define DOUT_FAULT_Pin GPIO_PIN_1
#define DOUT_FAULT_GPIO_Port GPIOG
#define BIAS_EN_Pin GPIO_PIN_7
#define BIAS_EN_GPIO_Port GPIOE
#define AFE_ID1_Pin GPIO_PIN_8
#define AFE_ID1_GPIO_Port GPIOE
#define AFE_ID0_Pin GPIO_PIN_9
#define AFE_ID0_GPIO_Port GPIOE
#define DIB_NSS_Pin GPIO_PIN_11
#define DIB_NSS_GPIO_Port GPIOE
#define DIB_SCLK_Pin GPIO_PIN_12
#define DIB_SCLK_GPIO_Port GPIOE
#define DIB_MISO_Pin GPIO_PIN_13
#define DIB_MISO_GPIO_Port GPIOE
#define DIB_MOSI_Pin GPIO_PIN_14
#define DIB_MOSI_GPIO_Port GPIOE
#define DIB_SYNC_Pin GPIO_PIN_15
#define DIB_SYNC_GPIO_Port GPIOE
#define DIB_IRQ_Pin GPIO_PIN_10
#define DIB_IRQ_GPIO_Port GPIOB
#define DAC_CS_DUAL_Pin GPIO_PIN_12
#define DAC_CS_DUAL_GPIO_Port GPIOB
#define DAC_SCK_Pin GPIO_PIN_13
#define DAC_SCK_GPIO_Port GPIOB
#define DAC_MISO_Pin GPIO_PIN_14
#define DAC_MISO_GPIO_Port GPIOB
#define DAC_MOSI_Pin GPIO_PIN_15
#define DAC_MOSI_GPIO_Port GPIOB
#define DAC_ALARM_Pin GPIO_PIN_8
#define DAC_ALARM_GPIO_Port GPIOD
#define ADIB1_ID2_Pin GPIO_PIN_11
#define ADIB1_ID2_GPIO_Port GPIOD
#define ADIB1_ID1_Pin GPIO_PIN_12
#define ADIB1_ID1_GPIO_Port GPIOD
#define ADIB1_ID0_Pin GPIO_PIN_13
#define ADIB1_ID0_GPIO_Port GPIOD
#define ADIB2_ID2_Pin GPIO_PIN_14
#define ADIB2_ID2_GPIO_Port GPIOD
#define ADIB2_ID1_Pin GPIO_PIN_15
#define ADIB2_ID1_GPIO_Port GPIOD
#define ADIB2_ID0_Pin GPIO_PIN_2
#define ADIB2_ID0_GPIO_Port GPIOG
#define IN_CTRL7_Pin GPIO_PIN_3
#define IN_CTRL7_GPIO_Port GPIOG
#define IN_CTRL6_Pin GPIO_PIN_4
#define IN_CTRL6_GPIO_Port GPIOG
#define IN_CTRL5_Pin GPIO_PIN_5
#define IN_CTRL5_GPIO_Port GPIOG
#define IN_CTRL4_Pin GPIO_PIN_6
#define IN_CTRL4_GPIO_Port GPIOG
#define IN_CTRL3_Pin GPIO_PIN_7
#define IN_CTRL3_GPIO_Port GPIOG
#define IN_CTRL2_Pin GPIO_PIN_8
#define IN_CTRL2_GPIO_Port GPIOG
#define IN_CTRL1_Pin GPIO_PIN_6
#define IN_CTRL1_GPIO_Port GPIOC
#define IN_CTRL0_Pin GPIO_PIN_7
#define IN_CTRL0_GPIO_Port GPIOC
#define SD_DETECT_Pin GPIO_PIN_12
#define SD_DETECT_GPIO_Port GPIOA
#define ADC_CS_Pin GPIO_PIN_15
#define ADC_CS_GPIO_Port GPIOA
#define USEL10_2_Pin GPIO_PIN_0
#define USEL10_2_GPIO_Port GPIOD
#define USEL1_2_Pin GPIO_PIN_1
#define USEL1_2_GPIO_Port GPIOD
#define ISEL_MID_2_Pin GPIO_PIN_4
#define ISEL_MID_2_GPIO_Port GPIOD
#define ISEL_LOW_2_Pin GPIO_PIN_5
#define ISEL_LOW_2_GPIO_Port GPIOD
#define ISEL10_R_2_Pin GPIO_PIN_6
#define ISEL10_R_2_GPIO_Port GPIOD
#define ISEL10_S_2_Pin GPIO_PIN_7
#define ISEL10_S_2_GPIO_Port GPIOD
#define ISEL_R_2_Pin GPIO_PIN_9
#define ISEL_R_2_GPIO_Port GPIOG
#define ISEL_S_2_Pin GPIO_PIN_10
#define ISEL_S_2_GPIO_Port GPIOG
#define USEL100_1_Pin GPIO_PIN_11
#define USEL100_1_GPIO_Port GPIOG
#define USEL10_1_Pin GPIO_PIN_12
#define USEL10_1_GPIO_Port GPIOG
#define USEL1_1_Pin GPIO_PIN_13
#define USEL1_1_GPIO_Port GPIOG
#define ISEL_1_Pin GPIO_PIN_14
#define ISEL_1_GPIO_Port GPIOG
#define ADC_DRDY_Pin GPIO_PIN_15
#define ADC_DRDY_GPIO_Port GPIOG
#define ADC_SCK_Pin GPIO_PIN_3
#define ADC_SCK_GPIO_Port GPIOB
#define ADC_MISO_Pin GPIO_PIN_4
#define ADC_MISO_GPIO_Port GPIOB
#define ADC_MOSI_Pin GPIO_PIN_5
#define ADC_MOSI_GPIO_Port GPIOB
#define ADC_START_Pin GPIO_PIN_6
#define ADC_START_GPIO_Port GPIOB
#define ADC_CLK_Pin GPIO_PIN_7
#define ADC_CLK_GPIO_Port GPIOB
#define SLOW_DIN_1_Pin GPIO_PIN_8
#define SLOW_DIN_1_GPIO_Port GPIOB
#define SLOW_DIN_0_Pin GPIO_PIN_9
#define SLOW_DIN_0_GPIO_Port GPIOB
#define DIN6_Pin GPIO_PIN_0
#define DIN6_GPIO_Port GPIOE
#define DIN5_Pin GPIO_PIN_1
#define DIN5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
