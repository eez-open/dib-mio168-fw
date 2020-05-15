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
#define ADC_SCLK_Pin GPIO_PIN_2
#define ADC_SCLK_GPIO_Port GPIOE
#define ADC_IRQ_Pin GPIO_PIN_3
#define ADC_IRQ_GPIO_Port GPIOE
#define ADC_CS_Pin GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOE
#define ADC_MISO_Pin GPIO_PIN_5
#define ADC_MISO_GPIO_Port GPIOE
#define ADC_MOSI_Pin GPIO_PIN_6
#define ADC_MOSI_GPIO_Port GPIOE
#define TEMP_SW_1_Pin GPIO_PIN_13
#define TEMP_SW_1_GPIO_Port GPIOC
#define TEMP_SW_2_Pin GPIO_PIN_14
#define TEMP_SW_2_GPIO_Port GPIOC
#define DAC_ALARM_Pin GPIO_PIN_15
#define DAC_ALARM_GPIO_Port GPIOC
#define DAC_CS_2_Pin GPIO_PIN_6
#define DAC_CS_2_GPIO_Port GPIOF
#define DAC_CLR_2_Pin GPIO_PIN_7
#define DAC_CLR_2_GPIO_Port GPIOF
#define DAC_CLR_SEL_2_Pin GPIO_PIN_8
#define DAC_CLR_SEL_2_GPIO_Port GPIOF
#define DAC_CLR_1_Pin GPIO_PIN_9
#define DAC_CLR_1_GPIO_Port GPIOF
#define DAC_CLR_SEL_1_Pin GPIO_PIN_10
#define DAC_CLR_SEL_1_GPIO_Port GPIOF
#define DAC_CS_1_Pin GPIO_PIN_1
#define DAC_CS_1_GPIO_Port GPIOC
#define DIN0_Pin GPIO_PIN_0
#define DIN0_GPIO_Port GPIOA
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOA
#define DAC_CS_DUAL_Pin GPIO_PIN_4
#define DAC_CS_DUAL_GPIO_Port GPIOA
#define DAC_SCLK_Pin GPIO_PIN_5
#define DAC_SCLK_GPIO_Port GPIOA
#define DAC_MISO_Pin GPIO_PIN_6
#define DAC_MISO_GPIO_Port GPIOA
#define DAC_MOSI_Pin GPIO_PIN_7
#define DAC_MOSI_GPIO_Port GPIOA
#define DOUT4_Pin GPIO_PIN_4
#define DOUT4_GPIO_Port GPIOC
#define DOUT5_Pin GPIO_PIN_5
#define DOUT5_GPIO_Port GPIOC
#define DOUT6_Pin GPIO_PIN_0
#define DOUT6_GPIO_Port GPIOB
#define DOUT7_Pin GPIO_PIN_1
#define DOUT7_GPIO_Port GPIOB
#define OUT_FAULT_Pin GPIO_PIN_2
#define OUT_FAULT_GPIO_Port GPIOB
#define DIB_IRQ_Pin GPIO_PIN_10
#define DIB_IRQ_GPIO_Port GPIOB
#define OUT_EN_Pin GPIO_PIN_11
#define OUT_EN_GPIO_Port GPIOB
#define DIB_NSS_Pin GPIO_PIN_12
#define DIB_NSS_GPIO_Port GPIOB
#define DIB_SCLK_Pin GPIO_PIN_13
#define DIB_SCLK_GPIO_Port GPIOB
#define DIB_MISO_Pin GPIO_PIN_14
#define DIB_MISO_GPIO_Port GPIOB
#define DIB_MOSI_Pin GPIO_PIN_15
#define DIB_MOSI_GPIO_Port GPIOB
#define DIB_SYNC_Pin GPIO_PIN_11
#define DIB_SYNC_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_12
#define PWM1_GPIO_Port GPIOD
#define PWM2_Pin GPIO_PIN_13
#define PWM2_GPIO_Port GPIOD
#define DOUT2_Pin GPIO_PIN_6
#define DOUT2_GPIO_Port GPIOG
#define DOUT3_Pin GPIO_PIN_7
#define DOUT3_GPIO_Port GPIOG
#define DIN2_Pin GPIO_PIN_6
#define DIN2_GPIO_Port GPIOC
#define DIN3_Pin GPIO_PIN_7
#define DIN3_GPIO_Port GPIOC
#define DOUT0_Pin GPIO_PIN_8
#define DOUT0_GPIO_Port GPIOA
#define DOUT1_Pin GPIO_PIN_9
#define DOUT1_GPIO_Port GPIOA
#define SD_DETECT_Pin GPIO_PIN_15
#define SD_DETECT_GPIO_Port GPIOA
#define CURR_SW0_Pin GPIO_PIN_3
#define CURR_SW0_GPIO_Port GPIOD
#define CURR_SW1_Pin GPIO_PIN_4
#define CURR_SW1_GPIO_Port GPIOD
#define CURR_SW2_Pin GPIO_PIN_5
#define CURR_SW2_GPIO_Port GPIOD
#define CURR_SW3_Pin GPIO_PIN_6
#define CURR_SW3_GPIO_Port GPIOD
#define IN_CTRL0_Pin GPIO_PIN_7
#define IN_CTRL0_GPIO_Port GPIOD
#define IN_CTRL1_Pin GPIO_PIN_9
#define IN_CTRL1_GPIO_Port GPIOG
#define IN_CTRL2_Pin GPIO_PIN_10
#define IN_CTRL2_GPIO_Port GPIOG
#define IN_CTRL3_Pin GPIO_PIN_11
#define IN_CTRL3_GPIO_Port GPIOG
#define IN_CTRL4_Pin GPIO_PIN_12
#define IN_CTRL4_GPIO_Port GPIOG
#define IN_CTRL5_Pin GPIO_PIN_13
#define IN_CTRL5_GPIO_Port GPIOG
#define IN_CTRL6_Pin GPIO_PIN_14
#define IN_CTRL6_GPIO_Port GPIOG
#define IN_CTRL7_Pin GPIO_PIN_3
#define IN_CTRL7_GPIO_Port GPIOB
#define DIN4_Pin GPIO_PIN_4
#define DIN4_GPIO_Port GPIOB
#define DIN5_Pin GPIO_PIN_5
#define DIN5_GPIO_Port GPIOB
#define DIN6_Pin GPIO_PIN_6
#define DIN6_GPIO_Port GPIOB
#define DIN7_Pin GPIO_PIN_7
#define DIN7_GPIO_Port GPIOB
#define SLOW_DIN_0_Pin GPIO_PIN_8
#define SLOW_DIN_0_GPIO_Port GPIOB
#define SLOW_DIN_1_Pin GPIO_PIN_9
#define SLOW_DIN_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
