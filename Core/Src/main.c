/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f411xe.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LED_PIN     (14U)
#define LED_PIN_ODR_MSK     (1U << LED_PIN)  // Pin 14

#define GPIOD_PIN6  (6U)
#define GPIOD_PIN6_ODR_MSK     (1U << GPIOD_PIN6)  // Pin 14


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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        // Clear update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
        GPIOD->ODR ^= LED_PIN_ODR_MSK;
        GPIOD->ODR ^= GPIOD_PIN6_ODR_MSK;
    }
}

void timer2_Int_set()
{
    TIM2->PSC = 159;
    TIM2->ARR = 26999;

    TIM2->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 = TIM_CR1_CEN;
}

void timer2_init()
{
    RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;
}
void init_led()
{
	/* ULED with a delay */
	//Send the clock to port D
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // Configure PD12 as output
    GPIOD->MODER &= ~(3U << (2 * LED_PIN));
    GPIOD->MODER |= (1U << (2 * LED_PIN));

    GPIOD->MODER &= ~(3U << (2 * GPIOD_PIN6));
    GPIOD->MODER |= (1U << (2 * GPIOD_PIN6));

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    init_led();

    timer2_init();

    timer2_Int_set();

    while (1)
    {
    }
}

