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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义中断引脚（PA15）
#define SWITCH_Pin       GPIO_PIN_15
#define SWITCH_GPIO_Port GPIOA
// 定义LED点亮/关闭电平
#define LED_ON           GPIO_PIN_RESET
#define LED_OFF          GPIO_PIN_SET
// 按键消抖时间
#define KEY_DEBOUNCE_MS  20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// 流水灯控制标志：0=关闭（LED全灭），1=运行（流水灯）
uint8_t flag = 0;  
// 中断消抖时间记录
uint32_t last_key_time = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LED_All_Off(void);  // 声明LED全灭函数
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  GPIO中断回调函数：处理PA15按键中断，更新流水灯标志（带消抖）
  * @param  GPIO_Pin: 触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 仅处理PA15引脚的中断（过滤其他中断）
    if (GPIO_Pin == SWITCH_Pin)
    {
        // 按键消抖：当前时间与上次触发时间间隔超过消抖时间，才执行逻辑
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_key_time) > KEY_DEBOUNCE_MS)
        {
            // 读取PA15当前电平，更新flag（0=低电平，1=高电平）
            flag = HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin);
            
            // 若flag=0（PA15低电平），立即关闭所有LED（避免残留点亮状态）
            if (flag == 0)
            {
                LED_All_Off();
            }
            
            // 更新上次中断时间（用于下次消抖判断）
            last_key_time = current_time;
        }
        
        // 强制清除中断标志（防止中断重复触发）
        __HAL_GPIO_EXTI_CLEAR_IT(SWITCH_Pin);
    }
}

/**
  * @brief  关闭所有LED（统一控制，确保PA1、PB1、PA5全灭）
  * @retval None
  */
void LED_All_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED_OFF);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, LED_OFF);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LED_OFF);
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
  /* USER CODE BEGIN 2 */
  // 初始化：关闭所有LED（与flag初始值0保持一致）
  LED_All_Off();
  // 初始化按键消抖时间（避免上电后首次中断误触发）
  last_key_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 根据flag状态执行对应逻辑（所有分支均在循环内，确保可执行）
    if (flag == 1)  // PA15高电平：流水灯运行
    {
      /* 点亮PA1，关闭其他LED */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED_ON);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, LED_OFF);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LED_OFF);
      HAL_Delay(100);  // 延时0.1秒
      
      /* 点亮PB1，关闭其他LED */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED_OFF);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, LED_ON);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LED_OFF);
      HAL_Delay(100);  // 延时0.1秒
      
      /* 点亮PA5，关闭其他LED */
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED_OFF);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, LED_OFF);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LED_ON);
      HAL_Delay(100);  // 延时0.1秒
    }
    else if (flag == 0)  // PA15低电平：保持LED全灭
    {
      LED_All_Off();  // 即使中断未触发，循环内也会持续确保LED全灭
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
    // 错误状态提示：3个LED快速闪烁（区别于正常流水灯，便于排查问题）
    LED_All_Off();
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, LED_ON);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, LED_ON);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, LED_ON);
    HAL_Delay(300);
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
