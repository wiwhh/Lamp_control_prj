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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
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
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessCanMessage(CAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]);
extern CAN_HandleTypeDef hcan1;
// 定时器句柄
TimerHandle_t left_turn_signal_timer;
TimerHandle_t right_turn_signal_timer;

// 左转向灯定时器回调函数
void left_turn_signal_timer_callback(TimerHandle_t xTimer) {
    static BaseType_t xLightOn = pdFALSE;
    if (xLightOn) {
        // 关闭车灯的具体实现
        printf("Blink off\n");
        LED3_ON;
        xLightOn = pdFALSE;
    } else {
        // 打开车灯的具体实现
        printf("Blink on\n");
        LED3_OFF;
        xLightOn = pdTRUE;
    }
}

// 右转向灯定时器回调函数
void right_turn_signal_timer_callback(TimerHandle_t xTimer) {
    static BaseType_t xLightOn = pdFALSE;
    if (xLightOn) {
        // 关闭车灯的具体实现
        printf("Blink off\r\n");
        LED4_ON;
        xLightOn = pdFALSE;
    } else {
        // 打开车灯的具体实现
        printf("Blink on\r\n");
        LED4_OFF;
        xLightOn = pdTRUE;
    }
}
void CAN_SendMessageTask(void *argument) {
    CAN_TxHeaderTypeDef TxHeader;
    TickType_t xLastWakeTime;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20毫秒
    // 初始化变量，存储当前时钟节拍计数??
    xLastWakeTime = xTaskGetTickCount();

    // 配置发???消息头
    TxHeader.StdId = 0x00;  // 设置标准ID
    TxHeader.ExtId = 0x12345678;   // 设置扩展ID
    TxHeader.IDE = CAN_ID_EXT;  // 使用标准ID
    TxHeader.RTR = CAN_RTR_DATA;  // 数据??
    TxHeader.DLC = 8;  // 数据长度

    while (1)
    {
      /* code */
      // 设置发???数??
    TxData[0] = K1_State;
    TxData[1] = K2_State;
    TxData[2] = K3_State;
    TxData[3] = K4_State;

    // 发???消??
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
      printf("send message failed\r\n");
        // 发???失败处??
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void CanReceiveTask(void *argument) {
    CAN_RxHeaderTypeDef RxHeader;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1毫秒
    TickType_t xLastWakeTime;
    uint8_t RxData[8];
    for(;;) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            // 处理接收到的CAN消息
            ProcessCanMessage(RxHeader, RxData);
        }
        else
        {
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ProcessCanMessage(CAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]) {
    // 根据ID和数据处理不同的灯控命令
    static uint8_t K3_last_state = 0, K4_last_state = 0;
    uint8_t K3_state, K4_state;
    switch (RxHeader.ExtId) {
        case 0x12345678:
            // printf("0x12345678:");
            // for(i = 0;i<8;i++)
            // {
            //   printf("%#X ",RxData[i]);
            // }
            // printf("\r\n");
            // 远光灯控??
            if (RxData[0] == 0x00) {
                LED1_ON;
            } else {
                LED1_OFF;
            }
            if (RxData[1] == 0x00) {
                LED2_ON;
            } else {
                LED2_OFF;
            }
            K3_state = (RxData[2] == 0x00) ? 0 : 1;
            if (K3_state != K3_last_state) {
                if (K3_state == 0) {
                    xTimerStart(left_turn_signal_timer, 0);
                } else {
                    LED3_OFF;
                    xTimerStop(left_turn_signal_timer, 0);
                }
                K3_last_state = K3_state;
            }

            // 控制右转向灯
            K4_state = (RxData[3] == 0x00) ? 0 : 1;
            if (K4_state != K4_last_state) {
                if (K4_state == 0) {
                    xTimerStart(right_turn_signal_timer, 0);
                } else {
                    LED4_OFF;
                    xTimerStop(right_turn_signal_timer, 0);
                }
                K4_last_state = K4_state;
            }
            break;
            break;
        default:
            printf("Error message!!\r\n");
            break;
        // 更多控制逻辑
    }
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("你好世界!\r\n");
  xTaskCreate(CAN_SendMessageTask, "CAN_SendMessageTask", 512, NULL, 5, NULL);
  xTaskCreate(CanReceiveTask, "CanReceiveTask", 512, NULL, 4, NULL);
  #define TURN_SIGNAL_BLINK_INTERVAL pdMS_TO_TICKS(500)
  left_turn_signal_timer = xTimerCreate("LeftTurnSignalTimer", TURN_SIGNAL_BLINK_INTERVAL, pdTRUE, (void *)0, left_turn_signal_timer_callback);
  right_turn_signal_timer = xTimerCreate("RightTurnSignalTimer", TURN_SIGNAL_BLINK_INTERVAL, pdTRUE, (void *)0, right_turn_signal_timer_callback);
  if (left_turn_signal_timer&&right_turn_signal_timer == NULL)
    {
        // 定时器创建失败处理
        printf("Timer creation failed.\n");
    }
    else
    {
        printf("Timer created successfully.\n");
    }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
