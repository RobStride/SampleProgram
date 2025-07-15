/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "RobStride.h"
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
/* USER CODE BEGIN PFP */
CAN_FilterTypeDef CAN_FilterStrue; 
CAN_TxHeaderTypeDef CAN_TxHeaderStrue; 
CAN_RxHeaderTypeDef CAN_RxHeaderStrue; 
uint8_t pRxdata[8], pTxdata[8]; 
RobStride_Motor RobStride_01(0x7F, false);

uint8_t mode = 0; 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterStrue.FilterBank = 0; 
  CAN_FilterStrue.FilterMode = CAN_FILTERMODE_IDMASK; 
  CAN_FilterStrue.FilterScale = CAN_FILTERSCALE_16BIT; 
  CAN_FilterStrue.FilterIdHigh = 0;
	CAN_FilterStrue.FilterIdLow = 0;
	CAN_FilterStrue.FilterActivation = ENABLE;
	CAN_FilterStrue.FilterFIFOAssignment = CAN_RX_FIFO0;
	
	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterStrue);
  HAL_CAN_Start(&hcan); //鍚姩CAN
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch(mode)
    {
        // ===== 普通模式接口 =====
        case 0: // 使能（普通模式）
            RobStride_01.Enable_Motor();
            break;
        case 1: // 失能（普通模式）
            RobStride_01.Disenable_Motor(1);
            break;
        case 2: // 运控模式
            HAL_Delay(5);
            RobStride_01.RobStride_Motor_move_control(5, 0, 0, 0.0, 0.0);
            break;
        case 3: // PP位置模式
            RobStride_01.RobStride_Motor_Pos_control(2.0, 2);
						HAL_Delay(5);
            break;
				case 4:	//CSP位置模式
						RobStride_01.RobStride_Motor_CSP_control(2.0, 2.0);
						HAL_Delay(5);
						break;
        case 5: // 速度模式
            RobStride_01.RobStride_Motor_Speed_control(3.5, 5.0);
						HAL_Delay(5);
            break;
        case 6: // 电流模式
            HAL_Delay(5);
            RobStride_01.RobStride_Motor_current_control(1.2);
            break;
        case 7: // 设置机械零点
            RobStride_01.Set_ZeroPos();
            break;
        case 8: // 读取参数
            RobStride_01.Get_RobStride_Motor_parameter(0x7014);
            break;
        case 9: // 设置参数
            RobStride_01.Set_RobStride_Motor_parameter(0x7014, 0.35f, Set_parameter);
            break;
        case 10: // 协议切换（如切MIT协议/Canopen/私有协议）
            RobStride_01.RobStride_Motor_MotorModeSet(0x02); // 0x02=MIT
            break;

        // ===== MIT模式接口（只能用MIT专用函数！） =====
        case 11: // MIT 使能
            RobStride_01.RobStride_Motor_MIT_Enable();
            break;
        case 12: // MIT 失能
            RobStride_01.RobStride_Motor_MIT_Disable();
            break;
        case 13: // MIT 综合控制
						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
            RobStride_01.RobStride_Motor_MIT_Enable();
            HAL_Delay(5);
            RobStride_01.RobStride_Motor_MIT_Control(0, 0, 0, 0, -1.0f);
            break;
        case 14: // MIT 位置控制
						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
						RobStride_01.RobStride_Motor_MIT_Enable();
            HAL_Delay(5);
            RobStride_01.RobStride_Motor_MIT_PositionControl(1.57f, 3.0f);
            break;
        case 15: // MIT 速度控制
						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x02);
						RobStride_01.RobStride_Motor_MIT_Enable();
            HAL_Delay(5);
            RobStride_01.RobStride_Motor_MIT_SpeedControl(4.5f, 3.2f);
            break;
        case 16: // MIT 零点设置（运行前需保证 MIT_Type != positionControl）
            RobStride_01.RobStride_Motor_MIT_SetZeroPos();
            break;
        case 17: // MIT 清错
            RobStride_01.RobStride_Motor_MIT_ClearOrCheckError(0x01);
            break;
        case 18: // MIT 设置电机运行模式
            RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
            break;
        case 19: // MIT 设置电机ID
            RobStride_01.RobStride_Motor_MIT_SetMotorId(0x05);
            break;
        case 20: //主动上报
            RobStride_01.RobStride_Motor_ProactiveEscalationSet(0x00);
            break;
        case 21: // 波特率修改
            RobStride_01.RobStride_Motor_BaudRateChange(0x01);
            break;
        case 22: // MIT 参数保存
            RobStride_01.RobStride_Motor_MotorDataSave();
            break;
				case 23: // MIT 协议切换（如切MIT协议/Canopen/私有协议）
						RobStride_01.RobStride_Motor_MIT_MotorModeSet(0x00);
						break;

        default:
            break;
    }		
		mode = 24;
		HAL_Delay(50);
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
}

/* USER CODE BEGIN 4 */
CAN_RxHeaderTypeDef RXHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)							
{

	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK)
	{
		if (RXHeader.IDE == CAN_ID_EXT)
		{
				RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.ExtId);
		}
		else
		{
				RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.StdId);
		}
	}	
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
