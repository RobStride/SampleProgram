/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "canDisposition.h"
#include "Robstrite.h"
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
CAN_TxHeaderTypeDef CAN_TxHeaderStrue; // ����CAN���ͱ�ͷ�ṹ��
CAN_RxHeaderTypeDef CAN_RxHeaderStrue; // ����CAN���ձ�ͷ�ṹ��
uint8_t pRxdata[8], pTxdata[8]; // ������պͷ������ݻ���
RobStrite_Motor RobStrite_01(0x01);
//Can_Struct Can_Robstrite;


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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // ���ý���FIFO0��Ϣ�����ж�
	HAL_CAN_Start(&hcan1); // ����CAN

	RobStrite_01.RobStrite_Get_CAN_ID(); //�����ȡ�豸ID��MCU���밴����Ҫʹ��
	
	RobStrite_01.Set_CAN_ID(0x01);				//�������CANID���밴����Ҫʹ��
	
	RobStrite_01.Set_ZeroPos();					//���õ�ǰλ��Ϊ�����е��λ���밴����Ҫʹ��
	
	RobStrite_01.Enable_Motor();					//���ʹ��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//*********�˿�ģʽ*************//
		float T = 0.1; 			//Ť��
		float Angle = 0.5; 	//Ҫת����λ��
		float Speed = 0.1;	//Ŀ���ٶ�
		float Pacceleration = 1; //���ٶ�
		float Kp = 0.1;			//Kp
		float Kd = 0.1;			//Kd
		RobStrite_01.RobStrite_Motor_move_control(T, Angle, Speed, Kp, Kd);
		
		
		//*********λ��ģʽ*************//	
		RobStrite_01.RobStrite_Motor_Pos_control(Speed, Pacceleration, Angle);
		
		
		
		//*********�ٶ�ģʽ*************//	
		float Limit_cur  = 0.1; //���Ƶ���
		float Cacceleration = 1; //���ٶ�
		RobStrite_01.RobStrite_Motor_Speed_control(Speed, Cacceleration, Limit_cur);
		
		
		//*********����ģʽ*************//	
		float current  = 0.1;	//Ŀ�����
		RobStrite_01.RobStrite_Motor_current_control(current);
		
		
		RobStrite_01.Disenable_Motor(0);	//���ʧ�ܣ�����ʹ��
		

		HAL_Delay(2);		
		
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
  RCC_OscInitStruct.PLL.PLLM = 6;
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

CAN_RxHeaderTypeDef RXHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)					//中断回调，当我认定的邮箱CAN口对应的邮箱有数据进入，则进入中�?		
{

	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK)
	{
		if (RXHeader.IDE == CAN_ID_EXT)
		{
				RobStrite_01.RobStrite_Motor_Analysis(RxData, RXHeader.ExtId);
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
