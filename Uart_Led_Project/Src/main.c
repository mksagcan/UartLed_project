/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "stdio.h"
	#include "stdlib.h"
	#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum led {Passive,Activate,StopMode,StartMode};
enum Communication{Communication_OFF,Communication_ON};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rxTempBuffer[10];
uint8_t rxIndex=0;
uint8_t *rxBuffer;
uint16_t ledOffTimeCount=0;
uint16_t ledOnTimeCount=0;
char compareString[]="=";
char *ptrTime1;
char *ptrTime2;
char stringVal[10];
uint8_t startStopFlag=0;
uint8_t ledActivePasive=0;
uint8_t LedMODE=3;
uint8_t DataComplate=0;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2) // if data is send another uart section, does not care data!
	{
	
		rxBuffer[rxIndex]=rxTempBuffer[0];
		if(rxBuffer[rxIndex]=='\0') //if all data came until NULL, compare string.
		{
			if((rxBuffer[2]=='o')||(rxBuffer[2]=='O')) //Control of STOP
			{
				startStopFlag=Communication_OFF;
				LedMODE=StopMode;               // interval 1 second blink led
				DataComplate=0;									// Stop Uart echo flag
			}
			else if((rxBuffer[2]=='a')||(rxBuffer[2]=='A')) // Control of START
			{
				startStopFlag=Communication_ON;
				LedMODE=StartMode;
				DataComplate=1;   //Uart echo for interval of 1 sn
			}
			else
			{
				if((rxBuffer[4]=='f')||(rxBuffer[4]=='F'))  // if ledoff data comes, get timer val!
				{
					ptrTime1=strstr((char *)rxBuffer,compareString); //Compare until =
					strcpy(stringVal,ptrTime1+1);   // Get value after '='
					ledOffTimeCount=atoi(stringVal);  //Convert led time 	
				}
				else if((rxBuffer[4]=='n')||(rxBuffer[4]=='N')) // if ledon data comes, get timer val!
				{
					ptrTime2=strstr((char *)rxBuffer,compareString); //Compare until =
					strcpy(stringVal,ptrTime2+1);   // Get value after '='
					ledOnTimeCount=atoi(stringVal);  //Convert led time to integer value	
				}			
			}
			rxIndex=0; //start over!
		}
		rxIndex++;
	
		HAL_UART_Receive_IT(&huart2,rxTempBuffer,1);  // created intterupt
	}
	
}
void StartDefaultTask(void const * argument);

uint8_t indx=0;

osThreadId defaultTaskHandle;
osThreadId Task2handler;
void task2_init (void const * argument);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,rxTempBuffer,1);  // created intterupt
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
  /* Start scheduler */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	osThreadDef(Task2,task2_init,osPriorityAboveNormal,0,128);  //UART TASK Created!
	
	Task2handler=osThreadCreate(osThread(Task2),NULL);
	
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// StartDefaultTask used as Led task!!!!!!!!!!!!!!!
		// task2 used as deciding uart task!!!!!!!!!!!!!!!
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
/* USER CODE BEGIN 4 */
void StartDefaultTask(void const * argument)       //LED TASK!
{
	static uint16_t ledTimeCount=0;
	static uint16_t ledStopTimeCount=0;
	
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		if(LedMODE==StopMode)
		{		
			ledStopTimeCount++;
			if((ledStopTimeCount==1000)&&(ledActivePasive==Passive))  //LED is ON interval of 1 second
			{
				HAL_GPIO_WritePin(GPIOC,LED_Pin,GPIO_PIN_RESET); //LED is ON
				ledStopTimeCount=0;
				ledActivePasive=Activate;
			}
			else if((ledStopTimeCount==1000)&&(ledActivePasive==Activate)) //LED is OFF interval of 1 second
			{
				HAL_GPIO_WritePin(GPIOC,LED_Pin,GPIO_PIN_SET);  //LED is OFF
				ledStopTimeCount=0;
				ledActivePasive=Passive;
			}
			ledTimeCount=0; //start over for UART StartMode		
		}
		else if(LedMODE==StartMode)
		{
			ledTimeCount++;
			if((ledOnTimeCount==ledTimeCount)&&(ledActivePasive==Passive))  //if LED is ON
			{
				HAL_GPIO_WritePin(GPIOC,LED_Pin,GPIO_PIN_RESET); //LED is ON
				ledTimeCount=0;
				ledActivePasive=Activate;
			}
			else if((ledOffTimeCount==ledTimeCount)&&(ledActivePasive==Activate)) //if LED is OFF
			{
				HAL_GPIO_WritePin(GPIOC,LED_Pin,GPIO_PIN_SET);  //LED is OFF
				ledTimeCount=0;
				ledActivePasive=Passive;
			}
		}

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}


void task2_init (void const * argument) //send start data echo interval of 1 second period
{
	
	while(1)
	{
		if(DataComplate==1) //if all data came until NULL, compare string.
		{
			HAL_UART_Transmit_IT(&huart2,(uint8_t *) "start \r\n",8);
		}
		osDelay(1000);
	}
	
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
