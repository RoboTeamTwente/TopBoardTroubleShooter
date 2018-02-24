/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
//!!include this file see all //!! for explanation
#include "PuttyInterface/PuttyInterface.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//!!declare this struct
PuttyInterfaceTypeDef puttystruct;

int HALL_1 = 0;
int HALL_2 = 0;
int HALL_3 = 0;

int pa0=0,pa2=0,pa5=0;
int hall1=0,hall2=0,hall3=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//!! create similar function protype
void HandleCommand(char* input);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == Motor3_2_Pin){
		pa0=1;
	}
	if(GPIO_Pin == Motor2_2_Pin){
		pa2=1;
	}
	if(GPIO_Pin == Motor1_2_Pin){
		pa5=1;
	}
}

void set_hallValue(int hall1,int hall2,int  hall3){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,hall1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,hall2);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,hall3);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  //!! assign function pointer and init
  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  set_hallValue(hall1,hall2,hall3);
	  PuttyInterface_Update(&puttystruct);
	  uprintf("hall: %d %d %d \n\r",hall1,hall2,hall3);
	  uprintf("gpio :%d %d %d \n\r",
			  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3),
			  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4),
			  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6));
	  uprintf("pwm  :%d %d %d\n\r",pa0,pa2,pa5);
	 int pa3=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	 int pa4=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);
	 int pa6=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
//	 if(pa3==1 && pa4==0 &&pa6==1){uprintf("state1 correct\n\r");}
//	 if(pa3==0 && pa4==0 &&pa6==1){uprintf("state2 correct\n\r");}
//	 if(pa3==0 && pa4==1 &&pa6==1){uprintf("state3 correct\n\r");}
//	 if(pa3==0 && pa4==1 &&pa6==0){uprintf("state4 correct\n\r");}
//	 if(pa3==1 && pa4==1 &&pa6==0){uprintf("state5 correct\n\r");}
//	 if(pa3==1 && pa4==0 &&pa6==0){uprintf("state6 correct\n\r");}
//	 if(pa3==0 && pa4==0 &&pa6==0){uprintf("state are all zero\n\r");}
	 	 if(pa3==1 && pa4==0 &&pa6==0  &&pa0==1){uprintf("state1 correct\n\r");}
		 if(pa3==0 && pa4==0 &&pa6==1 &&  pa0==1){uprintf("state2 correct\n\r");}
		 if(pa3==0 && pa4==0 &&pa6==1 && pa2==1){uprintf("state3 correct\n\r");}
		 if(pa3==0 && pa4==1 &&pa6==0 && pa2==1){uprintf("state4 correct\n\r");}
		 if(pa3==0 && pa4==1 &&pa6==0 && pa5==1){uprintf("state5 correct\n\r");}
		 if(pa3==1 && pa4==0 &&pa6==0 && pa5==1){uprintf("state6 correct\n\r");}
		 if(pa3==0 && pa4==0 &&pa6==0 ){uprintf("state are all zero\n\r");}


	  pa0=0;pa2=0;pa5=0;
	  HAL_Delay(400);
//	  PuttyInterface_Update(&puttystruct);
//	  uprintf("%d %d %d \n\r%d %d %d \n\r \n\r%d %d %d \n\r\n\r\n\r\n\r",
//		 HAL_GPIO_ReadPin(Motor1_1_GPIO_Port, Motor1_1_Pin),
//		 HAL_GPIO_ReadPin(Motor2_1_GPIO_Port, Motor2_1_Pin),
//		 HAL_GPIO_ReadPin(Motor3_1_GPIO_Port, Motor3_1_Pin),
//		 HAL_GPIO_ReadPin(Motor1_2_GPIO_Port, Motor1_2_Pin),
//		 HAL_GPIO_ReadPin(Motor2_2_GPIO_Port, Motor2_2_Pin),
//		 HAL_GPIO_ReadPin(Motor3_2_GPIO_Port, Motor3_2_Pin),
//		 HALL_1, HALL_2, HALL_3);
//	  HAL_Delay(400);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //!!run the update function for everything that needs to be done outside interrupts

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//!!this function will receive the commands when the are complete strcmp(input, "command") will return zero if "command" is equal to input
void HandleCommand(char* input){
	if(!strcmp(input, "start")){
		uprintf("started;>)\n\r");
	}else if(!strcmp(input, "stop")){
		uprintf("stopped\n\r");
	}else if(!strcmp(input, "1")){
		hall1=1;hall2=0;hall3=1;
	}else if(!strcmp(input, "2")){
		hall1=0;hall2=0;hall3=1;
	}else if(!strcmp(input, "3")){
		hall1=0;hall2=1;hall3=1;
	}else if(!strcmp(input, "4")){
		hall1=0;hall2=1;hall3=0;
	}else if(!strcmp(input, "5")){
		hall1=1;hall2=1;hall3=0;
	}else if(!strcmp(input, "6")){
		hall1=1;hall2=0;hall3=0;
	}
//	if(!strcmp(input,"00")){
//		hall1=0;hall2=0;hall3=1;
//		HAL_Delay(1000);
//		hall1=0;hall2=0;hall3=1;
//		HAL_Delay(1000);
//		hall1=0;hall2=1;hall3=1;
//		HAL_Delay(1000);
//		hall1=0;hall2=1;hall3=0;
//		HAL_Delay(1000);
//		hall1=1;hall2=1;hall3=0;
//		HAL_Delay(1000);
//		hall1=1;hall2=0;hall3=0;
//		HAL_Delay(1000);
//	}else if(!strcmp(input, "stop")){
//		uprintf("stopped\n\r");
//	}
}
//!! function gets called from usbd_cdc_if but needs to be added there in both c and h file
void USB_RxCallBack(uint8_t* Buf, uint32_t Len){
	puttystruct.huart2_Rx_len = Len;
	memcpy(puttystruct.small_buf, Buf, Len);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
