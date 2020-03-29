/**
******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************


*/




/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

TIM_HandleTypeDef    Tim3_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure;
uint16_t Tim3_PrescalerValue;
__IO uint16_t Tim3_CCR; // the pulse of the TIM3

static GPIO_InitTypeDef  GPIO_InitStruct;


char lcd_buffer[6];    // LCD display buffer

enum STEPMODES {FULL=1, HALF=0};
enum STEPMODES step = FULL; //start in fullstepping mode
uint16_t direction = 1;
volatile uint16_t 	Tim3_CCR = 10000;

uint8_t WINDING = 0; //indicator for which winding

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void TIM3_OC_Config(void);
void GPIO_Init(void);
void GPIO_Output_Clear(void);
void TurnOn(uint16_t PIN);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	HAL_Init();
	

	SystemClock_Config();   //sysclock is 80Hz. HClkm apb1 an apb2 are all 80Mhz.
  
	HAL_InitTick(0x0000); // set systick's priority to the highest.

	BSP_LED_Init(LED4);
	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI); 	
	
	GPIO_Init();

	TIM3_Config();
	TIM3_OC_Config();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB5");
 	
  while (1)
  {
		/*
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"SET");
		BSP_LED_On(LED4);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay(1000);
		
		BSP_LCD_GLASS_DisplayString((uint8_t*)"RESET");
		BSP_LED_Off(LED4);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_Delay(1000);
	*/
		
	} //end of while 1

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button	- switch between full & half				
						step=(step+1)%2;
						break;	
			case GPIO_PIN_1:     //left button - 
							if (direction==1){
									direction = 0;
									BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)"CCW");
							}
							else{
									direction = 1;
									BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)"CW");
							}
							break;
			case GPIO_PIN_2:    //right button						 
						
							break;
			case GPIO_PIN_3:    //up button - increase speed
							BSP_LCD_GLASS_Clear();
							if(Tim3_CCR>= 4000){
								Tim3_CCR -= 1000;
								TIM3_OC_Config();
								TIM3_Config();
							}
							break;
			
			case GPIO_PIN_5:    //down button - decrease speed			
							BSP_LCD_GLASS_Clear();
							if(Tim3_CCR<=20000)	{								
								Tim3_CCR += 1000;
								TIM3_OC_Config();
								TIM3_Config();
							}
							break;
			default://
						//default
						break;
	  } 
}

void  TIM3_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; 
	Tim3_Handle.Init.Period = Tim3_CCR;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK)
  //{
    /* Initialization Error */
    //Error_Handler();
  //} 
}



void  TIM3_OC_Config(void)
{
		Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim3_OCInitStructure.Pulse=Tim3_CCR;
		Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim3_Handle); 
	
		HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1); 
	
	 	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1); 
				
		
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
if (direction ==1){
	if(step==HALF)
	{
		switch(WINDING){
			case (0):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					WINDING=11;
					break;
			case (11):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					TurnOn(GPIO_PIN_11);
					WINDING=1;
					break;
			case (1):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_11);
				WINDING=21;
				break;
			case (21):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_11);
				TurnOn(GPIO_PIN_12);
				WINDING=2;
				break;
			case (2):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_12);
				WINDING=31;
				break;
			case (31):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_12);
				TurnOn(GPIO_PIN_15);
				WINDING=3;
				break;
			case (3):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_15);
				WINDING=4;
				break;
			case (4):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_15);
				TurnOn(GPIO_PIN_10);
				WINDING=0;
				break;
		}
	}
	
	if(step==FULL)
		{
			switch (WINDING)
			{
				case (0):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					WINDING++;
					break;

				case (1):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_11);
					WINDING++;
					break;
				
				case (2):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_12);
					WINDING++;
					break;
				
				case (3):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_15);
					WINDING=0;
					break;
				default: //if WINDING isn't 0-3, may have been in half-stepping mode; reset back to 0
					WINDING=0;
					break;
			}
		}	
				
}

else{
	if(step==HALF)
	{
		switch(WINDING){
			case (0):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					WINDING=11;
					break;
			case (11):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					TurnOn(GPIO_PIN_15);
					WINDING=1;
					break;
			case (1):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_15);
				WINDING=21;
				break;
			case (21):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_15);
				TurnOn(GPIO_PIN_12);
				WINDING=2;
				break;
			case (2):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_12);
				WINDING=31;
				break;
			case (31):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_12);
				TurnOn(GPIO_PIN_11);
				WINDING=3;
				break;
			case (3):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_11);
				WINDING=4;
				break;
			case (4):
				GPIO_Output_Clear();
				TurnOn(GPIO_PIN_11);
				TurnOn(GPIO_PIN_10);
				WINDING=0;
				break;
		}
	}
	
	if(step==FULL)
		{
			switch (WINDING)
			{
				case (0):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_10);
					WINDING++;
					break;

				case (1):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_15);
					WINDING++;
					break;
				
				case (2):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_12);
					WINDING++;
					break;
				
				case (3):
					GPIO_Output_Clear();
					TurnOn(GPIO_PIN_11);
					WINDING=0;
					break;
				default: //if WINDING isn't 0-3, may have been in half-stepping mode; reset back to 0
					WINDING=0;
					break;
			}
		}
}

}
void GPIO_Init(void)
{
		__HAL_RCC_GPIOE_CLK_ENABLE();
	
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
}

void GPIO_Output_Clear(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_RESET);
	BSP_LED_Off(LED4);
	BSP_LED_Off(LED5);
}

void TurnOn(uint16_t PIN)
{
	if (PIN==GPIO_PIN_10)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	}
	else if (PIN==GPIO_PIN_11)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}
	else if (PIN==GPIO_PIN_12)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else if (PIN==GPIO_PIN_15)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	}
	
	//HAL_GPIO_WritePin(GPIOE, PIN,GPIO_PIN_SET);
}

static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
