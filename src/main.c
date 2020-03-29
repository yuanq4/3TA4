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

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char lcd_buffer[6];    // LCD display buffer
char val[6];					//char array for tempreture value
char set[6];					//char array for setpoint value

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

ADC_HandleTypeDef    Adc_Handle;
ADC_ChannelConfTypeDef sConfig;
ADC_AnalogWDGConfTypeDef watchdog;


TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;
uint16_t Tim4_PrescalerValue;


uint32_t ADC1ConvertedValue=0; 


volatile double  set_pnt=23.5;

double measuredTemp; 


enum state {SetPoint = 0, display = 1, FanLevel = 2};
enum state currentS;
enum state prevS;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void ADC_Config(void);
void Watchdog_Config(void);
void Watchdog_ChangeSP(double sp);
void TIM4_Config(void);
void TIM4_OC_Config(uint32_t dutyCycle);
void TurnOffFanLevel(void);
double GetTemp(uint32_t ADC1ConvertedValue);
int GetThreshold(volatile double SP);


//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


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

	SystemClock_Config();   
	
	HAL_InitTick(0x0000); // set systick's priority to the highest.

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LCD_GLASS_Init();

	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	ADC_Config();
	Watchdog_Config();
	TIM4_Config();
	
	currentS = display;

	//When RESET is hit, display current temp
	HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
	HAL_ADC_Stop_DMA(&Adc_Handle);
	measuredTemp=GetTemp(ADC1ConvertedValue);
	sprintf((char*)val, "%f", measuredTemp);
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)val);
	HAL_Delay(2000);
	
	
 	
  while (1)
  {
		switch (currentS){
			case display: //check the temperature and display it
				BSP_LCD_GLASS_Clear();
				HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
				HAL_ADC_Stop_DMA(&Adc_Handle);
				measuredTemp=GetTemp(ADC1ConvertedValue);
				sprintf((char*)val, "%f", measuredTemp);
				BSP_LCD_GLASS_DisplayString((uint8_t*)val);
				HAL_Delay(1000);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"TEMP");
				HAL_Delay(1000);
					break;
			
			
			case SetPoint: //setting the setpoint
				sprintf((char*)set, "%f", set_pnt);
				BSP_LCD_GLASS_DisplayString((uint8_t*)set);
				
					break;
			
			
			case FanLevel: //displaying the temp to see which level to set the FanLevel at
				HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
				HAL_ADC_Stop_DMA(&Adc_Handle);
				measuredTemp=GetTemp(ADC1ConvertedValue);
				sprintf((char*)val, "%f", measuredTemp);
				BSP_LCD_GLASS_DisplayString((uint8_t*)val);
				
				if (measuredTemp-set_pnt<0)								 //if temp below setpoint
				{
					BSP_LCD_GLASS_Clear();
					TurnOffFanLevel();
					currentS=display; 												//switch back to temp displaying
				}
				else if (measuredTemp-set_pnt<1)
				{
					BSP_LCD_GLASS_Clear();
					TIM4_OC_Config(6); 												//17% power level for fan
					currentS=display; 												//switch backto display
				}
				else if (measuredTemp-set_pnt<2)
				{
					BSP_LCD_GLASS_Clear();
					TIM4_OC_Config(3); 												//33% power level for fan
					currentS=display; 												//switch back to temp displaying
				}
				else if (measuredTemp-set_pnt<4)
				{
					BSP_LCD_GLASS_Clear();
					TIM4_OC_Config(2); 												//50% power level for fan
					currentS=display; 												//switch back to temp displaying
				}
				else if (measuredTemp-set_pnt>=4)
				{
					BSP_LCD_GLASS_Clear();
					TIM4_OC_Config(1); 												//100% power level for fan
					currentS=display; 												//switch back to temp displaying			
				}
				break;
		}
			
	}

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
			case GPIO_PIN_0: 																		//select botton
				if (currentS == display || currentS==FanLevel){ 
							BSP_LCD_GLASS_Clear();
							prevS = currentS; 													//store currentS in to prevS
							currentS = SetPoint;
							BSP_LCD_GLASS_DisplayString((uint8_t*)"SET");
							HAL_Delay(1000);
				}
				else{ 																						//switch back to the previous state
							currentS = prevS;
				}		
						break;
			case GPIO_PIN_1:     																//left button						
							
							break;
			case GPIO_PIN_2:    																//right button	
						
							break;
			case GPIO_PIN_3:    																//up button
							if(currentS == SetPoint){
							BSP_LCD_GLASS_Clear();
							set_pnt = set_pnt + 0.5;										//increase setpoint
							Watchdog_ChangeSP(set_pnt);
							break;
							}
			
			case GPIO_PIN_5:    																//down button				
						if(currentS == SetPoint){
							BSP_LCD_GLASS_Clear();
							set_pnt = set_pnt - 0.5;										//decrease setpoint
							Watchdog_ChangeSP(set_pnt);
							break;
							}
							break;
			
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
	//BSP_LED_On(LED5); //don't actually need this
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
	
	__HAL_TIM_SET_COUNTER(htim, 0x0000);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
		if (currentS==display) //don't want to set it to FanLevel mode if current in setpoint
		{
			currentS=FanLevel;
		}
}

void ADC_Config(void)
{
	Adc_Handle.Instance = ADC1;
	if (HAL_ADC_DeInit(&Adc_Handle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }
	
	Adc_Handle.Init.ClockPrescaler 				= ADC_CLOCK_ASYNC_DIV1;
	Adc_Handle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  Adc_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  Adc_Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  Adc_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  Adc_Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  Adc_Handle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  Adc_Handle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.DiscontinuousConvMode = ENABLE;                       /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  Adc_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  Adc_Handle.Init.DMAContinuousRequests = DISABLE;                        /* DMA circular mode selected */
  Adc_Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  Adc_Handle.Init.OversamplingMode      = DISABLE;
	
	/* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&Adc_Handle) != HAL_OK)
  {
    Error_Handler();
  }
	
	sConfig.Channel      = ADC_CHANNEL_6;                /* Sampled channel number 6 */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;               
     
	if (HAL_ADC_ConfigChannel(&Adc_Handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
}





void Watchdog_Config(void)
{
	watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  watchdog.Channel = ADC_CHANNEL_6;
  watchdog.ITMode = ENABLE;
  watchdog.HighThreshold = GetThreshold(set_pnt);
  watchdog.LowThreshold = 0;
	
  if (HAL_ADC_AnalogWDGConfig(&Adc_Handle, &watchdog) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

void Watchdog_ChangeSP(double sp)
{
	//convert threshold to a value the ADC watchdog will actually understand
	int threshold = GetThreshold(sp);
	
	watchdog.HighThreshold = threshold;
	if (HAL_ADC_AnalogWDGConfig(&Adc_Handle, &watchdog) != HAL_OK) 											//reconfigure watchdog
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Set variable to report analog watchdog out of window status to main      */
  /* program. In this case, start the FanLevel mode                                */
	if (currentS == display)
	{
		currentS = FanLevel;
	}
	
	
}

void  TIM4_Config(void)
{
	/* Compute the prescaler value to have TIM4 counter clock equal to 50 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 50000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = 199;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4_Handle.Init.RepetitionCounter = 0;
	
	if (HAL_TIM_PWM_Init(&Tim4_Handle) != HAL_OK)
	{
		Error_Handler();
	}
}


void  TIM4_OC_Config(uint32_t dutyCycle)
{
		Tim4_OCInitStructure.OCMode=TIM_OCMODE_PWM1;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		Tim4_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
		Tim4_OCInitStructure.OCNPolarity=TIM_OCNPOLARITY_HIGH;
		Tim4_OCInitStructure.OCNPolarity=TIM_OCNIDLESTATE_RESET;
		Tim4_OCInitStructure.OCIdleState=TIM_OCIDLESTATE_RESET;
		
		Tim4_OCInitStructure.Pulse=(uint32_t)(200)/dutyCycle; 									//this is for duty cycle
	
		HAL_TIM_PWM_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1);
	
	 	HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1); 				
		
}

void TurnOffFanLevel(void)
{
	Tim4_OCInitStructure.Pulse=(uint32_t)(0); //set the pulse as 0 so the FanLevel isn't getting power
	HAL_TIM_PWM_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
	HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1); 
}

double GetTemp(uint32_t ADC1ConvertedValue)
{
	//TF is 3 for circuit - divide input by 3 to get temperature sensor output
	//Sensor is going up by 10mV for each degree - divide by .01 to get V
	//Resoltion is 12 bits - 4095 values
		//On 3V division, each output is 3/4095 volts, so multiply by 3/4095
	return (ADC1ConvertedValue)*(3.0/4095.0)/(3.0*.01);
}

int GetThreshold(volatile double SP)
{
	int threshold = SP/((3.0/4095.0)/(3.0*.01));
	return threshold;
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
