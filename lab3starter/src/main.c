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


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

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
I2C_HandleTypeDef  pI2c_Handle;
HAL_StatusTypeDef EE_status;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

typedef enum stateType{getTime,setState,getLast,getDate} stateType;
typedef enum setType{second,minute,hour,wkd,date,month,year} setType;
typedef enum weekdays{SUNDAY=1,MONDAY=2,TUESDAY=3,WEDNESDAY=4,THURSDAY=5,FRIDAY=6,SATURDAY=7} weekday;
stateType state=getTime;
setType stState=second;

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range
 

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  //   
char datestring[6]={0};
char dString[4]={0}; //String stores that the changing unit would be printed

uint8_t dd=0x01, mo=0x0A, yy=0x18, ss=0x00, mm=0x00, hh=0x00; // for weekday, day, month, year, second, minute, hour
weekday wd=0x01;

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed;  // button pressed 
__IO uint8_t sel_held;   // if the selection button is held for a while (>800ms)
__IO uint8_t sLast;				// to ensure user does not save the same second twice into ee_prom, basically a software debounce
__IO uint8_t currentTime;
uint8_t allowchange = 0;		//Use it to allow the time and date to be changed when sel pressed
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void RTC_Config(void);
void RTC_Update(void);						// to update RTC in real time
void RTC_AlarmAConfig(void);
void RTC_DateShow(RTC_HandleTypeDef *hrtc);
void RTC_TimeShow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef EE_RecordTime(RTC_HandleTypeDef *hrtc);
void EE_DisplayTime(RTC_HandleTypeDef *hrtc);


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

	leftpressed=0;
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	HAL_Delay(1000);

	memLocation=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,0x0000);
//configure real-time clock
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	RTC_AlarmA_IT_Enable(&RTCHandle);
	
	I2C_Init(&pI2c_Handle);
	


//*********************Testing I2C EEPROM------------------
/*
	//the following variables are for testging I2C_EEPROM
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	

*/
//******************************testing I2C EEPROM*****************************	
		

  /* Infinite loop */
  while (1)
  {
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>800){	
									selpressed=0;
									state=getDate;
									break;
						}
					}
			}

			switch (state) {
						case getTime:												// default state: shows timer going up in 1 sec increments
								RTC_AlarmA_IT_Enable(&RTCHandle);
								if (selpressed!=0) {						// if external button 1 is pressed, record time onto EEprom with function EE_RecordTime
										selpressed=0;
										RTC_AlarmA_IT_Disable(&RTCHandle);
										if (sLast == RTC_TimeStructure.Seconds){					// software debounce; will not save multiple times in one second
												state=getTime;
											break;
										}
										EE_RecordTime(&RTCHandle);
										BSP_LCD_GLASS_Clear();
										BSP_LCD_GLASS_DisplayString((uint8_t*)"SAVED");
										HAL_Delay(1000);
				
										sLast = RTC_TimeStructure.Seconds;								// record time and saves the time you saved into variable sLast, if sLast == current time on RTC then break
										state=getTime;
										break;
								}
								else if (rightpressed==1) {									// if right button 2 is pressed, enter set mode and temporarily disable AlarmA
										rightpressed=0;
										RTC_AlarmA_IT_Disable(&RTCHandle);
										BSP_LCD_GLASS_Clear();
										BSP_LCD_GLASS_DisplayString((uint8_t*)"SET");
										state=setState;
										break;
								}
								else if (leftpressed==1){															// if left button is pressed, enter getLast state
										leftpressed=0;
										RTC_AlarmA_IT_Disable(&RTCHandle);
										BSP_LCD_GLASS_Clear();
										BSP_LCD_GLASS_DisplayString((uint8_t*)"SHOW");
										state=getLast;
										break;
								}
							break;
							
						case setState:												// time setting state: joystick buttons have differnet functions while in this state
							if (leftpressed==1){								// 	shifts 'cursor' between different setting states: secs,mins,hours,wkday,day,month,year
										leftpressed=0;
										switch (stState){
												case second:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"YEAR");
													stState = year;
													break;
												case minute:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"SECS");
													stState = second;
													break;
												case hour:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"MINS");
													stState = minute;
													break;
												case wkd:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"HOURS");
													stState = hour;
													break;
												case date:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"WKDAY");
													stState = wkd;
													break;
												case month:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"DATE");
													stState = date;
													break;
												case year:
													BSP_LCD_GLASS_Clear();
													BSP_LCD_GLASS_DisplayString((uint8_t*)"MONTH");
													stState = month;
													break;
											}
								}
								else if (selpressed==1){					//	increases/decreases value of state you're on
										selpressed=0;
										switch (stState){
												case second:
													BSP_LCD_GLASS_Clear();
													ss = (ss+1)%60;
													sprintf(dString,"%u",ss); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case minute:
													BSP_LCD_GLASS_Clear();
													mm = (mm+1)%60;
													sprintf(dString,"%u",mm); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case hour:
													BSP_LCD_GLASS_Clear();
													hh = (hh+1)%24;
													sprintf(dString,"%u",hh); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case wkd:
													BSP_LCD_GLASS_Clear();
													wd = (wd+1)%7;
													if (wd == 0){
															wd = 7;
													}
													sprintf(dString,"%u",wd); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case date:
													BSP_LCD_GLASS_Clear();
													// if (mo%12==4,6,9,11)  put check here for months w/ < 31 days
													dd = (dd+1)%31;
													if (dd == 0){
															dd = 31;
													}
													sprintf(dString,"%u",dd); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case month:
													BSP_LCD_GLASS_Clear();
													mo = (mo+1)%12;
													if (mo==0){
															mo = 12;
													}
													sprintf(dString,"%u",mo); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
												case year:
													BSP_LCD_GLASS_Clear();
													yy = (yy+1)%100;
													sprintf(dString,"%u",yy); 
													BSP_LCD_GLASS_DisplayString((uint8_t*)dString);
													break;
											}
								}
								else if (rightpressed==1){
										rightpressed=0;
										RTC_Update();							// update time before you go back to getTime
										RTC_Config();							// have to reconfig
										state=getTime;
										break;
								}
								break;
									
						case getLast:												// getLast state simply calls EE_DisplayTime
							EE_DisplayTime(&RTCHandle);
							if (leftpressed==1){
								leftpressed=0;
								state=getTime;
								break;
							}
							break;

						case getDate:
								RTC_DateShow(&RTCHandle);
								state = getTime;
								break;
					} //end of switch					
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

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
		//************students: need to complete the following lines******************************			
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
				
				RTCHandle.Init.AsynchPrediv = 127; 			// got this from example RTC_Alarm
				RTCHandle.Init.SynchPrediv = 249; 
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	
	
	
	//****3.***** init the time and date
				
				
 		/*****************Students: please complete the following lnes*****************************
		*/// ***************************************************************************************
				RTC_DateStructure.Year = yy;				// initial year/month/etc
				RTC_DateStructure.Month = mo;
				RTC_DateStructure.Date = dd;
				RTC_DateStructure.WeekDay = wd;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
  
				RTC_TimeStructure.Hours = hh;  
				RTC_TimeStructure.Minutes = mm;
				RTC_TimeStructure.Seconds = ss;
				//RTC_TimeStructure.TimeFormat = ???
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  
 // ********************************************************************************/



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	//**************students:  you need to set the followint two lines****************
	//********************************************************************************
	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
	
	
	//********************************************************************************/			
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)		/// interupts
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
							if ((HAL_GetTick()-SEL_Pressed_StartTick)<=800){
								selpressed=1;
							}
						break;	
			case GPIO_PIN_1:     //left button						
							leftpressed=1;
							break;
			case GPIO_PIN_2:    //right button						 
							rightpressed=1;			
							break;
			case GPIO_PIN_3:    //up button							
							uppressed=1;
							break;
			case GPIO_PIN_5:    //down button						
							downpressed=1;
							break;
			default://
						//default
							break;
	  } 
}

void RTC_TimeShow(RTC_HandleTypeDef *hrtc){					// to show time
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);			// GetTime locks value in shadow register
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);			// have to GetDate to unlock
	
	ss=RTC_TimeStructure.Seconds;
	mm=RTC_TimeStructure.Minutes;
	hh=RTC_TimeStructure.Hours;
	
	sprintf(timestring,"%02u%02u%02u",hh,mm,ss); 
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);		// displays time string to LCD
	//EE_RecordTime(&RTCHandle);
	//currentTime = RTC_TimeStructure.Seconds;
	
}

void RTC_DateShow(RTC_HandleTypeDef *hrtc)
{
	RTC_AlarmA_IT_Disable(&RTCHandle);
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);

	wd=RTC_DateStructure.WeekDay;
	dd=RTC_DateStructure.Date;
	mo=RTC_DateStructure.Month;
	yy=RTC_DateStructure.Year;

	sprintf(datestring," %02u/%02u/%02u ",dd,mo,yy);
	
	switch(wd) {
		case SUNDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"SUN");
			break;
		case MONDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"MON");
			break;
		case TUESDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"TUES");
			break;
		case WEDNESDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"WED");
			break;
		case THURSDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"THUR");
			break;
		case FRIDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"FRI");
			break;
		case SATURDAY:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"SAT");
			break;
	}
	HAL_Delay(500);
	
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)datestring,1,500);
	RTC_AlarmA_IT_Enable(&RTCHandle);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)			// after each alarm interrupt, toggle LED and clear screen/display time
{
  BSP_LED_Toggle(LED5);
	BSP_LCD_GLASS_Clear();
	RTC_TimeShow(hrtc);
}

HAL_StatusTypeDef EE_RecordTime(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_GetTime(hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);				// get current time
	HAL_RTC_GetDate(hrtc, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	ss=RTC_TimeStructure.Seconds;
	mm=RTC_TimeStructure.Minutes;
	hh=RTC_TimeStructure.Hours;
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, ss);			// check EE_status to ensure memory at memLocation is OK
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1, mm);
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+2, hh);
  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
	}
	
	if (memLocation < 0xFFFD)				// make sure you have at least 3 bytes in memory
	{
		memLocation=memLocation+3;		
		
		EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, 0x0000, memLocation); // write memory location to 0x0000 to remember last location

		if(EE_status != HAL_OK)
		{
			I2C_Error(&pI2c_Handle);
		}
	}
	else{
		memLocation=0x000A;					// if you don't have at least 3 bytes, restart from beginning
		EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, 0x0000, memLocation);	// update memlocation
  
		if(EE_status != HAL_OK)
		{
			I2C_Error(&pI2c_Handle);
		}
	}
	
	return EE_status;
}

void EE_DisplayTime(RTC_HandleTypeDef *hrtc)
{
	
	memLocation=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,0x0000);
	
	if (memLocation < 0x06)		//This would mean there haven't been 2 presses yet saved in eeprom
	{
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"TOOFEW");
	}
	else 
	{		
		HAL_Delay(1000);
		hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-1);	// DO NOT change memLocation value, we still want it pointing at latest timestamp
		mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-2);
		ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-3);
		sprintf(timestring,"%02u%02u%02u",hh,mm,ss);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"time1");
		HAL_Delay(500);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
		HAL_Delay(500);
		
		hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-4);
		mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-5);
		ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS,memLocation-6);
		sprintf(timestring,"%02u%02u%02u",hh,mm,ss);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)"time2");
		HAL_Delay(500);
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*)timestring);
		HAL_Delay(500);
	}
	
	//RTC_AlarmA_IT_Enable(hrtc);
}


void RTC_Update(void){									// update RTC
				RTC_DateStructure.Year = yy;
				RTC_DateStructure.Month = mo;
				RTC_DateStructure.Date = dd;
				RTC_DateStructure.WeekDay = wd;
				RTC_TimeStructure.Hours = hh;  
				RTC_TimeStructure.Minutes = mm;
				RTC_TimeStructure.Seconds = ss;
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
