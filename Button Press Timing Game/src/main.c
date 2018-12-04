/**
  ******************************************************************************
  * @file    Lab2/main.c
  * @author  Ricardo Tan , Template by Robert Li
  * @version 
  * @date    August 2014
  * @brief   this starter project demonstrates: 
						1. timer configuration
						2. timer output compare interrupt set up
						3. LCD configuration
						4. EEPROM emulator config
						5. RNG config
	* @projectfunction  1.Upon reset signal LEDs should blink
2.After button press, LEDs should turn off for about 2 seconds(used random number generator) to ensure this
3.LEDs turn back on which in which a counter is started in which counts the reaction of the user between LED turn on to button press
(can detect cheat condition, when user presses the button before ready)
4.Display the results , and store best time in EEPROM
					
  
  ******************************************************************************
 * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h" 




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))    //see font.h, for defining LINE(X)




/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint16_t CCR1_Val = 500; //choose this number so to avoid loss of accuracy of prescale due to integer division
uint16_t PrescalerValueTIM3 = 0; //TIM3 interrupt used for timing ms
uint16_t PrescalerValueTIM2 =0; //TIM2 interrupt used for blinking leds at a given frequency
__IO uint8_t UBPressed = 0; //state of onboard push button , 1 if pressed 
__IO uint8_t extUBPressed=0; // state of external push button, 1 if pressed

uint8_t waitComplete; //is waiting time completed
uint32_t random; //random number generated for time
uint32_t waitTime; //wait time until LED turns back on again
uint32_t currentTime; //stores the time when the button is pressed
uint32_t bestTime; //stores the best reaction time
extern uint32_t msTicks; // counter for milliseconds 

uint16_t line;//line on the LCD display

char lcd_buffer[14];    // LCD display buffer


/* Virtual address defined by the user: 0xFFFF value is prohibited
 * This global variable stores the EEPROM addresses for NB_OF_VAR(=3) variables
 * where the user can write to. To increase the number of variables written to EEPROM
 * modify this variable and NB_OF_VAR in eeprom.h
 */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};//virtual address of the EEPROM
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0}; //where the corresponding data is actually stored, in this case 0x5555 data is stored in the first slot(VarDataTab[0])
//BEST time stored in 0x5555 of the virtual address


enum modes currentMode=IDLE; //the full enumeration in main.h so that can be used in stm32f4xx_it.c as well



/* Private function prototypes -----------------------------------------------*/
void PB_Config(void);
void LED_Config(void);
void TIM3_Config(void);//used for timing milliseconds
void TIM3_OCConfig(void);
void TIM2_Config(void);//used for the LED blink
void TIM2_OCConfig(void);


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);
void RNG_Config(void);
void external_button_config(void); // external button is connected to PC1



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
	
//initiate user button
  PB_Config();
	
	//Initiate external button PD1
	//external_button_config();

	//initiate LEDs and turn them on
  LED_Config();	
	
 

  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 0.5 MHz, the prescaler is computed as follows: //i believe this should be 0.5MHZ pg 587 standard peripheral lib manual
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /0.5 MHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 10.0 Hz
    ==> Toggling frequency = 5 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
		 ----------------------------------------------------------------------- */ 	
	
	//=======================Configure and init Timer======================
  /* Compute the prescaler value */
  PrescalerValueTIM3 = (uint16_t) ((SystemCoreClock / 2) / (500000)) - 1; //interrupt every 1ms(1000HZ)
	PrescalerValueTIM2 =  (uint16_t) ((SystemCoreClock / 2) / 500) - 1;  //interrupt every 1 s(1 HZ)


 /* TIM Configuration */
  TIM3_Config();
	TIM2_Config();

	// configure the output compare
	TIM3_OCConfig();
	TIM2_OCConfig();


	
//======================================configure and init LCD  ======================	
	 /* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	
//================EEPROM init====================================

/* Unlock the Flash Program Erase controller */
		FLASH_Unlock();
		/* EEPROM Init */
		EE_Init();

//============ Set up for random number generation==============
	RNG_Config();


	//with the default font, LCD can display  12 lines of chars, they are LINE(0), LINE(1)...LINE(11) 
	//with the default font, LCD can display  15 columns, they are COLUMN(0)....COLUMN(14)
		LCD_Clear(LCD_COLOR_WHITE);
			
		line=0;
		external_button_config();
		
  while (1){ 

		
		switch(currentMode){
			//the mode where the LED just blinks
			case IDLE:
					TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
					TIM_Cmd(TIM2, ENABLE);
				
			//if onboard button is pressed turn off the blinking LED, go to the WAIT state 		
			if (UBPressed == 1){
								UBPressed=0;
								TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
								TIM_Cmd(TIM2, DISABLE);
								msTicks=0;
								currentMode=WAIT;
					}
					break;
			
			//the mode where the LED is off for a random amount of time
			case WAIT:
					STM_EVAL_LEDOff(LED3);
					STM_EVAL_LEDOff(LED4);
					
					random = RNG_GetRandomNumber();
					random &= 0x000003FF; //only using the last 10 bits so that number produced 0-1023 which is a random number between 0 and 1 second 
					waitTime = 1000 + random; //this will add up to around 2 seconds
					TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
					TIM_Cmd(TIM3, ENABLE);
					
				//this satisfies requirements to prevent cheating and also to exit when an external push button is pressed, by returning to IDLE
					while(1){
						if (UBPressed == 1 || extUBPressed==1){
							UBPressed=0;
							extUBPressed=0;
							currentMode = IDLE;
							break;
							
						}
						//LED stays off while timer counter counts to specified random time
						else if (waitComplete == 1)
						{
							waitComplete=0;
							currentMode=START;
							msTicks =0;
							break;
						}
					}
					break;
					
			//the mode where LED turns back ON and counter begins
			case START:
					STM_EVAL_LEDOn(LED3);
					STM_EVAL_LEDOn(LED4);
					
					//when the user reacts to the LED turning on, save that reaction time
					if (UBPressed ==1){
						UBPressed =0;
						STM_EVAL_LEDOff(LED3);
						STM_EVAL_LEDOff(LED4);
						currentTime = msTicks;
						currentMode = DISPLAY;
					}
					//part of the requirement in which must go back to IDLE state when external button is pressed
					else if (extUBPressed==1){
						extUBPressed=0;
						currentMode =IDLE;
					}
					break;
			

			//the mode where best time and current time is displayed, and times placed in EEPROM
			case DISPLAY:
					//must clear LCD to update screen 
					LCD_Clear(LCD_COLOR_WHITE);
					
					//part of the requirement in which must go back to IDLE state when external button is pressed
					if (extUBPressed ==1){
						extUBPressed=0;
						currentMode =IDLE;
						break;
					}
					
					EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]); //IMPORTANT: Must call this or else wont get updated value for VarDataTab[0]
					if (currentTime < VarDataTab[0]){
						EE_WriteVariable(VirtAddVarTab[0], currentTime); //only stores when current time is the best time
						EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]); //the read the updated value 
						}
					//displays the data
					LCD_DisplayInt(1, 2, currentTime);
					LCD_DisplayInt(5, 2, VarDataTab[0]);
					LCD_DisplayString(4,2, (uint8_t *) "best time:");
					LCD_DisplayString(0,2, (uint8_t *) "current time:");
			
					currentMode= IDLE;
					break;
			
			
			default:
				break;
		}
	}//end of while

}

/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  No
  * @retval None
  */


void PB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

void LED_Config(void)
{
 /* Initialize Leds mounted on STM32F429-Discovery board */
  STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4); 

}

void TIM3_Config(void) 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//since TIMER 3 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//====================================================
	//Enable TIM3 global interrupt ====does this part need to be done before TIM_BaseStructure set up?
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//================================================
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR1_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValueTIM3;    //why all the example make this one equal 0, and then use 
					//function TIM_PrescalerConfig() to re-assign the prescaller value?
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//TIM_PrescalerConfig(TIM3, TIM3Prescaler, TIM_PSCReloadMode_Immediate);
}


void TIM3_OCConfig(void) {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); //if disabled, 
	//the TIMx_CCRx register can be updated at any time by software to control the output
	//waveform---from the reference manual, I BELIEVE THIS IS DONE SINCE TIM3 RESET TO 0 AFTER reaching pulse or CCR1 value.
}	
	



void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//since TIMER 2 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//====================================================
	//Enable TIM2 global interrupt ====does this part need to be done before TIM_BaseStructure set up?
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//================================================
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR1_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValueTIM2;    //why all the example make this one equal 0, and then use 
					//function TIM_PrescalerConfig() to re-assign the prescaller value?
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

}


void TIM2_OCConfig(void)
{
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable); //if disabled, 
	//the TIMx_CCRx register can be updated at any time by software to control the output
	//waveform---from the reference manual, I BELIEVE THIS IS DONE SINCE TIM3 RESET TO 0 AFTER reaching pulse or CCR1 value.
	
	
}





void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				LCD_DisplayChar(LINE(LineNumber), COLUMN(ColumnNumber), *ptr);
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if (ColumnNumber*(((sFONT *)LCD_GetFont())->Width)>=LCD_PIXEL_WIDTH ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void RNG_Config(void){
	//Enable RNG controller clock
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	
	//activate the RNG peripheral 
	RNG_Cmd(ENABLE);
	
	// to get a random number, need to continue steps: 3. Wait until the 32 bit Random number Generator 
	//contains a valid random data (using polling/interrupt mode). For more details, 
	//refer to Section 20.2.4: "Interrupt and flag management" module description.
	//4. Get the 32 bit Random number using RNG_GetRandomNumber() function
	//5. To get another 32 bit Random number, go to step 3.
}	


//Configuring external button connected to pin PC1 
void external_button_config(void){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
		 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	//set up as internal pull up input 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);  

	//connects the interrupt, so that button will be activated by interrupt instead of polling
	EXTI_InitStructure.EXTI_Line=EXTI_Line1; 
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; 	
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
