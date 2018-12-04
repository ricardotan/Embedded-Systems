//the general way of how to set it up
//connect 5v and ground on the board
//the user button used PC1 and PD2
//ADC in is PC3
//ADC out (pwm signal) is PA7

#include "main.h"

//the following two addresses are useful when using the ADC and DAC in DMA mode

#define ADC_CDR_ADDR 				((uint32_t)(ADC_BASE + 0x08)) //ADC_CDR address as described in section 10.13.17 of reference manual
//ADC CDR is the common regular (not injected) data register for dual and triple modes
//also defined in stm32f4xx_adc.c as 
//#define CDR_ADDRESS 			((uint32_t)0x40012308) 
//which results in the same address
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C) //used for DMA configuration periph base address

#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))    //see font.h, for defining LINE(X)

//Variable declaration
__IO uint16_t ADC3ConvertedValue=0; //used in DMA configuration for DMA base memory address
double tempMeasured; //the measured temperature
double setPoint=30; //at temperature above the set point the fans will turn on
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint8_t UBPressed = 0; //user button press
__IO uint8_t extUBPressed1 = 0; //user button press PC1
__IO uint8_t extUBPressed2 = 0; //user button press PD2
uint16_t PrescalerValueTIM2; //sets the timer 2 counter clock 
__IO uint16_t CCR1_Val = 500; //choose this number so to avoid loss of accuracy of prescale due to integer division

//for the PWM
uint16_t PrescalerValueTIM3;
int TIM3_ARR = 999; //counts from 0 to 999, will be set to TIM_period of TIM3
int TIM3_CCR=0; //value can go from 0 to 1000(ARR+1). Where at 1000 it is 100% duty cycle

extern uint8_t waitComplete; //is waiting time completed

enum mode currentMode = RUNNING; //full enumeration in main.h

//Function Prototypes
void PB_Config(void); //push button configuration
void external_button1_config(void);//PC1
void external_button2_config(void);//PD2
void LED_Config(void);

//used for 0.5s interrupts for display and timing holding down button
void TIM2_Config(void);
void TIM2_OCConfig(void);

//using TIM3 channel 1 as pwm output for fan
//all of this function is called in PWM_Config()
void TIM3_Config(void);
void TIM3_OCConfig(void);
void TIM3_PinConfig(void);

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);
void ADC_Config(void);  //Use this function to configure the ADC input.  Using ADC3 configured to channel 13
void DMA_Config(void);//Configuration of DMA(direct memory access)
void PWM_Config(void);//sets up PWM
void getTemp(void); //obtains the temperature measurement and does calculation to convert it correctly
void showTemp(void);//displays the temperature on the LCD screen



int main(void){
	
	//PC1 Button config
	 external_button1_config();
	
	//PD2 Button Config
	 external_button2_config();
	
	//Push Button Config
	PB_Config();
	//LEDs config
	LED_Config();
	 /* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	
	//configure push button
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	//timer configuration
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
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 10.0 Hz --> this is the actually HZ when interrupt called
    ==> Toggling frequency = 5 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
		 ----------------------------------------------------------------------- */ 	
	//Setting up the prescaler value
	PrescalerValueTIM2 =  (uint16_t) ((SystemCoreClock / 2) / 1000) - 1;  //interrupt every 0.5 s(2 HZ)
	
	PrescalerValueTIM3 = (uint16_t) ((SystemCoreClock / 2) / 50000) - 1; //each clock tick frequency of TIM3 is 50kHz(50 000 ticks per second)
	/*
	since TIM3_ARR = 999, this will mean that a whole PWM cycle will be  20 ms. Sample calculation:
	period of PWM = (ARR + 1) / Clock counter frequency  = (999+1) / (50 000) = 0.02 sec = 20 ms
	This is found on pg 371 of the textbook, also found my written lab notes also on chapter 15 slide 12 
	
	
	if PWM mode 1	
	duty cycle = TIM3_CCR / (TIM3_ARR + 1) * 100%
	*/
	
	//Configuring the TIM2
	TIM2_Config();
	TIM2_OCConfig();
	
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE); //enable the TIM2 interrupt
	TIM_Cmd(TIM2,ENABLE); //enable TIM2

	
	//configure ADC device
	ADC_Config();
	
	//configure the DMA mode
	DMA_Config();
	//configure PWM output
	PWM_Config();
	
	//start ADC3 software conversion
	 ADC_SoftwareStartConv(ADC3);
	
	LCD_Clear(LCD_COLOR_WHITE);
	
	//initial display
	LCD_DisplayStringLine(LINE(0),  (uint8_t *) "PWM Fan control");
	LCD_DisplayString(2,0, (uint8_t *) "Current Temp:");
	LCD_DisplayString(5,0, (uint8_t *) "Set Point Temp:");
	LCD_DisplayFloat(6, 1, setPoint, 2);

	
	
	while(1){
		switch(currentMode){
			//the mode where if the current temp is greater than the temperature set point, the fan will turn on to bring it down below it
			case RUNNING:
				//If the increase(PC1) or decrease(PD2) set point button is pressed
				if (extUBPressed1 ==1 || extUBPressed2 ==1 ) {
						extUBPressed1=0;
						extUBPressed2=0;
						currentMode=CHANGE_SP;
				}
				
				//checking whether or not to turn on the fan
				if (tempMeasured  > setPoint){ //everytime showTemp() is called in interrupt, it automatically calls getTemp() as well which updates the temp reading
					//turn on fan
					//STM_EVAL_LEDOn(LED3);
					/*Explanation of the calculation
					initially tempMeasured - setPoint =0 therefore start of at 50% duty cycle which is CCR=500
					then, I assumed that if the different between the temperature reaches 5 degrees then 
					it should turn on the fan full power therefore to get to 100% duty cycle I need 500, 
					since it is 5 degrees therefore 500/5 which 100, that why you have that multiplication
					*/
					TIM3_CCR = 500 + (tempMeasured - setPoint) * 100;
					if (TIM3_CCR > 1000) TIM3_CCR =1000; //making sure that it does not go above 
					TIM_SetCompare2(TIM3,TIM3_CCR); //must use this function , can't do .TIM_Pulse=TIM3_CCR;

				}
				
				else{
					//turn off the fan
					TIM3_CCR=0;
					TIM_SetCompare2(TIM3,TIM3_CCR);
					//STM_EVAL_LEDOff(LED3);
					
				}
				break;
				
			//the mode to change the set point temperature
			case CHANGE_SP:
					//hold PC1 to increase the set point by 1 degrees every 0.5 seconds
					while (GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_1)==0){ //set to zero because button is active low, when held down it is connected to ground
						if (waitComplete ==1){
							waitComplete=0;
							setPoint += 1;
							LCD_ClearLine(LINE(6));
							LCD_DisplayFloat(6, 1, setPoint, 2);
						}

					}
					//hold PD2 to decrease the set point by 1 degrees every 0.5 seconds
					while (GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_2)==0){ //set to zero because button is active low, when held down it is connected to ground
						if (waitComplete ==1){
							waitComplete=0;
							setPoint-=1;
							LCD_ClearLine(LINE(6));
							LCD_DisplayFloat(6, 1, setPoint, 2);
						}	
					}
						//reset the button, because when let go it will equal to 1
						extUBPressed1=0;
						extUBPressed2=0;
						currentMode =RUNNING;
						break;
				
			default:
				break;
		}	
	}//end of while
}//end of main

//set up user pushbutton
void PB_Config(void){
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

//set up LED
void LED_Config(void)
{
 /* Initialize Leds mounted on STM32F429-Discovery board */
  STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4); 
}

//Configuring external button connected to pin PC1 (to see which pins are available check the stm32 usermanual)
void external_button1_config(void){
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

//Configuring external button connected to pin PD2 (to see which pins are available check the stm32 usermanual)
void external_button2_config(void){  

	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	//set up as internal pull up input
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;      				
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);  

	//connects the interrupt, so that button will be activated by interrupt instead of polling
	EXTI_InitStructure.EXTI_Line=EXTI_Line2; 
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling; 		
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//setting up timer 2 used for interrupts
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//since TIMER 2 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//====================================================
	//Enable TIM2 global interrupt 
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//================================================
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR1_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValueTIM2;    
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

}

//setting up timer 2 output compare 
void TIM2_OCConfig(void)
{
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable); //if disabled, CCR register can be changed

	
}

//Setting up the TIM3 clock used for PWM signal
void TIM3_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period=TIM3_ARR; //the autoreload register in this case 999, so after that value it reload back to 0
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValueTIM3;    
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM3, ENABLE); 
	
	// Enable TIM3 counter 
	TIM_Cmd(TIM3, ENABLE);
}

//setting up PWM as mode 1 check out chapter 15 slide 12 for more information
//summary of PWM mode 1: high if counter < CCR , low if counter >=CCR
void TIM3_OCConfig(void){
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;  //PWM mode 1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=TIM3_CCR; //changing this value while TIM3_ARR remains constant will change duty cycle
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	//TIM 3 channel 2 (need this spefically since using PA7 in documentation only allows this)
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable); //disable this so that can change CCR value, thus changing the duty cycle
}

//setting up the ADC output pins for the PWM signal
//using button PA7 according to STM32F429DatasheetAF.pdf, PA7 has AF with tim3 channel 2
void TIM3_PinConfig(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
		
	//since TIMER 3 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//enable GPIOA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//sets up AF function for PA7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //enable alternate function for the PIN in this case PWM signal
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //enables the alternate function for PA7

}


//Displays string
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

//Displays an integer
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

//Displays a float
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}


/**
 * Use this function to configure the ADC input.  Using ADC3 configured to channel 13
Template from STM32F429I-Discovery_FW_V1.0.1\Projects\Peripheral_Examples\ADC_DMA\main.c
 */
void ADC_Config(void){
			//initial structures
			ADC_InitTypeDef	      ADC_InitStructure;
			ADC_CommonInitTypeDef ADC_CommonInitStructure;
			GPIO_InitTypeDef      GPIO_InitStructure;
	
			
			//Enabling the ADC clock in this case it is ADC3
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
			//Enabling clock for the GPIO
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
			//Configuring ADC3 Channel 13 pin as analog input (using PC3)
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  //setting as analog mode
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		
			//ADC common configuration
			ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; //single ADC mode
					//ADCCLK=PCLK2/2  
					//SYSclock and AHB clock is 180Mhz, APB2 prescaler is 2, so the APB2 CLK is 90Mhz.
			ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
					//then at the anlog part, the clock freq is 45Mhz.
			ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  //Disabled or 1, 2, or 3.
					//why disable here? ---Configures the Direct memory access mode for multi ADC mode
			ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  //can be 5--20
					//----is this the sampling cycles? or is the sampling cycle is default to 3????
					//----NO, it is not sampling cycle number, the line below: ADC_regualrChannelConfig() defines
					// the sampling time as 3 Cycles!!!
					//Actually, These  are used in dual or triple interleaved modes. In this lab only one ADC auded, so this setting mean nothing
			ADC_CommonInit(&ADC_CommonInitStructure);
			
			
			//ADC3 init
			ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;  //12 bit resolution, takes an additional 12 bit clock cycle after sampling time
			
			//Single channel,continous mode on textbook pg.450 also on slide I believe
			ADC_InitStructure.ADC_ScanConvMode = DISABLE;
			ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
			
			
			ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //Falling, Rising or RisingFalling
			ADC_InitStructure.ADC_ExternalTrigConv =0; 
			ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
			ADC_InitStructure.ADC_NbrOfConversion = 1;   //number of ADC conversions that will be done using the sequencer for regular channel group.
			ADC_Init(ADC3, &ADC_InitStructure);
			
			//ADC3 regular channel 13 configuration
			ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_480Cycles); 
							
		 /* Enable DMA request after last transfer (Single-ADC mode) */ 
			ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
			
			ADC_DMACmd(ADC3, ENABLE); //Enable ACD3 DMA
					
			ADC_Cmd(ADC3, ENABLE); //Enables ADC3
}

//Configuration of DMA(direct memory access) template obtained from STM32F429I-Discovery_FW_V1.0.1\Projects\Peripheral_Examples\ADC_DMA\main.c
//using DMA2
void DMA_Config(void){
			//initial structure
			DMA_InitTypeDef DMA_InitStructure;
			
			//Enabling the DMA clock, in this case using DMA2
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		
			//DMA2 channel 2 configuration.The requests from ADC3 are all mapped to channels of streams of DMA2
		  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_BufferSize = 1; //this is 1 data unit -> "The data unit is equal to the configuration set in DMA_PeripheralDataSize"
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //dont increase the peripheral address register
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; //don't increase the memory address register
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //Byte, HalfWord(16bit), or Word
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//amount of data sent in one burst
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream0, &DMA_InitStructure);   //using stream 0 in this case
			DMA_Cmd(DMA2_Stream0, ENABLE);
}

/**
 * Use this function to configure the PWM output.
 */
void PWM_Config(void){
	TIM3_Config();
	TIM3_OCConfig();
	TIM3_PinConfig();
}

//obtain the temperature that is converted
void getTemp(void){
	/*explanation of calculation
	according to the reference manual Vref+ is 3V, here I am using 12 bit ADC
	values range from 0 to 4095 (2^12=4096 numbers), therefore the ADC resolution is Vref/4096 = 3/4096.
	However, the signal is received a gain 0f 3x, therefore (3/4096)/3.
	Furthermore, the temperature sensor outputs 10mV/degree = 0.01V/degree.
	Therefore the converted value factor is ((3/4096)/3)/0.01 = 0.02441
	*/
	tempMeasured = 0.02441*ADC3ConvertedValue;
}

//shows the temperature reading on the LCD screen
void showTemp(void){
	getTemp();
	LCD_DisplayFloat(3, 1, tempMeasured, 2);
}


