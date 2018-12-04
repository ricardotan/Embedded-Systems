#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


//function prototype
void GPIO_Config(void);
void TIM3_Config(void);
void TIM3_OCConfig(void);
void PB_Config(void); //set up user push button
void LED_Config(void);
void external_button1_config(void); //sets up external pushbutton1 PC1
void external_button2_config(void);//sets up external pushbutton2 PD2
void external_button3_config(void);//sets up external pushbutton2 PC3
void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);
void fullStep_rotate(void); //full step movements of the motor
void halfStep_rotate(void);// half step movement of the motor



unsigned char fullStepSequence[]= {0x02,0x04,0x01,0x08};
/*Assumption of the PINS: PE2=A , PE3=B, PE4=C , PE5=D
my configuration assumption: imagine as cross starting from the left side going clockwise is labelled A C B D
A-B are one coil(opposite ends of each other), CD are one coil
A=0x08 , B=0x04 , C=0x02 , D=0x01
sequence to go clockwise is therefore C-B-D-A 
*/


unsigned char halfStepSequence[] = {0x02,0x06,0x04,0x05,0x01,0x09,0x08,0x0A};
//same configuration as the full step mentioned earlier
//for clockwise it would be C, CB, B, BD, D, DA, A, AC

uint8_t pinValues; //set to one of the element in the array of the sequences(either full or half)
uint16_t PrescalerValueTIM3 = 0; //TIM3 interrupt used for timing ms
__IO uint16_t CCR1_Val = 500; //choose this number so to avoid loss of accuracy of prescale due to integer division
double rev_time; //time it takes to do one revolution (last 2 dig of student # +33 = 09 + 33 =4)
double fullStep_time; //time it takes to do 1 step in full sequence. unit is sec/step
double halfStep_time; //time it takes to do 1 step in half sequence. unit is sec/step
int tick =0; //counter for the timer, increases by 1 every 0.1ms set by prescaler
int ticks_fullStep; //number of ticks needed before a step can be done when using fullstep sequence
int ticks_halfStep; //number of ticks needed before a step can done when using half step sequence
uint8_t mode;//either FULLSTEP or HALFSTEP , the defines are in main.h
uint8_t direction; //either CW OR CCW, the defines are in main.h 
int step = 0; //used as the index of the fullStepSequence and the halfStepSequenceArray
uint8_t move=0;//ready to step or not. 0 not ready, 1 is ready

uint8_t UBPressed; //user push button to change direction
uint8_t extUBPressed1; //external button connected to PC1 to change the mode
uint8_t extUBPressed2; //external button conncted to PD2 to increase the revolution time
uint8_t extUBPressed3; //external button connected PC3 to to decrease the revolution time

int main(void){

	GPIO_Config();
	
	PB_Config();
	external_button1_config();
	external_button2_config();
	external_button3_config();
	LED_Config();
	
	/* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	LCD_Clear(LCD_COLOR_WHITE);
	
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
  PrescalerValueTIM3 = (uint16_t) ((SystemCoreClock / 2) / (5000000)) - 1; //need interrupt every 0.1ms(10000Hz), 10000 ticks happens in 1 second
	
	TIM3_Config();
	TIM3_OCConfig();
	
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); //enable the TIM3 interrupt
	TIM_Cmd(TIM3,ENABLE); //enable TIM3
	
	//default settings
	rev_time =42; //time it takes to do one revolution (last 2 dig of student # +33 = 09 + 33 =42)
	fullStep_time = rev_time / 48 ; //divide by 48 because the motor need 48 steps to do 1 rev in full sequence. Unit is sec/step
	halfStep_time = rev_time / 96 ; //divide by 96 because the motor needs 96 steos to do 1 rev in half sequence. Unit is sec/step
	
	ticks_fullStep = fullStep_time / 0.0001; //divided by 0.1ms because that is time for 1 interrupt to happen
	ticks_halfStep = halfStep_time / 0.0001;
	
	mode=FULLSTEP;
	direction= CW;
	

	while(1){
		

		//changing the direction
		if (UBPressed == 1){
			UBPressed =0;
			STM_EVAL_LEDOn(LED4);
			if (direction == CW) direction=CCW;
			else direction = CW;
		}
		
		//changing the speed
		if ((extUBPressed2 == 1) ||(extUBPressed3 ==1 )){
			fullStep_time = rev_time / 48 ; 
			halfStep_time = rev_time / 96 ; 
			
			ticks_fullStep = fullStep_time / 0.0001; 
			ticks_halfStep = halfStep_time / 0.0001;
			
			STM_EVAL_LEDToggle(LED4);
			extUBPressed2=0;
			extUBPressed3=0;
		}
		
		
		//Changing the mode
		if (extUBPressed1==1){
			extUBPressed1=0;
			STM_EVAL_LEDToggle(LED3);
			if (mode == FULLSTEP) mode = HALFSTEP;
			else mode =FULLSTEP;
		}
		
		//Making the motor move 
		if (mode == FULLSTEP){
			if (move ==1){
				fullStep_rotate();
				move=0;//reset the move state
			}
		}
		else{ //HALF-STEP
			if (move==1){
				halfStep_rotate();
				move=0;
			}
		}
		
		
	}//END OF WHILE
}//END OF MAIN

//sets up user push button
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
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; //pull up therefore active-low (connect one end of button to PC1 and the other end directly to ground)
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
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; //pull up therefore active-low (connect one end of button to PD2 and the other end directly to ground)
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

//Configuring external button connected to pin PC3 (to see which pins are available check the stm32 usermanual)
void external_button3_config(void){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
		 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	//set up as internal pull up input 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; //pull up therefore active-low (connect one end of button to PC3 and the other end directly to ground)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);  

	//connects the interrupt, so that button will be activated by interrupt instead of polling
	EXTI_InitStructure.EXTI_Line=EXTI_Line3; 
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; 	
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//Configure the GPIO for output to the motor.
void GPIO_Config(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
		//Enable the GPIO AHB clock 
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  //using GPIOE
	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //using push-pull recommended by TA in tutorial
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; 
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ; //PE2,PE3,PE4,PE5
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOE, &GPIO_InitStructure);  
}

void setPinValues(uint8_t pinValues){
	//Assumption of the PINS: PE2=A , PE3=B, PE4=C , PE5=D
	//A=0x08 , B=0x04 , C=0x02 , D=0x01
	//notice that if not the pins you want to set it will just be all zeros since ANDED
	//for 0x02, 0x04, 0x08 the bits are shifted to make it equal to 1
	//for example 0x02 AND with 0x02 is 0010, need to make this into a 1 so that BITACTION will take it as set it, thus shifted by 1, same logic follows for 0x04 and 0x08
	
	//BitAction is  Bit_RESET = 0, and Bit_SET=1
	//in C any value not zero is considered true
	
	GPIO_WriteBit(GPIOE, GPIO_Pin_5, (BitAction)(pinValues & 0x01) );
	GPIO_WriteBit(GPIOE, GPIO_Pin_4, (BitAction)((pinValues & 0x02)));
	GPIO_WriteBit(GPIOE, GPIO_Pin_3, (BitAction)((pinValues & 0x04)));
	GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)((pinValues & 0x08)));
	
}


/**
 * Configure the TIM3 in output compare mode.
 */
void Timer_Config(void){
	
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
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValueTIM3;  
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

}

//output compare
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

//move a full step
void fullStep_rotate(void){
	if (direction == CW){
		step++;
		if (step > 3) step =0; //this also protect against when changing mode from halfStep to full since lets say you in half step with step=6, then change to fullStep
	}
	else //the CCW case
	{
		step--;
		if (step<0) step = 3;
	}
	
	pinValues = fullStepSequence[step]; //the pin(s) that have to be turned on or off
	setPinValues(pinValues); //sets the actual pin(s) to either high or low
}

//moves a halfstep
void halfStep_rotate(void){
	if (direction == CW){
		step++;
		if (step > 7) step =0;
	}
	else //the CCW case
	{
		step--;
		if (step<0) step =7;
	}
	
	pinValues = halfStepSequence[step];//the pin(s that have to be turned on or off
	setPinValues(pinValues); //sets thet actual pin(s) to either high or low
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


