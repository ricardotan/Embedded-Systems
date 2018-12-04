//Ricardo Tan
//RTC clock shown at all time. If user push button is pressed it will save the timestamp to the EEPROM.
//It will save all timestamps(Log it) however only show the past 2 timestamp
//You can change the date by pressing external button 2, you cycle changing the field by pressing this button.
//You increase the value of the specified field by pressing external button 1.
//Also if the onboard user button is pushed and held for 2 seconds it will show the date.

#include "main.h" //sometimes this might say that it can't find a specific file all you need to do it go to the Options for Target and
									//configure it the same way you always do

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

uint8_t Tx1_Buffer = 0;
uint8_t Rx1_Buffer;
uint16_t NumDataRead = 1;


enum modes currentMode=IDLE; //full enumeration in main.h so that it can be used in stm32f4xx_it.c


__IO uint8_t UBPressed = 0; //user button press
__IO uint8_t extUBPressed1 = 0; //user button press pc1
__IO uint8_t extUBPressed2 = 0; //user button press pc3


extern uint8_t sTicks;//used for the waitTime
extern uint8_t waitComplete; //is waiting time completed

uint16_t line;
uint8_t wd, dd, mo, yy, ss, mm, hh; // represents weekday, day, month, year, second, minute, hour

/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


//function prototypes
void showDate(void); //shows the current date
void showTime(void); //shows the current time
void RTC_AlarmAConfig(void); //alarm set to 1HZ which helps updates the screen to show the time
void saveTimeStamp(void); //save the time when the on board push button is pressed
void viewTimeStamp(void); //shows the last 2 timestamps that are stores in the EEPROM
void initValueTimeStamp(void); //first two values of EEPROM initialized so that format is nice when there is no previous timestamps
void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr); //prints a string on the LCD screen
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number); //prints an integer on the LCD screen
void external_button1_config(void); //button PC1 on the stm32 board
void external_button2_config(void);//button PC3 on the stm32 board
void setTimeAndDate(void); //change the time and date
void restartRTC(void); //need to restart the RTC after turning it off to set date and time


int main(void){
	
	//configure push-button interrupts
	PB_Config();
	
	 /* LCD initiatization */
  LCD_Init();
  
  /* LCD Layer initiatization */
  LCD_LayerInit();
    
  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
	
	//Initiliazing LED
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4); 

	
	//======You need to develop the following functions======
	//Note: these are just placeholders; function definitions are at bottom of this file
	//configure real-time clock
	RTC_Config();
	
	//Configures Alarm A used for interrupt to update screen when viewing the clock
	RTC_AlarmAConfig();
	
	
	LCD_Clear(LCD_COLOR_WHITE);
		
		//FOR NEXT TIME: to check if your circuit has been set up properly, open the LAB3 starter, then have 
		// a code written if success or failure
		
	//Configuring the external push button with interrupts
	external_button1_config();
	external_button2_config();
	
	//Initialize the EEPROM
	sEE_Init();  
	
	//Initializes the first two values on the EEPROM so it looks properly formatted
	initValueTimeStamp(); 
	
	//main loop
	while(1){
		switch(currentMode){
			//The mode where it shows the the current time
			case IDLE:
					//if the on board user button is pressed then save the time when that happened
					if (UBPressed ==1){
						UBPressed=0;
						currentMode = SAVE_TS;
					}
					//Press external button 1 to see the past 2 timestamp
					else if (extUBPressed1 ==1){
						extUBPressed1=0;
						currentMode=VIEW_TS;
					}
					//Press external button2 to start changing the time/date
					else if (extUBPressed2 ==1){
						extUBPressed2=0;
						currentMode = CHANGE_TIME_DATE;
					}
				break;
			//The mode where the time when the user push button was pressed		
			case SAVE_TS:
					saveTimeStamp();
					currentMode = SHOW_DATE;
					sTicks=0;
					break;
			
			//The mode in which is the user push button is push and held it will show the date
			case SHOW_DATE:
				while (STM_EVAL_PBGetState(BUTTON_USER)==1) {
					if (waitComplete ==1)
						showDate();
				}
				LCD_ClearLine(LINE(4));
				LCD_ClearLine(LINE(5));

				waitComplete=0;
				currentMode =IDLE;
				break;
			
			//The mode 	where the past two timestamp is shown
			case VIEW_TS:
				viewTimeStamp();
				while(1){
					if (extUBPressed1 ==1){
						extUBPressed1=0;
						LCD_ClearLine(LINE(8));
						LCD_ClearLine(LINE(9));
						LCD_ClearLine(LINE(10));
						//Just in case someone pressed these two button during this mode
						UBPressed=0; 
						extUBPressed2=0;
						
						currentMode = IDLE;
						break;
					}
				}
				break;
			//The mode where the time/date is changed
			case CHANGE_TIME_DATE:
				RCC_RTCCLKCmd(DISABLE); //turn off the RTC while changing the time/date
				showTime();
				showDate();
		
	
				LCD_SetTextColor(0xF800); //RED
				
				//changing the seconds
				LCD_DisplayInt(1,6,ss);
				while (extUBPressed2 ==0){
					if (extUBPressed1 == 1){
						extUBPressed1=0;
						ss= (ss+1)%59; //seconds are 0-59
						LCD_DisplayInt(1,6,ss);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(1,6,ss);
				extUBPressed2=0; //reset the button
				
				//changing the minutes
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(1,3,mm);
				while (extUBPressed2 ==0){
					if(extUBPressed1 == 1){
						extUBPressed1=0;
						mm = (mm+1)%59; //minutes goes from 0-59
						LCD_DisplayInt(1,3,mm);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(1,3,mm);
				extUBPressed2=0;
				
				
				//changing the hours
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(1,0,hh);
				while(extUBPressed2 ==0){
					if(extUBPressed1 == 1){
						extUBPressed1=0;
						hh = (hh+1)%24; //hours from 0-23
						LCD_DisplayInt(1,0,hh);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(1,0,hh);
				extUBPressed2=0;
				
				//changing the year
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(5,9, yy);
				while (extUBPressed2 ==0){
					if (extUBPressed1 ==1){
						extUBPressed1=0;
						yy= (yy+1)%100; //year is from 00-99
						LCD_DisplayInt(5,9,yy);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(5,9,yy);
				extUBPressed2=0;
				
				//changing the month
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(5,6, mo);
				while (extUBPressed2 ==0){
					if (extUBPressed1 ==1){
						extUBPressed1=0;
						mo= mo%12+1; //month is 1-12
						LCD_DisplayInt(5,6,mo);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(5,6,mo);
				extUBPressed2=0;
				
				//Changing the day
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(5,3, dd);
				while (extUBPressed2 ==0){
					if (extUBPressed1 ==1){
						extUBPressed1=0;
						dd= dd%31+1; //day is from 1-31, notice that can put invalid date
						LCD_DisplayInt(5,3,dd);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(5,3,dd);
				extUBPressed2=0;
				
				
				//Changing the weekday
				LCD_SetTextColor(0xF800); //RED
				LCD_DisplayInt(5,0, wd);
				while (extUBPressed2 ==0){
					if (extUBPressed1 ==1){
						extUBPressed1=0;
						wd= wd%7+1; //weekday is 1-7
						LCD_DisplayInt(5,0,wd);
					}
				}
				LCD_SetTextColor(0x0000); //Black
				LCD_DisplayInt(5,0,wd);
				extUBPressed2=0;
				
				
				LCD_ClearLine(LINE(4));
				LCD_ClearLine(LINE(5));
		
				//need to restart RTC after turning it off		
				restartRTC();
				setTimeAndDate();
				currentMode = IDLE;
				break;
				
			default:
				break;
		}
	}//end while
}//end of main



void PB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
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
		sprintf(lcd_buffer,"%02d",Number); //has 02 in the front because want to display 2 numbers, if one number then stuff with 0
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}




/**
 * Configure the RTC to operate based on the LSI (Internal Low Speed oscillator)
 * and configure one of the alarms (A or B) to trigger an external interrupt every second
 * (e.g. EXTI line 17 for alarm A).
 */
void RTC_Config(void){
	
	//Structure Definitions to set up the RTC,date, and time
	RTC_InitTypeDef RTC_InitStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
	//This essentially enables write access to RTC, because write protected by default
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	
	//Enable access to RTC domain
	PWR_BackupAccessCmd(ENABLE);
	
	//enabling the LSI
	RCC_LSICmd(ENABLE);
	//Selecting LSI for RTC 
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	
	//enable the RTC clock
	RCC_RTCCLKCmd(ENABLE);

	/*
		RTC frequency = (clock frequency) / ((AsynchPrediv+1) * (SynchPrediv+1))
	*/
	//Setting so that RTC is running at 1HZ
	RTC_InitStructure.RTC_AsynchPrediv = 127; //using LSI therefore these are the values, shown on Application Note pg 9
	RTC_InitStructure.RTC_SynchPrediv = 249;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
	
	//Setting the time
	//This is done in BCD format, remember each character in hex represent a byte in binary
	RTC_TimeStructure.RTC_Hours=0x23;
	RTC_TimeStructure.RTC_Minutes=0x59;
	RTC_TimeStructure.RTC_Seconds=0x40;
	
	RTC_SetTime(RTC_Format_BCD,&RTC_TimeStructure);
	
	//Setting the date, this is done in BCD
	RTC_DateStructure.RTC_WeekDay=RTC_Weekday_Sunday;
	RTC_DateStructure.RTC_Month=RTC_Month_October;
	RTC_DateStructure.RTC_Date=0x28;
	RTC_DateStructure.RTC_Year=0x17;
	
	RTC_SetDate(RTC_Format_BCD,&RTC_DateStructure);
}


//Setting up the RTC alarm, will be used to show the time, using Alarm A in this case
void RTC_AlarmAConfig(void){
	EXTI_InitTypeDef EXTI_InitStructure;
  RTC_AlarmTypeDef RTC_AlarmStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
 	
	//Configuring the EXTI Line 17
	EXTI_InitStructure.EXTI_Line=EXTI_Line17;    
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;   //required to be Rising edge.
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	//Configurig the IRQ Handler
	NVIC_InitStructure.NVIC_IRQChannel=RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Configuring the Alarm (when it should go off)
	/*Also note that Alarm_mask means which component of time will be ignored to compare if all 0000 therefore don't ignore anything,
		the day and exact time(to the second) will be used for comparison  of when to turn on the alarm
		On the other hand if the mask is all 1111 therefore it means ignore everything, which also means turn on alarm every second.
		Since it is turned on every second, I dont need to set up the other elements in the RTC_alarmStructure since it will be ignored anyways 
	*/
	RTC_AlarmStructure.RTC_AlarmMask=RTC_AlarmMask_All; //if mask all therefore occurs every second, Application Note pg 11 & 12															
	RTC_SetAlarm(RTC_Format_BCD,RTC_Alarm_A, &RTC_AlarmStructure);
	
	//Enable interrupt from Alarm A
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);  //Enables or disables the specified RTC interrupts, RTC_IT_TS  for Time Stamp interrupt 
	
	//Enables the specified alarm itself
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);  //Enables or disables the specified RTC Alarm. 
	
	//Clears Alarm A's Flag
	RTC_ClearFlag(RTC_FLAG_ALRAF);    

}



//Showing the current date
void showDate(void){
	RTC_DateTypeDef RTC_DateStructure;
	
	//Eventhough date done in BCD it must be read in BIN format
	//Or else you will get the number incorrectly converted to decimal 
	RTC_GetDate(RTC_Format_BIN,&RTC_DateStructure);
	
	LCD_DisplayString(4,0, (uint8_t *) "WD:DD:MM:YY");
	
	wd=RTC_DateStructure.RTC_WeekDay;
	dd=RTC_DateStructure.RTC_Date;
	mo=RTC_DateStructure.RTC_Month;
	yy=RTC_DateStructure.RTC_Year;
	
	LCD_DisplayInt(5,0,wd);
	LCD_DisplayInt(5,3, dd);
	LCD_DisplayInt(5,6, mo);
	LCD_DisplayInt(5,9, yy);
	
}

//Showing the current time
void showTime(void)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	
	//Eventhough date done in BCD it must be read in BIN format
	//If set in BIN, can also read in BIN
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);

	ss=RTC_TimeStructure.RTC_Seconds;
	mm=RTC_TimeStructure.RTC_Minutes;
	hh=RTC_TimeStructure.RTC_Hours;
	
	LCD_DisplayString(0,0, (uint8_t *) "HH:MM:SS");
	
	LCD_ClearLine(LINE(1));
	LCD_DisplayInt(1,0,hh);
	LCD_DisplayInt(1,3,mm);
	LCD_DisplayInt(1,6,ss);
}

//Saves the time when the button was pressed in the EEPROM
void saveTimeStamp(void){
	uint8_t ssTS, mmTS, hhTS ; //temporary stores the timestamp
	
	RTC_TimeTypeDef RTC_TimeStructure;
	
	//Eventhough set in BCD must read in BIN
	//If set in BIN, can read in BIN
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeStructure);

	ssTS=RTC_TimeStructure.RTC_Seconds;
	mmTS=RTC_TimeStructure.RTC_Minutes;
	hhTS=RTC_TimeStructure.RTC_Hours;
	
	//This creates a log on the EEPROM of the timestamp
	sEE_WriteBuffer(&hhTS, memLocation++,1); 
	sEE_WriteBuffer(&mmTS, memLocation++,1); 
	sEE_WriteBuffer(&ssTS, memLocation++,1); 
}

//View the last 2 Timestamps
void viewTimeStamp(void){
	uint8_t ssTS1,mmTS1,hhTS1,ssTS2,mmTS2,hhTS2; //buffer for the timestamp
	
	memLocation -=6;//to get the last 2 time stamps, each time stamp is 3 memory location
	//Getting the first timestamp
	sEE_ReadBuffer(&hhTS1, memLocation++, (uint16_t *)(&NumDataRead)); 
	sEE_ReadBuffer(&mmTS1, memLocation++, (uint16_t *)(&NumDataRead)); 
	sEE_ReadBuffer(&ssTS1, memLocation++, (uint16_t *)(&NumDataRead)); 
	
	//Getting the second timestamp
	sEE_ReadBuffer(&hhTS2, memLocation++, (uint16_t *)(&NumDataRead)); 
	sEE_ReadBuffer(&mmTS2, memLocation++, (uint16_t *)(&NumDataRead)); 
	sEE_ReadBuffer(&ssTS2, memLocation++, (uint16_t *)(&NumDataRead)); 

	
	LCD_DisplayString(8,0, (uint8_t *) "Timestamps:");
	
	//Displaying first timestamp
	LCD_ClearLine(LINE(9));
	LCD_DisplayString(9,9, (uint8_t *) "LAST");
	LCD_DisplayString(9,0, (uint8_t *) "  :  :");
	LCD_DisplayInt(9,0,hhTS1);
	LCD_DisplayInt(9,3,mmTS1);
	LCD_DisplayInt(9,6,ssTS1);
	
	//Displaying second timestamp
	LCD_ClearLine(LINE(10));
	LCD_DisplayString(10,9, (uint8_t *) "RECENT");
	LCD_DisplayString(10,0, (uint8_t *) "  :  :");
	LCD_DisplayInt(10,0,hhTS2);
	LCD_DisplayInt(10,3,mmTS2);
	LCD_DisplayInt(10,6,ssTS2);
}
//Gives values in EEPROM initially(so that when view timestamps for first time it looks nice)
void initValueTimeStamp(void)
{
	int i;
	uint8_t Tx1_Buffer = 0;
	for (i=0;i<6;i++){
			sEE_WriteBuffer(&Tx1_Buffer, memLocation++,1); 
	}
	memLocation = memLocation -6;//bring backs memory location to initial location
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


//Configuring external button connected to pin PC3  (to see which pins are available check the stm32 usermanual)
void external_button2_config(void){
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
		 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	//set up as internal pull up input 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
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


void setTimeAndDate(void){
			RTC_TimeTypeDef RTC_TimeStructure;
			RTC_DateTypeDef RTC_DateStructure;

			RTC_TimeStructure.RTC_Hours=hh;
			RTC_TimeStructure.RTC_Minutes=mm;
			RTC_TimeStructure.RTC_Seconds=ss;
			
			RTC_SetTime(RTC_Format_BIN,&RTC_TimeStructure); //setting in binary unlike the RTC config which was in BCD
			
			
			RTC_DateStructure.RTC_WeekDay=wd;
			RTC_DateStructure.RTC_Month=mo;
			RTC_DateStructure.RTC_Date=dd;
			RTC_DateStructure.RTC_Year=yy;
		
			RTC_SetDate(RTC_Format_BIN,&RTC_DateStructure);//setting in binary unlike the RTC config which was in BCD
}

//Restart the RTC config
void restartRTC(void){
	//Structure Definitions to set up the RTC,date, and time
	RTC_InitTypeDef RTC_InitStructure;

	
	//This essentially enables write access to RTC, because write protected by default
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
	
	//Enable access to RTC domain
	PWR_BackupAccessCmd(ENABLE);
	
	//enabling the LSI
	RCC_LSICmd(ENABLE);
	//Selecting LSI for RTC 
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	
	//enable the RTC clock
	RCC_RTCCLKCmd(ENABLE);

	/*
		RTC frequecy = (clock frequency) / ((AsynchPrediv+1) * (SynchPrediv+1))
	*/
	//Setting so that RTC is running at 1HZ
	RTC_InitStructure.RTC_AsynchPrediv = 127; //using LSI therefore these are the values, shown on Application Note pg 9
	RTC_InitStructure.RTC_SynchPrediv = 249;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
	
}








