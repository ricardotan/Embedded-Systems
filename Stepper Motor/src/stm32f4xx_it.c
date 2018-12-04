/**
  ******************************************************************************
  * @file    TIM_TimeBase/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f4xx_it.h"
#include "stm32f429i_discovery.h"
#include "main.h"
__IO uint32_t RTCAlarmCount = 0;
extern __IO uint8_t AlarmUp;
//extern void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
//extern void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
extern uint8_t mode;
extern uint8_t move;//ready to step or not. 0 not ready, 1 is ready
extern int tick; //counter for the timer, increases by 1 every 0.1ms set by prescaler
extern int ticks_fullStep; //number of ticks needed before a step can be done when using fullstep sequence
extern int ticks_halfStep; //number of ticks needed before a step can done when using half step sequence
extern double rev_time;

extern uint8_t UBPressed;
extern uint8_t extUBPressed1;
extern uint8_t extUBPressed2; //external button conncted to PD2 to increase the revolution time
extern uint8_t extUBPressed3; //external button connected PC3 to to decrease the revolution time

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void){
	//uncomment me if you want to handle systicks
	//TimingDelay_Decrement();
}

/******************************************************************************/
/*            STM32F4xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		
		tick++;
		
		/*Reasoning behind why tick >= ticks_halfStep instead of just ==, Short answer is that this is so that it fault tolerating when changing modes
		ticks_fullStep > ticks_halfStep, so this means that when changing between the modes often time you can get stuck on one particular step because 
		the move flag will never be set. For example, let say you are initially in FULLSTEP mode with tick_fullStep =8000 and tick_halfStep =3000.
		Imagine you are in FULLSTEP with your tick=5000, and at this point you press the button to change to HALFSTEP mode, notice that your move flag will never
		be set for halfstep mode since your tick is already greater than tick_halfStep and you wont move.
		
		Usually moving from halfstep to fullstep you can't get stuck since even if you are halfway between a step counter in halfstep, your tick
		value will always be smaller than tick_fullStep therefore your move flag will get set. So essentially I don't need tick>= tick_fullStep, but I do need it
		for tick >= tick_halfStep.Reason I kept it is to keep it consistent on both.
		*/
		if ((mode == FULLSTEP ) && (tick >= ticks_fullStep)){
			move=1;//ready to move to the next step
			tick =0 ;//reset the counter
		}
		else if ((mode == HALFSTEP) && (tick >= ticks_halfStep)){
			move =1; //ready to move to the next step
			tick=0; //reset the counter
		}
		TIM_SetCounter(TIM3, 0x0000); 
  }
}

/**
 * External interrupt channel 0 Interrupt Handler. This handles
 * the user button.
 */
void EXTI0_IRQHandler(void){
	STM_EVAL_LEDToggle(LED3);
	UBPressed=1;
	//clear the pending bit otherwise the handler will fire continually
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	//this is configured to handle the push-button
}

//this handles PC1 external button
void EXTI1_IRQHandler(void){
	extUBPressed1=1;
	EXTI_ClearITPendingBit(EXTI_Line1);
}

//this handles PD2 external button
void EXTI2_IRQHandler(void){
	
	
	rev_time +=10;
	extUBPressed2=1;
	//STM_EVAL_LEDToggle(LED4);
	EXTI_ClearITPendingBit(EXTI_Line2);
}

//this handles PC3 external button
void EXTI3_IRQHandler(void){
	
	rev_time-=10;
	extUBPressed3=1;
	EXTI_ClearITPendingBit(EXTI_Line3);
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
