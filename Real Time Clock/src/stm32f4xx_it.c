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

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t UBPressed;//state of onboard push button , 1 if pressed 
extern __IO uint8_t extUBPressed1; //external button 1 
extern __IO uint8_t extUBPressed2; //external button 2
uint8_t sTicks=0; //seconds ticks
uint8_t waitComplete=0; //is waiting time completed
extern enum modes currentMode;


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
  
}

/**
 * External interrupt channel 0 Interrupt Handler. This handles
 * the user button.
 */
void EXTI0_IRQHandler(void){
	UBPressed=1;
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	
}

/**
 * External interrupt channel 1 for external button interrupts.
 * Think about using this put the program into 'set mode'.
 */
void EXTI1_IRQHandler(void){
	
	extUBPressed1=1;
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/**
 * External interrupt channel 3 for external button interrupts.
 * Think about using this to change the time segment when in 'set mode' and otherwise
 * to write to the EEPROM when in 'display mode'.
 */
void EXTI3_IRQHandler(void){
	extUBPressed2=1;
	EXTI_ClearITPendingBit(EXTI_Line3);
}

/*
 * This can be used to handle the RTC alarm interrupts. If you are
 * using alarm A or B you can configure the alarms to trigger every second.
 */
void RTC_Alarm_IRQHandler(void){
	
	if(RTC_GetITStatus(RTC_IT_ALRA)!=RESET) { 
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		showTime();
		
		//if wait for button to held for 2 full seconds
		if (currentMode == SHOW_DATE){
			sTicks++;
			if (sTicks == 2) waitComplete=1;
		}

	}
	EXTI_ClearITPendingBit(EXTI_Line17);
	
	//needed to clear the pending bits or else the interrupts will always occur
		
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
