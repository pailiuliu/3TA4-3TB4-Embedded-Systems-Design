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


extern int ClkRefresh; //LCD refresh counter
extern int state; // FSM state
extern int EEDone; // Should we save to EEPROM?

// these three hold the time when the UB is pressed, without waiting for any loops
extern uint8_t CHour;
extern uint8_t CMin;
extern uint8_t CSec;

extern int adder; // increments the dates/times


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
  
}

/**
 * External interrupt channel 0 Interrupt Handler. This handles
 * the user button.
 */
void EXTI0_IRQHandler(void){ //USER BUTTON
	//state 1 is when the button is pressed
	if (state==0){ //if it's in the base state

		state=1; // go into the state where the date is shown and it saves to EEprom
		
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure); //Saves the current time to put into EEPROM
		CHour= RTC_TimeStructure.RTC_Hours; //Putting struct into specific variables
		CMin= RTC_TimeStructure.RTC_Minutes;
		CSec= RTC_TimeStructure.RTC_Seconds;
		
		EEDone=1;	//Tells to save to EEprom
		
	}
	
	//clear the pending bit otherwise the handler will fire continually
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	//this is configured to handle the push-button
}

/**
 * External interrupt channel 1 for external button interrupts.
 * Think about using this put the program into 'set mode'.
 */
void EXTI1_IRQHandler(void){
	
	if (state == 0 || state == 1){ //In any of the first two states
		state = 2; // go to display the two last things		
	}
	if (state == 2){ // press it again and go back to the base function
		state = 0;
	}
	
	//clear pending bit
	EXTI_ClearITPendingBit(EXTI_Line1);
}

/**
 * External interrupt channel 3 for external button interrupts.
 * Think about using this to change the time segment when in 'set mode' and otherwise
 * to write to the EEPROM when in 'display mode'.
 */

void EXTI2_IRQHandler(void){ // second pushbutton
	//your code here - set time and date
 
		adder=0; // resets adder every time it's pressed
	// clears most of the screen
	LCD_ClearLine(LCD_LINE_3);
	LCD_ClearLine(LCD_LINE_4);
	LCD_ClearLine(LCD_LINE_5);
	LCD_ClearLine(LCD_LINE_6);
	LCD_ClearLine(LCD_LINE_7);
	LCD_ClearLine(LCD_LINE_8);	
	
	
		if (state < 3){ //If we're not setting anything yet
			state = 4; // setting hour
		}
		else if (state == 4){
			state = 5; //minute
		}
		else if (state == 5){
			state = 6; //second
		}
		else if (state == 6){
			state = 7; //month
		}
		else if (state == 7){
			state = 8; //number date
		}
		else if (state == 8){
			state = 9; //year
		}
		else if (state == 9){
			state = 0; //HOME
		}

		//don't forget to clear the pending bit
		EXTI_ClearITPendingBit(EXTI_Line2);

}

void EXTI3_IRQHandler(void){ //Button 3 sets tims
	//your code here - increment number of time and date

//In any state that we're incrementing something,
	//increment the adder 
	if (state <= 4){
	 adder++;
	}
	//don't forget to clear the pending bit
	EXTI_ClearITPendingBit(EXTI_Line3);
}


/*
 * This can be used to handle the RTC alarm interrupts. If you are
 * using alarm A or B you can configure the alarms to trigger every second.
 */
void RTC_Alarm_IRQHandler(void){ //Each second when the alarm interrupts
	ClkRefresh=1; //Tell the while loop to refresh the screen
	
	
	STM_EVAL_LEDToggle(LED3); // just to do error checking
	RTC_ClearITPendingBit(RTC_IT_ALRA);
	EXTI_ClearITPendingBit(EXTI_Line17);
	//make sure to clear pending bit
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
