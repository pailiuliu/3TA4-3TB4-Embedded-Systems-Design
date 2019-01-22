/**
  ******************************************************************************
  * @file    Lab2/stm32f4xx_it.c
  * @author Rert Li
  * @version 
  * @date    August 2014
  * @brief   
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"


/** @addtogroup Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern __IO uint8_t UBPressed;
extern int counter;
extern int state;

int pre = 0; //pre initiated to 0
//int state = 0;
int check = 0;	//check initiated to 0
int pause = 0;	//pause initiated to 0
int D;		//D
int count = 0;	//count initiated to 0
// int hiscore = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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
  {
  }
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
  {
  }
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
  {
  }
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
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


/******************************************************************************/
/*            STM32F4xx Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void) //function for timer 3 IRQ handler
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)	//if TIM_GetITStatus does not return reset value
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	//clear pending bit


    if (state == 0){	//if state is 0
		//before any button presses
		// Because the clock frequency is 1000Hz, we only want it to toggle every 500ms
			pre++;	//increment pre variable every interrupt
			if (pre >= 500){	//if pre is greater or equal to 500; allows for slower toggles
				STM_EVAL_LEDToggle(LED4);	//toggle LED4
				STM_EVAL_LEDToggle(LED3);	//toggle LED3
				pre = 0; //set pre to 0 again to be incremented upon each interrupt, toggles everytime pre hits 500
			}
			
		}

		else if (state == 1){	//if state is 1
		//LEDs are off for 2ish seconds
			if (check != pause){	//if check does NOT equal to pause
				check++;			//increment check for each interrupt,
			}
			else if (check == pause){	//if check equals to pause
				// allow button presses again?
				state = 2;		//set state to 2
				STM_EVAL_LEDOn(LED4);	//turn LED4 on after 2ish seconds
				STM_EVAL_LEDOn(LED3);	//turn LED3 on afte 2ish seconds
				check = 0;				//check is returned to 0 to be incremented again
			}
		}
		
		else if(state == 2){ //if state is 2
			//As soon as LEDs go back on, counting time
			count++;	//increment count
			}
		
		else if	(state == 3){ //
		//After button is pressed computing time and outputting
			
		}
			
		}
		
		
		TIM_SetCounter(TIM3, 0x0000); 

}

 

void EXTI0_IRQHandler(void){
	/* add user-button handling code here */

	/* don't execute the ISR until the button is released */
//	while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET);
	
// These are the possible button press changes. 
// State 0 refers to the toggling LEDs before any button is pressed
// State 1 refers to turning LEDs off for 2ish seconds
// State 2 is the period immediately after the LEDs turn on, counting miliseconds
// State 3 is after the button is pressed, where score is computed 
	
	if(state==0){		//state 0 code
		state = 1;		//changed state to 1
		STM_EVAL_LEDOff(LED4);	//turn LED4 off
		STM_EVAL_LEDOff(LED3);	//turn LED4 off
		D = RNG_GetRandomNumber() % 1000;	//generates a gerandomnumber and using modulus operator, obtain number within 0-999 range to assign to D
		pause = 1500 + D; //assign D plus 1500 to pause, for pause to be 1500 to 2499 range
}
	
	else if (state==2){	//state 2 code

		
		if (count < 5){		//if count is less than 5
			counter=0;		//counter has 0 value
			state= 0;		//state has 0 value
			//Allow button presses
		}
		else {			//if count is NOT less than 5
			STM_EVAL_LEDOff(LED3);	//LED3 turns off
			counter = count;		//count is assigned to counter
			//	if (counter < hiscore){ //should do this in the main function
			//		hiscore = counter;}
			state = 3;		//change state is 3
				}	
			
		}
	

	
	UBPressed = 1; //Userbutton pressed initiated to 1
	
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
} //Onboard Button Interrupt Handler 




/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                                               */
/******************************************************************************/

void EXTI1_IRQHandler(void){
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
		state=0;
    
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
