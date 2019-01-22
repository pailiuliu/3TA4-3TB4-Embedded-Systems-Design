/*
State 0 = Pin PE2, PE5
State 1 = Pin PE2
State 2 = Pin PE2, PE3
State 3 = Pin PE3
State 4 = Pin PE3, PE4
State 5 = Pin PE4
State 6 = Pin PE4, PE5
State 7 = Pin PE5
*/



#include "stm32f4xx_it.h"
#include "stm32f429i_discovery.h"
#include "main.h"
__IO uint32_t RTCAlarmCount = 0;

extern __IO uint8_t AlarmUp;
char lcd_buffer2[15];
extern uint16_t PrescalerValue;

extern int speed;
extern int stepmode;
extern int direction;
extern int count;
int state = 1;

extern void TIM3_Config(void);
extern void TIM3_OCConfig(void);

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */

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
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)	//if TIM_GetITStatus does not return reset value
  {
		count++;
		if (count >=50){
//			STM_EVAL_LEDToggle(LED4);
//			LCD_DisplayStringLine(LINE(9),  (uint8_t *) "       ");
			count =0; }
		
		if (stepmode == 0 && direction == 0){ //halfstep, clockwise
			LCD_DisplayStringLine(LINE(1),  (uint8_t *) "Halfstep, CCW   ");
			state = (state + 1)%8;
		}
		else if (stepmode == 0 && direction == 1){  //halfstep, counterclockwise
			LCD_DisplayStringLine(LINE(1),  (uint8_t *) "Halfstep, CW   ");
			state = (state + 7)%8;
		}
		else if (stepmode == 1 && direction == 0){ //fullstep, clockwise
			
			LCD_DisplayStringLine(LINE(1),  (uint8_t *) "Fullstep, CCW   ");
			state = (state + 2)%8;
		}
		else if (stepmode == 1 && direction == 1){ //fullstep, counterclockwise
			LCD_DisplayStringLine(LINE(1),  (uint8_t *) "Fullstep, CW   ");
			state = (state + 6)%8;
		}
		switch(state){
			case 0:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 0,  PE4&2   ");
				break;
			case 1:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 1, PE4   ");
				break;	
			case 2:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 2, PE4&3   ");
				break;
			case 3:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 3, PE3   ");
				break;
			case 4:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 4, PE3&5   ");
				break;
			case 5:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 5, PE5   ");
				break;
			case 6:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 6, PE2&5   ");
				break;
			case 7:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 7, PE2   ");
				break;
		}
		
		/*
		switch(state){
			case 0:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 0,  PE5&2   ");
				break;
			case 1:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 1, PE2   ");
				break;	
			case 2:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 2, PE2&3   ");
				break;
			case 3:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 3, PE3   ");
				break;
			case 4:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_3);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 4, PE3&4   ");
				break;
			case 5:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 5, PE4   ");
				break;
			case 6:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_4);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 6, PE4&5   ");
				break;
			case 7:
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
				GPIO_ResetBits(GPIOE, GPIO_Pin_3);
				GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				GPIO_ResetBits(GPIOE, GPIO_Pin_5);
				GPIO_SetBits(GPIOE, GPIO_Pin_5);
				LCD_DisplayStringLine(LINE(7),  (uint8_t *) "State 7, PE5   ");
				break;
		}
		*/
						
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	//clear pending bit
	}
	TIM_SetCounter(TIM3, 0x0000);
}

/**
 * External interrupt channel 0 Interrupt Handler. This handles
 * the user button.
 */
void EXTI0_IRQHandler(void){
	
	//SWITCH DIRECTIONS
		if (direction == 0){
			direction = 1;
		}
		else if (direction == 1){
			direction = 0;
		}
	
	
	//clear the pending bit otherwise the handler will fire continually
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	//this is configured to handle the push-button
}

void EXTI1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
		speed = speed - 50; 
		sprintf(lcd_buffer2,"speed %d     ", speed);
		LCD_DisplayStringLine(LINE(6),  (uint8_t *) lcd_buffer2);
		
		//PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / speed) - 1; 
		PrescalerValue= speed;
		TIM3_Config();
		TIM3_OCConfig();
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		TIM_Cmd(TIM3, ENABLE);
    
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
//		speed = speed - 1000;
//		sprintf(lcd_buffer2,"speed %d", speed);
//		LCD_DisplayStringLine(LINE(6),  (uint8_t *) lcd_buffer2);

		if (stepmode == 0){ // mode 0 is half step
			speed = ((speed+1)*2)-1;
			stepmode = 1; //go to full step
			state = 1;
		}
		else if (stepmode == 1){
			speed = ((speed+1)/2)-1;
			stepmode = 0;
			state = 1;
		}
    
		PrescalerValue= speed;
		TIM3_Config();
		TIM3_OCConfig();
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		TIM_Cmd(TIM3, ENABLE);
    /* Clear the EXTI line 2 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}

void EXTI3_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
		speed = speed + 50;
		sprintf(lcd_buffer2,"speed %d    ", speed);
		LCD_DisplayStringLine(LINE(6),  (uint8_t *) lcd_buffer2);
		
		PrescalerValue = speed; 
		TIM3_Config();
		TIM3_OCConfig();
		TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
		TIM_Cmd(TIM3, ENABLE);
		
//		LCD_DisplayStringLine(LINE(6),  (uint8_t *) "mode    ");
//		if (stepmode == 0){
//			stepmode = 1;
//			state = 1;
//		}
//		else if (stepmode == 1){
//			stepmode = 0;
//			state = 1;
//		}
    
    /* Clear the EXTI line 2 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}