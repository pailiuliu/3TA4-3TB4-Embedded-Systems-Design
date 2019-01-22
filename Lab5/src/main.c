/*
PA1 = speed up
PA2 = switch stepping mode
PA3 = speed down

PE2, PE3, PE4, PE5 motor connection
*/


#include "main.h"
#include <stdio.h>
#include "stm32f4xx.h"

void GPIO_Config(void);
char lcd_buffer[15];

__IO uint16_t CCR_Val = 25000;
uint16_t PrescalerValue;
void TIM3_Config(void);
void TIM3_OCConfig(void);


void PB1_Config(void);
void PB2_Config(void);
void PB3_Config(void);
void Motor_Config(void);

int speed;
int stepmode = 0; //0 = halfstep, 1 = fullstep
int direction = 0; //0 = clockwise, 1 = counterclockwise
int count;



int main(void){
	LCD_Init();/* LCD Layer initiatization */
  LCD_LayerInit(); 
  LTDC_Cmd(ENABLE);	/* Enable the LTDC */
  LCD_SetLayer(LCD_FOREGROUND_LAYER); /* Set LCD foreground layer */
	LCD_Clear(LCD_COLOR_WHITE);
	LCD_DisplayStringLine(LINE(0),  (uint8_t *) "Step Motor Gang");
	
	GPIO_Config();
	
	speed = 2474;
//	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / speed) - 1; 
	PrescalerValue = speed;
	TIM3_Config();
	TIM3_OCConfig();
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
	
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	PB1_Config();
	PB2_Config();
	PB3_Config();
	
	Motor_Config();
	

	
	while(1)
	{
		sprintf(lcd_buffer,"stepmode %d", stepmode);				//Display stepmode, stepmode = 0 for halfstep, stepmode = 1 for fullstep
		LCD_DisplayStringLine(LINE(8),  (uint8_t *) lcd_buffer);
		sprintf(lcd_buffer,"direction %d", direction);				//Display direction, direction = 0 for CCW, direction = 1 for CW
		LCD_DisplayStringLine(LINE(9),  (uint8_t *) lcd_buffer);	
		sprintf(lcd_buffer,"%d  ", count);				//Display count for TIM3
		LCD_DisplayStringLine(LINE(10),  (uint8_t *) lcd_buffer);
		
		
	};
}

/**
 * Configure the GPIO for output to the motor.
 */
void GPIO_Config(void){

}

/**
 * Configure the TIM3 in output compare mode.
 */
void TIM3_Config(void){
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
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValue;    //why all the example make this one equal 0, and then use 
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
	TIM_OCInitStructure.TIM_Pulse=CCR_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable); //if disabled, 
	//the TIMx_CCRx register can be updated at any time by software to control the output
	//waveform---from the reference manual
}


void UB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

void PB1_Config(void){	
	// Using pin PA1
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
  GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
  NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
// Enable GPIO Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable peripheral clock AHB1
// Enable SYSCFG clock	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//enable peripheral clock AHB2
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Needs to be UP to not burn out the board
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//using pull UP mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		//initiate GPIO pin 1
  GPIO_Init(GPIOA, &GPIO_InitStructure);		//GPIO structure
	
	  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1); /* Connect EXTI Line1 to PA1 pin */

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;	//initiate EXTI structure to line 1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//initiate EXTI structure to interrupt mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //initiate EXTI structure to detect the rising edge, positive signal
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//eable line command to EXTI
  EXTI_Init(&EXTI_InitStructure);	//EXTI structure
	
	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;	//initiate NVIC structure to IRQ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;	//set  NVIC structure preemption priority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F; //set  NVIC structure preemption -subpriority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //enable NVIC structure command
  NVIC_Init(&NVIC_InitStructure); //NVIC structure	
}

void PB2_Config(void){
	// Using pin PA2
	
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
	GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
	NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
// Enable GPIO Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable peripheral clock AHB1
// Enable SYSCFG clock	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//enable peripheral clock AHB2
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Needs to be UP to not burn out the board
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//using pull UP mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		//initiate GPIO pin 2
  GPIO_Init(GPIOA, &GPIO_InitStructure);		//GPIO structure
	
	  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2); /* Connect EXTI Line2 to PA2 pin */

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;	//initiate EXTI structure to line 2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//initiate EXTI structure to interrupt mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //initiate EXTI structure to detect the rising edge, positive signal
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//eable line command to EXTI
  EXTI_Init(&EXTI_InitStructure);	//EXTI structure
	
	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;	//initiate NVIC structure to IRQ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;	//set  NVIC structure preemption priority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F; //set  NVIC structure preemption -subpriority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //enable NVIC structure command
  NVIC_Init(&NVIC_InitStructure); //NVIC structure
}

void PB3_Config(void){
	// Using pin PA4
	
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
	GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
	NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
	
// Enable GPIO Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //enable peripheral clock AHB1
// Enable SYSCFG clock	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//enable peripheral clock AHB2
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Needs to be UP to not burn out the board
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//using pull UP mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//initiate GPIO pin 2
  GPIO_Init(GPIOD, &GPIO_InitStructure);		//GPIO structure
	
	  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource3); /* Connect EXTI Line2 to PA2 pin */

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;	//initiate EXTI structure to line 2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//initiate EXTI structure to interrupt mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //initiate EXTI structure to detect the rising edge, positive signal
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//eable line command to EXTI
  EXTI_Init(&EXTI_InitStructure);	//EXTI structure
	
	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;	//initiate NVIC structure to IRQ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;	//set  NVIC structure preemption priority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F; //set  NVIC structure preemption -subpriority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //enable NVIC structure command
  NVIC_Init(&NVIC_InitStructure); //NVIC structure	
}

void Motor_Config(void){
	GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
	
	/* Enable GPIOD's AHB interface clock */
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure PE2, PE3, PE4, PE5 in output pushpull mode */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
 /* Set PE1 to high level 
 GPIO_SetBits(GPIOE, GPIO_Pin_1);
 */
}
