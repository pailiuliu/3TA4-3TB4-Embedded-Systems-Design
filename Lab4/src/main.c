#include "main.h"
#include "stm32f4xx.h"
//#include "stm324xg.h"
//#include "stm324xg_eval.h"
//#include "stm324xg_eval_lcd.h"
#include <stdio.h>

//the following two addresses are useful when using the ADC and DAC in DMA mode

#define ADC_CDR_ADDR 				((uint32_t)(ADC_BASE + 0x08)) //ADC_CDR address as described in section 10.13.17 of reference manual
//ADC CDR is the common regular (not injected) data register for dual and triple modes
//also defined in stm32f4xx_adc.c as 
//#define CDR_ADDRESS 			((uint32_t)0x40012308) 
//which results in the same address
#define ADC3_DR_ADDRESS    ((uint32_t)0x4001224C)

void ADC3_Config(void);
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint16_t ADC3ConvertedVoltage = 0;
char lcd_buffer[15];
char lcd_buffer2[15];
void Display();

//PWM
void PWM_Config(void);
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 0;
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;
uint16_t PrescalerValue = 0;
int PWMValue=0;

//external push buttons
void PB1_Config(void);
void PB2_Config(void);

//TIM2
//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint16_t CCR_Val = 50000;
uint16_t PrescalerValue2 = 0;
void TIM2_Config(void);
void TIM2_OCConfig(void);


int settemp = 30;
int temp = 25;
extern int count;


int main(void){
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDOn(LED4);
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
	PB2_Config();
	PB1_Config();
	
	/***********************Implement the following functions*****************/
	
	//configure PWM output
	PWM_Config();
	
	LCD_Clear(LCD_COLOR_WHITE);
	
	//Display a string in one line, on the first line (line=0)
	LCD_DisplayStringLine(LINE(0),  (uint8_t *) "PWM Fan control");
	
	
	//TIM2
	PrescalerValue2 = (uint16_t) ((SystemCoreClock / 2) / 5000000) - 1; 
	TIM2_Config();
	TIM2_OCConfig();
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
	
	
	//configure ADC device
	ADC3_Config();
	/* Start ADC3 Software Conversion */ 
  ADC_SoftwareStartConv(ADC3);
	
	//PWM COnfig
	
	while(1){
		//measure room temperature with an LM35
		//convert the output voltage form the LM35 to a number using an A/D converter in the MCU
		ADC3ConvertedVoltage = (ADC3ConvertedValue-300)/30;
		
		//display this temperature on the LCD along with the fan setpoint temperature
		LCD_DisplayStringLine(LINE(2),  (uint8_t *) "Current Temp(C): ");
		if (count == 49){
			lcd_buffer[3] = '\0'; 
			sprintf(lcd_buffer,"%d", ADC3ConvertedVoltage);
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) lcd_buffer);
			temp = ADC3ConvertedVoltage;
		}
					
		sprintf(lcd_buffer2,"%d", temp);
		lcd_buffer2[2] = '\0'; 
		lcd_buffer2[3] = '\0';
		lcd_buffer2[4] = '\0'; 
		LCD_DisplayStringLine(LINE(10),  (uint8_t *) lcd_buffer2);

		//set temperature settemp
		
		if (GPIO_ReadInputDataBit(GPIOA, 2)==1){		//PA1 increase settemp
			if (count == 49){
				settemp = settemp +1;
				count =0;
			}
		}
		if (GPIO_ReadInputDataBit(GPIOA, 1)==1){		//PA0 decrease settemp
			if (count == 49){
				settemp = settemp - 1;
				count = 0;
			}
		}
		//displaying settemp
		LCD_DisplayStringLine(LINE(5),  (uint8_t *) "Set Temp(C): ");
		sprintf(lcd_buffer2,"%d", settemp);
		LCD_DisplayStringLine(LINE(6),  (uint8_t *) lcd_buffer2);

		sprintf(lcd_buffer2,"%d", count);
		LCD_DisplayStringLine(LINE(9),  (uint8_t *) lcd_buffer2);
		
		
		//If the actual sensor temperature is above the setpoint, turn on the fan
		//When the fan turns on, it should spin slowly (about 1/2 power).  
		//As the sensor gets warmer, the fan should increase its speed (reaching full power eventually)
	//	TIM_Compare1(TIM4, PWMValue);
		PWM_Config();

		if (temp == settemp){
			LCD_DisplayStringLine(LINE(7),  (uint8_t *) "Fan Half ON ");
			CCR1_Val = 333/2;
			CCR2_Val = 249/2;
			CCR3_Val = 166/2;
			CCR4_Val = 83/2;

		}
		else if ( temp > settemp){
			LCD_DisplayStringLine(LINE(7),  (uint8_t *) "Fan FULL ON ");
			CCR1_Val = 333;
			CCR2_Val = 249;
			CCR3_Val = 166;
			CCR4_Val = 83;
		}
		//turn itself off  when the temperature sensor reads a temperature smaller than the fan setpoint
		else{
			LCD_DisplayStringLine(LINE(7),  (uint8_t *) "Fan OFF           ");
			CCR1_Val = 0;
			CCR2_Val = 0;
			CCR3_Val = 0;
			CCR4_Val = 0;
		}
	};
	
}

/**
 * Use this function to configure the ADC input.
 */
void ADC3_Config(void){
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}




/**
 * Use this function to configure the PWM output.
 */
void PWM_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM3); 
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
	
}


//either of the two buttons labelled increase and decrease for more than 1/2 second, 
//the set point is modified in the appropriate direction, at about 1 degree every 0.5 second
//displayed on the LCD
//When the button is released, the new set-point and current temperature are displayed.

void PB1_Config(void){
  EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
  GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
  NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
	
	// Using pin PA1
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
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
  GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
  NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
	
	// Using pin PA2
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

void TIM2_Config(void) 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//since TIMER 2 is on APB1 bus, need to enale APB1 bus clock first
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//====================================================
	//Enable TIM3 global interrupt ====does this part need to be done before TIM_BaseStructure set up?
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0X00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//================================================
	
	TIM_TimeBaseStructure.TIM_Period=65535; // need to be larger than CCR_VAL, has no effect on the Output compare event.
	TIM_TimeBaseStructure.TIM_Prescaler=PrescalerValue2;    //why all the example make this one equal 0, and then use 
					//function TIM_PrescalerConfig() to re-assign the prescaller value?
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//TIM_PrescalerConfig(TIM3, TIM3Prescaler, TIM_PSCReloadMode_Immediate);
}

void TIM2_OCConfig(void) {
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=CCR_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable); //if disabled, 
	//the TIMx_CCRx register can be updated at any time by software to control the output
	//waveform---from the reference manual
}


