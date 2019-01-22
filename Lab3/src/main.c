#include "main.h"


//memory location to write to in the device
__IO uint16_t mem1 = 0x0004; //pick any location within range
__IO uint16_t mem2 = 0x0008; //These are the memory locations in the EEPROM
__IO uint16_t mem3 = 0x000C;	// They are 4 bytes away  because that is one word length away
__IO uint16_t mem4 = 0x0004;
__IO uint16_t mem5 = 0x0010;
__IO uint16_t mem6 = 0x0014;

uint8_t Tx1_Buffer = 'A';
uint8_t Rx1_Buffer;
uint16_t NumDataRead = 1;

uint16_t line;
int ClkRefresh=0; // Keeps track of the interval between clock refreshed
uint8_t Hour; // holds the current hour for showing the current time
uint8_t Min; // holds the current minute for showing the current time
uint8_t Sec; // holds the current secind for showing the current time

uint8_t CHour; // these three hold the time that the user button is pressed
uint8_t CMin; // They are externed in the interrupt file
uint8_t CSec;

uint8_t Day; //holds today's date
uint8_t Month;
uint8_t Year;

int adder=0; //this increments the times and dates 

int sethour = 0; 
int setminute = 0;
int setsecond = 0;
int setmonth = 0;
int setdate = 0;
int setyear = 0;

int recordhour1; //holds the second last time the button's been pressed
int recordminute1;
int recordsecond1;

int recordhour2; // holds the last time the button's been pressed
int recordminute2;
int recordsecond2;

char CTime[15]; //String buffer for outputting the current time
char CDate[15]; //string buffer for the current date

int state=0; //the FSMD state of the system
int EEDone=0; //should EEPROM be written to? 

#define COLUMN(x) ((x) * (((sFONT *)LCD_GetFont())->Width))    //see font.h, for defining LINE(X)

// 
	RTC_InitTypeDef RTC_InitStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;

//


int main(void){

	STM_EVAL_LEDInit(LED3);
	
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

	
	
	//======You need to develop the following functions======
	//Note: these are just placeholders; function definitions are at bottom of this file
	//configure real-time clock
	RTC_Config();
	
	//configure external push-buttons and interrupts
	ExtPB_Config();
	
	
	//main program
	
	LCD_Clear(LCD_COLOR_WHITE);
		
//	line=0;
	//Display a string in one line, on the first line (line=0)
//	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Init EEPROM...");
//	line++;
	
	//i2c_init(); //initialize the i2c chip
	sEE_Init();  

	LCD_DisplayStringLine(LINE(8),  (uint8_t *) "ABCDEFGHIJKLMNO"); // Just for testing
//	line++;
	
//	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Writing...");
//	line++;
	
	
	/* First write in the memory followed by a read of the written data --------*/
  /* Write on I2C EEPROM from memLocation */
  //sEE_WriteBuffer(&Tx1_Buffer, memLocation,1); 
// sEE_WaitEepromStandbyState();


  /* Wait for EEPROM standby state */
  //sEE_WaitEepromStandbyState();  
 
  
//	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Reading...");
  /* Read from I2C EEPROM from memLocation */
  //sEE_ReadBuffer(&Rx1_Buffer, memLocation, (uint16_t *)(&NumDataRead)); 
//	line++;
	
//	LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Comparing...");  
//	line++;
	
	/*
	if(Tx1_Buffer== Rx1_Buffer){
		LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Success!");  
	}else{
		LCD_DisplayStringLine(LINE(line),  (uint8_t *) "Mismatch!"); 
	}
	*/
	
	//main loop
	
	while(1){
		
		//this is showing the current time, state 0
		
		
		if (ClkRefresh==1){ /// SCREEN THINGS IN THIS BLOCK
			//Basically, the screen takes a long time to update. It is only updated once every second
			// All code that relates to updating the screen is in here
			
		LCD_ClearLine(LCD_LINE_1); //clears the LED line to make way for the new stuff
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure); //gets the current time
		Hour= RTC_TimeStructure.RTC_Hours; //assigns the number to the appropriate spot
		Min= RTC_TimeStructure.RTC_Minutes;
		Sec= RTC_TimeStructure.RTC_Seconds;

			//char CTime[15];
			sprintf(CTime,"%0.2d:%0.2d:%0.2d",Hour, Min, Sec); // prints the numbers into formatting into buffer
			LCD_DisplayString(0, 0, (uint8_t *) CTime); //actually displays the string
			ClkRefresh=0;	
		// showing current time
		

	if (state==1){ //Slow LED Screen things
		if (STM_EVAL_PBGetState(BUTTON_USER)==1){ //If the user button is still high aka pressed, 
	//display date
		LCD_ClearLine(LCD_LINE_2);
		
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure); //getting current date
 
		Day=RTC_DateStructure.RTC_Date; //assigning to appropriate holders
		Month=RTC_DateStructure.RTC_Month;
		Year= RTC_DateStructure.RTC_Year;
		
		sprintf(CDate,"%0.2d/%0.2d/%0.2d",Day, Month, Year); // outputting with formatting to the buffer
		LCD_DisplayString(1,0, (uint8_t*) CDate); //output to LCD
		//	LCD_DisplayString(1,0, (uint8_t*) "Button");
			
			//We keep updating this every second until the user button is no longer pressed. 
				// When it's not, we clear the line and go back to state 0.
	
		}
		else {
			LCD_ClearLine(LCD_LINE_2); // Takes off the date line
			state=0;
			} 
		} //State 1 end
	
	else if (state==2){ // Showing last two things fro EEprom
		//get from EEProm
		/*
		 recordhour1;
		recordminute1;
		recordsecond1;

		recordhour2;
		recordminute2;
		recordsecond2;
		*/
		
		//I wish we could have gotten this completed in time :/
		
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Last Recorded: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) "INSERT EEPROM 1");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) "INSERT EEPROM 2");
	
	}
		////////////////////////////////////////////////
	/*
	int sethour = 0;
int setminute = 0;
int setsecond = 0;
int setmonth = 0;
int setdate = 0;
int setyear = 0;

*/	
	
	// These states are for setting the time / date
	/*
	In each case:
Adder is incremented in the third button interrupt. This is so that it doesn't depend on the one-second-display-refresh
You can increment it as many times as you want in one second, each second it will update how many increments you want to go up
The modulus functions make sure tht you don't go over the limit for each number type, and that the number always rolls over to 0.
	Example: going from 59 minutes to 0 minutes. That would be mon 60

Then, adder is set back to 0 so we don't add it to the total twice
The new time/date is then put into the structure, and then the SetDate or SetTime is used to update it.

Finally, it is outut onto the LCD
	
	*/
			else if (state == 4){ //Setting the hour
				RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure); //get current time
				sethour = (RTC_TimeStructure.RTC_Hours + adder)%24; // adds the adder to the current, and the mod 24 hours
				adder=0;			// Then reset Adder so we don't add it twice
				RTC_TimeStructure.RTC_Hours = sethour; // reset the hour
				RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure); // set the time
			//set hour
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Hour: "); // display
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) sethour);
	
		}
				
		else if (state == 5){ // etting the minute
			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure); //get current time
			setminute = (RTC_TimeStructure.RTC_Minutes + adder)%60; // adds the adder
			adder=0;
			RTC_TimeStructure.RTC_Minutes= setminute;
			RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
			
			//set minute
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Minute: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) setminute);

		}
		
		else if (state == 6){ // Setting the seconds
			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
			setsecond = (RTC_TimeStructure.RTC_Seconds + adder)%60;
			adder=0;
			RTC_TimeStructure.RTC_Seconds=setsecond;
			RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
			//set seconds
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Second: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) setsecond);

		}
		
		else if (state == 7){ //Setting the month
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
			setmonth=(RTC_DateStructure.RTC_Month+adder)%12;
			adder=0;
			RTC_DateStructure.RTC_Month=setmonth;
			RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			//set month
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Month: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) setmonth);
	
		}
		else if (state == 8){ // setting the day
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
			setdate=(RTC_DateStructure.RTC_Date+adder)%31;
			adder=0;
			RTC_DateStructure.RTC_Date=setdate;
			RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			
			//set date
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Date: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) setdate);

		}
		else if (state == 9){ // setting the year
			RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
			setyear=(RTC_DateStructure.RTC_Year+adder)%100;
			adder=0;
			RTC_DateStructure.RTC_Year=setyear;
			RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
			
			//set year
			LCD_DisplayStringLine(LINE(3),  (uint8_t *) "Set Year: ");
			LCD_DisplayStringLine(LINE(4),  (uint8_t *) setyear);
	
		}
	
	} // end of clock cycle LED screen update things
		
	// Anything below here is asynchronous with the one second LCD update rate
	
	if (EEDone == 1){ //this is while the button is pressed
		
		//EEDone will be one is the user button is pushed. Since it should only happen once, it goes below the LCD code
		//I wish we finished saving things to EEPROM
		//saving to EEPROM
		  sEE_WriteBuffer(&Tx1_Buffer, memLocation, 1); 
// sEE_WaitEepromStandbyState();
	
		
		
		EEDone=0; // After we're done saving, we reset EEDone back to 0 so we don't save again
		}		// EEPROM	
			



		

	
	
	}  //end of while
	
} // end of main




void PB_Config(void)
{
/* Initialize User_Button on STM32F4-Discovery
   * Normally one would need to initialize the EXTI interrupt
   * to handle the 'User' button, however the function already
   * does this.
   */
	
	
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}


/**
 * Use this function to configure the GPIO to handle input from
 * external pushbuttons and configure them so that you will handle
 * them through external interrupts.
 */
void ExtPB_Config(void){

// We made three ExtPB functions to handle each of the buttons separately	
	
ExtPB3_Config();
ExtPB2_Config();
ExtPB1_Config();
	
}

void ExtPB1_Config(void){
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
void ExtPB2_Config(void){
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
void ExtPB3_Config(void){
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
  GPIO_InitTypeDef   GPIO_InitStructure;//initiate GPIO structure and type
  NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
	
	// Using pin PA3
// Enable GPIO Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable peripheral clock AHB1
// Enable SYSCFG clock	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//enable peripheral clock AHB2
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Needs to be UP to not burn out the board
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//using pull UP mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//initiate GPIO pin 3
  GPIO_Init(GPIOA, &GPIO_InitStructure);		//GPIO structure
	
	  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3); /* Connect EXTI Line3 to PA3 pin */

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;	//initiate EXTI structure to line 3
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

/**
 * Configure the RTC to operate based on the LSI (Internal Low Speed oscillator)
 * and configure one of the alarms (A or B) to trigger an external interrupt every second
 * (e.g. EXTI line 17 for alarm A).
 */



void RTC_Config(void){
	
	/* This is all global at the top
	RTC_InitTypeDef RTC_InitStructure;
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	*/
		
	EXTI_InitTypeDef   EXTI_InitStructure; //initiate EXTI structure and type
  NVIC_InitTypeDef   NVIC_InitStructure;//initiate NVIC structure and type
	
 /* Enable write access to the RTC ****************************/
 /* Enable the PWR clock */
	
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
 /* Allow access to RTC */
	
 PWR_BackupAccessCmd(ENABLE);
 /* Configure the RTC clock source ****************************/
 /* Enable the LSE OSC */
	
 RCC_LSEConfig(RCC_LSE_ON);
 /* Wait till LSE is ready */
 while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){}
 
	 /* Select the RTC Clock Source */
 RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
 /* Enable the RTC Clock */
 RCC_RTCCLKCmd(ENABLE);
 /* Wait for RTC APB registers synchronisation */
 RTC_WaitForSynchro();
 /* Configure the RTC calendar, Time and Date *****************/
 /* RTC time base = LSE / ((AsynchPrediv+1) * (SynchPrediv+1))
 = 1 Hz
 */
 RTC_InitStructure.RTC_AsynchPrediv = 127;
 RTC_InitStructure.RTC_SynchPrediv = 255;
 RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
 RTC_Init(&RTC_InitStructure);
 /* Set the Time */
 RTC_TimeStructure.RTC_Hours = 0x00;
 RTC_TimeStructure.RTC_Minutes = 0x00;
 RTC_TimeStructure.RTC_Seconds = 0x00;
RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

 /* Set the Date */
 RTC_DateStructure.RTC_Month = RTC_Month_November;
 RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Wednesday;
 RTC_DateStructure.RTC_Date = 0x01;
 RTC_DateStructure.RTC_Year = 0x17;
 RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure); 
 
 /// INTERUPTS
	
	// Using exti 17, this exti line is for RTC Alarm interrupts

  /* Configure EXTI Line17 */
	EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;	//initiate EXTI structure to line 1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//initiate EXTI structure to interrupt mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //initiate EXTI structure to detect the rising edge, positive signal
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;	//eable line command to EXTI
  EXTI_Init(&EXTI_InitStructure);	//EXTI structure
	
	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;	//initiate NVIC structure to IRQ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;	//set  NVIC structure preemption priority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F; //set  NVIC structure preemption -subpriority to hex 0F
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //enable NVIC structure command
  NVIC_Init(&NVIC_InitStructure); //NVIC structure
 

	//RTC_AlarmStructInit(&RTC_AlarmStructure);
  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All;
 
 //RTC_SetAlarm()
 RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
 RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
 
 
 RTC_ITConfig(RTC_IT_ALRA, ENABLE);
 
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
/*
End of code
*/

