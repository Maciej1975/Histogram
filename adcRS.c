#include "adcRS.h"
#include "buffer.h"

//uint8_t EEMEM LoggingInterval500MS_EEPROM;
//uint8_t LoggingInterval500MS_SRAM;
uint16_t CurrentLoggingTicks;


uint8_t isr_dbg2=0;
uint8_t randNr;

uint16_t ADC_Temp;// = ADC_GetChannelReading(ADC_CHANNEL7 | ADC_REFERENCE_AVCC ); //MUX mask: ADC channel mask, reference mask and adjustment mask | ADC_RIGHT_ADJUSTED	 
uint16_t ADC_Temp1, ADC_Temp2, ADC_Temp4, ADC_Temp5, ADC_Temp6, ADC_Temp7; //current value
uint16_t ADC_Temp1b, ADC_Temp2b, ADC_Temp4b, ADC_Temp5b, ADC_Temp6b, ADC_Temp7b; //current value, second buffer
uint16_t ADC_Max1=0xFFFF, ADC_Max2=0xFFFF, ADC_Max4=0xFFFF, ADC_Max5=0xFFFF, ADC_Max6=0xFFFF, ADC_Max7=0xFFFF;	//find max value of sensor describing darkest area of bright background - black line 
uint8_t S1=0, S2=0, S3=0, S4=0; //stan czujnika linii; ==0 to ciemny, ==1 to jasny
 
 
/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
char StrinOut[50] ;
char StrinIn[50] ;
uint16_t tADC[6], tADCmin[6], tADCmax[6]= {0, 0, 0, 0, 0, 0}, tADCdel[6]= {0, 0, 0, 0, 0, 0};
uint8_t calibrated;
unsigned int valRcv; //received value
unsigned int volt; //received value
unsigned int curr; //received value
const unsigned char *pTemp;
 

 
	volatile unsigned char HState; //z wewnatrz przrwania
	volatile char Xstate = 0;
	

	//#define tmpDly 400
	unsigned char tmpDly = 0;
 


ISR( PCINT0_vect, ISR_BLOCK )
{
	//unsigned char HState;

		LEDs_ToggleLEDs(LEDS_LED3);
		
	//} //else
}
 
int main(void) {
		
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	USB_Init();	// /usb/LowLevel/LowLevel.c
	LEDs_Init();
	
	SetupHardware();
		
	//--- Histogram tests

	h_buff_t buffer01;

	buffer01.buff[0].bin[0] = 33;
	buffer01.buff[0].bin[1]++;
	buffer01.max_total = 200;
	h_buff_clean_unit(&buffer01, 0);


	//--- Histogram tests


	sei();

	for(int u=0; u<200; u++){
		_delay_ms(10);
	}
	
	for(;;){
		//while ( !RecvStringUART(StrinIn) );  //wait for RS data
		//receive string: u20z (char+number+delimiter)
		
		if (RecvStringUART(StrinIn))
		{
			//SendStringUART("\r\nReceived: ");

			
			switch (StrinIn[0]) {
				
				//setting options:
				case 'q':	//duty cycle 1st phase
					if (valRcv <= ICR3) 
						OCR3A = valRcv;
					else
						OCR3A = ICR3;
					sprintf(StrinOut, "Duty %d. Cycles %d.\r\n", OCR3A, ICR3);
				break;

				case 'w':	//PWM period = 1/frequency
					if ( (valRcv <= 255) && (valRcv > 20) ) 
						ICR3 = valRcv;
					else
						valRcv = ICR3;
					sprintf(StrinOut, "Cycles: %d\r\n", ICR3);
				break;				
				
				default:
					sprintf(StrinOut, "Unknown message:(\r\n");
					
			}
			SendStringUART(StrinOut);
		} //if USART

	}
	

}

/** Opens the log file on the Dataflash's FAT formatted partition according to the current date */


/** Configures the board hardware and chip peripherals for the demo's functionality. */
inline static void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	LEDs_Init();
	Buttons_Init();	//button HWB
	Joystick_Init();

	//gate driving output - PORTC.6 / PWM-A Timer3
	DDRC |= (1<<PC6);
	PORTC &= ~(1<<PC6);
	Init_PWM_timer();
	
	/* conversion mode (free running or single) and prescaler masks 
	 * ACDSRA: start ADC, clock prescaler, interrupt, trigger enable, start single conv */
	ADC_Init( ADC_SINGLE_CONVERSION | ADC_PRESCALE_128);	
	/* Must setup the ADC channel to read beforehand */
	ADC_SetupChannel(3);
	ADC_SetupChannel(4);
	
	/* Start the ADC conversion in free running mode 
	 * ADMUX and Vref, adjust, sel mux channel with gain
	 * 
	*/
	//ADC_StartReading(ADC_REFERENCE_INT2560MV | ADC_RIGHT_ADJUSTED | ADC_1100MV_BANDGAP);
	
	//Temperature_Init();
	//Dataflash_Init();
	SerialStream_Init(9600, false);

}


inline static void Init_PWM_timer( void )
{
	//PORTC = 0;
	//DDRC = 0xFF;
	
	ICR3 = 50;	//50 -> 10kHz @8MHz clk (clkdiv8)
	OCR3A = 15;	//duty (of ICR3)
	//OCR3B = 15;
	//OCR3C = 15;
	//stop timer 3	
	TCCR3B =  (0<<CS32) | (0<<CS31) | (0<<CS30); // no clock source = timer stoped
	//non Interrupted
	TIMSK3 = 0;
	// Synchronize timer
	TCNT3 = 0;
		   
  //Timer Counter 3. OCRA and OCRB used for motor
  TCCR3A = (1<<COM3A1)|(0<<COM3A0)|        // Clear OCRA on compare match
           (1<<COM3B1)|(0<<COM3B0)|        // Clear OCRB on compare match
           (1<<COM3C1)|(0<<COM3C0)|        // Clear OCRB on compare match
           (0<<WGM31)|(0<<WGM30);         //  PWM mode 8
  TCCR3B = (0<<ICNC3)|(0<<ICES3)|
           (0<<WGM32)|(1<<WGM33)|         //  PWM mode 8
           //(0<<CS32)|(1<<CS31)|(0<<CS30); // Prescaler = CLK/8
		   (0<<CS12)|(0<<CS11)|(1<<CS10); // Prescaler = CLK/1
	//TCCR3C = ..; //for non-PWM mode

  //TIFR0 = TIFR0;    // Clear TC0 interrupt flags
  //TIFR1 = TIFR1;    // Clear TC2 interrupt flags
}
