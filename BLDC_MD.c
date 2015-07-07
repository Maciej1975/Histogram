#include "BLDC_MD.h"


const unsigned char M1drvPatternsCCW[] PROGMEM = {
//const unsigned char M1drvPatternsCCW[]  = {
//const - value cannot be modified in code, checed at compile time
//PROGREM - data stored in PgmFlash
  0,    //Stop - all pins low
  0,	//TCCRxA=0 OC-pins disconnected
  //0,
	//5-1-3-2-6-4 -
	//4-6-2-3-1-5 -CCW Hall sensor sequence

  M1s5_CCW, //Phase1
  TC1s5_CCW, //Phase1 Timer1
  //TC3s5_CCW, //Phase1 Timer3   
    
  M1s3_CCW, //Phase3
  TC1s3_CCW, //Phase3 Timer1
  //TC3s3_CCW, //Phase3 Timer3   
       
  M1s4_CCW, //Phase6
  TC1s4_CCW, //Phase6 Timer1
  //TC3s4_CCW, //Phase6 Timer3 
  	
  M1s1_CCW, //Phase2 (port B: U1l, V1l, W1L, U1H), 0, 0, (Port C: V1H, W1H) 
  TC1s1_CCW, //Phase2 Timer1 PWM (U1L, V1L, W1L)
  //TC3s1_CCW, //Phase2 Timer3 PWM (U2L, V2L, W2L)
 
  M1s6_CCW, //Phase6	-	pinout portB and C
  TC1s6_CCW, //Phase6 Timer1
  //TC3s6_CCW,  //Phase6 Timer3  
 
  M1s2_CCW, //Phase4
  TC1s2_CCW, //Phase4 Timer1 (U1L, V1L, W1L)
  //TC3s2_CCW, //Phase4 Timer3 (U2L, V2L, W2L) 
 
  0,0
  //,0
};


const unsigned char M1drvPatternsCCW_RAM[] = {
//PROGREM - data stored in PgmFlash
  0,    //Stop - all pins low
  0,	//TCCRxA=0 OC-pins disconnected

  M1s5_CCW, //Phase1
  TC1s5_CCW, //Phase1 Timer1
    
  M1s3_CCW, //Phase3
  TC1s3_CCW, //Phase3 Timer1
       
  M1s4_CCW, //Phase6
  TC1s4_CCW, //Phase6 Timer1
  	
  M1s1_CCW, //Phase2 (port B: U1l, V1l, W1L, U1H), 0, 0, (Port C: V1H, W1H) 
  TC1s1_CCW, //Phase2 Timer1 PWM (U1L, V1L, W1L)
 
  M1s6_CCW, //Phase6	-	pinout portB and C
  TC1s6_CCW, //Phase6 Timer1
 
  M1s2_CCW, //Phase4
  TC1s2_CCW, //Phase4 Timer1 (U1L, V1L, W1L)
 
  0,0
};

//const unsigned char M2drvPatternsCCW[] PROGMEM = M1drvPatternsCCW...

const unsigned char M1drvPatternsCW[] PROGMEM = {
	//const unsigned char M1drvPatternsCCW[]  = {
	//const - value cannot be modified in code, checed at compile time
	//PROGREM - data stored in PgmFlash
	0,    //Stop - all pins low
	0,	//TCCRxA=0 OC-pins disconnected
	//0,

	M1s1_CCW, //Phase2 (port B: U1l, V1l, W1L, U1H), 0, 0, (Port C: V1H, W1H) 
	TC1s1_CCW, //Phase2 Timer1 PWM (U1L, V1L, W1L)
	//TC3s1_CCW, //Phase2 Timer3 PWM (U2L, V2L, W2L)  

	M1s2_CCW, //Phase4
	TC1s2_CCW, //Phase4 Timer1 (U1L, V1L, W1L)
	//TC3s2_CCW, //Phase4 Timer3 (U2L, V2L, W2L)  

	M1s3_CCW, //Phase3
	TC1s3_CCW, //Phase3 Timer1
	//TC3s3_CCW, //Phase3 Timer3   

	M1s4_CCW, //Phase6
	TC1s4_CCW, //Phase6 Timer1
	//TC3s4_CCW, //Phase6 Timer3    

	M1s5_CCW, //Phase1
	TC1s5_CCW, //Phase1 Timer1
	//TC3s5_CCW, //Phase1 Timer3  

	M1s6_CCW, //Phase6	-	pinout portB and C
	TC1s6_CCW, //Phase6 Timer1
	//TC3s6_CCW,  //Phase6 Timer3

	0,0
	//,0
};

/*! \brief CCW rotation patterns.
 *
 * Configuration of pin drive levels
 * and timer COM bits in different
 * phases for CounterClockWise rotation.
 */
 

/*! \brief  Pin Change Interrupt Service Routine.
 *
 * Updates the PWM outputs controlling the low side of the driver and
 * the IO controlling the high side of the driver. To ensure a speed
 * optimal interrupt the variables used in the interrupt are placed
 * in reserved registers (locked for this purpose only). Further, the
 * information required to do the commutation is placed in tables that
 * are accessed very efficiently using the Hall sensor input signals
 * as offset.
 */
 

/*
//dla Hall sensor z M2 motorka
ISR( EXTINT_vect, ISR_BLOCK )
{
//dla 2 silnikow nalezy rozpoznac z ktorej grupy przyszlo przerwanie Hall
//najprosciej PCINT dla silnika M1, EXTINT dla silnika M2
	volatile const unsigned char *pTemp;
	unsigned char HState;

	HState = ((PIN_HALL & HALL_BMASK)>>1);  // Read Hall, Mask Pins, shift to use as pointer offset
	//jesli piny PIN_HALL jako pointer to w tablicy musza byc obsluzone zakazane stany 000 i 111
	if ((HState == 0) || (HState == 7)) //--error w tablicy??
	{
		//stop and error
		//red led
		while(1); //critical halt
	}
	else
	{

		pTemp = M1drvPatternsCCW + HState + HState ; //HState - o 3 pola
		//pTemp = pDrvPattern + HState + HState + HState; //HState - o 3 pola
		//  TCCR0A = fastTemp.HByte; //Disable PWM outputs (and thereby close low side FET)
		//  TCCR2A = fastTemp.HByte; //Disable PWM output (and thereby close low side FET)

		//PORTB &= ~(M1_BMASK); //clear demanded bits
		//PORTB |= (( pgm_read_byte(&pTemp[0]) ) & M1_BMASK );    //Change drive levels on high side
		PORTC &= ~(M1_CMASK);
		PORTC |= (( pgm_read_byte(&pTemp[0]) ) & M1_CMASK );    //Change drive levels on high side
		
		TCCR3A = pgm_read_byte(&pTemp[1]);    // Reconfigure output compare operation for T1 (U1L, V1L, W1L)
	}
}

*/
void Init_MC_pin_change_interrupt( void )
{
	//PORTB &= ~( (1<<U1L) | (1<<V1L) | (1<<W1L) | (1<<U1H) );
	//DDRB |= ( (1<<U1L) | (1<<V1L) | (1<<W1L) | (1<<U1H) ); //outputs PWM
	PORT_HALL |= (HALL_BMASK); //pull-up
	DDR_HALL &= ~(HALL_BMASK); //input
  //PinChange und ExtInt....
  PCMSK0 = (1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3); //Enable pin change interrupt on PB1:3
  PCICR = 1<<PCIE0;    // Enable pin change interrupt0 (PORTB)
}


//!
/*! \brief  Initialize motor control timers.
 *
 * Initialize the Timer 1 and timer 2 to run in phase and frequency correct
 * PWM mode (symmetric PWM). The base frequency is set to 32kHz (can be
 * reduced at the expense of lower resolution on the speed control). The
 * functions also ensures that the timers are counting in synch.
 *
 *  \param void
 *
 *  \return void
 */
 
void Init_MC_timers( void )
{
	PORTB &= ~( (1<<U1L) | (1<<V1L) | (1<<W1L) | (1<<U1H) );
	DDRB |= ( (1<<U1L) | (1<<V1L) | (1<<W1L) | (1<<U1H) ); //outputs PWM
	PORTC = 0;
	DDRC = 0xFF;
	
	//some initial PWM duty cycle, i.e. speed or torque
	//M1 and M2 stopped
	ICR1 = 50;	//50 -> 10kHz @8MHz clk (clkdiv8)
	OCR1A = 10;
	OCR1B = 10;
	OCR1C = 10;
	
	ICR3 = 200;
	OCR3A = 15;
	OCR3B = 15;
	OCR3C = 15;
	//stop timer 1 and 3
	TCCR1B =    (0<<CS12)|(0<<CS11)|(0<<CS10); // no clock source = timer stoped	
	TCCR3B =  (0<<CS32)|(0<<CS31)|(0<<CS30); // no clock source = timer stoped
	//non Interrupted
	TIMSK1 = 0;
	TIMSK3 = 0;
	// Synchronize timers
	TCNT1 = 0;
	TCNT3 = 3;	//because 3 clock later, no practical meaning because different motors
  
  //Timer Counter 1. OCR1A, OCR1B, OCR1C used for motor
  TCCR1A = (1<<COM1A1)|(0<<COM1A0)|        // Clear OCRA on compare match
           (1<<COM1B1)|(0<<COM1B0)|        // Clear OCRB on compare match
           (1<<COM1C1)|(0<<COM1C0)|        // Clear OCRB on compare match
           (0<<WGM11)|(0<<WGM10);         // PWM mode 8
  TCCR1B = (0<<ICNC1)|(0<<ICES1)|
           (0<<WGM12)| (1<<WGM13)|                    // PWM mode 8
           (0<<CS12)|(1<<CS11)|(0<<CS10); // Prescaler = CLK/8
           //(0<<CS12)|(0<<CS11)|(1<<CS10); // Prescaler = CLK/1
	//TCCR1C = ..; //for non-PWM mode

		   
  //Timer Counter 3. OCRA and OCRB used for motor
  TCCR3A = (1<<COM3A1)|(0<<COM3A0)|        // OCRA not connected
           (1<<COM3B1)|(0<<COM3B0)|        // Clear OCRB on compare match
           (1<<COM3C1)|(0<<COM3C0)|        // Clear OCRB on compare match
           (0<<WGM31)|(0<<WGM30);         //  PWM mode 8
  TCCR3B = (0<<ICNC3)|(0<<ICES3)|
           (0<<WGM32)|(1<<WGM33)|         //  PWM mode 8
           (0<<CS32)|(1<<CS31)|(0<<CS30); // Prescaler = CLK/8


  //TIFR0 = TIFR0;    // Clear TC0 interrupt flags
  //TIFR1 = TIFR1;    // Clear TC2 interrupt flags
}


/*! \brief Set motor speed.
 *
 * Updates the output compare registers of the timer 0 and timer 2 which
 * control the duty cycle of the PWM output and thereby the speed of the
 * motor. The method used ensures that that all PWM channels are behaving
 * same duty cycle.
 *
 *  \param speed Compare match value that defines PWM duty cycle.
 *
 *  \return void
 */
static void M1Set_Speed(unsigned char speed)
{
  TIFR1 = TIFR1;    // Clear TC1 interrupt flags
  while( !(TIFR1 & (1<<TOV1)));  // Wait for TOV to ensure that all registers are
                            //  updated in the same timer cycle
							//mode 8, TOV1/TOV3 set at BOTTOM cnt
  cli(); //__disable_interrupt(); //qatomic transaction
  OCR1A = speed;        // Change the duty cycle
  OCR1B = speed;
  OCR1C = speed;
  sei(); //__enable_interrupt();
}

static void M2Set_Speed(unsigned char speed)
{
  TIFR3 = TIFR3;    // Clear TC1 interrupt flags
  while( !(TIFR3 & (1<<TOV1)));  // Wait for TOV to ensure that all registers are
                            //  updated in the same timer cycle
							//mode 8, TOV1/TOV3 set at BOTTOM cnt
  cli(); //__disable_interrupt(); //qatomic transaction
  OCR3A = speed;        // Change the duty cycle
  OCR3B = speed;
  OCR3C = speed;
  sei(); //__enable_interrupt();
}



/*! \brief Set motor direction, CW og CCW.
 *
 * Set the commutation table pointer up to point at either the clockwise
 * or counter clockwise direction table. Note that it is not recommended
 * to change direction without first reducing the speed of the motor,
 * preferably stopping it fully.
 *
 *  \param direction Direction is given as Clockwise or Counter Clockwise.
 *
 *  \return void
 */
static void Set_Direction(unsigned char direction)
{
/*
  if(direction == CLOCKWISE)
  {
    __disable_interrupt();        //Variable also used in interrupt and access most be protected
    pDrvPattern = drvPatternsCW;   // Set dir to CW, by pointing to CW pattern
    __enable_interrupt();
  }
  else
  {
    __disable_interrupt();        //Variable also used in interrupt and access most be protected
    pDrvPattern = drvPatternsCCW;   // Set dir to CCW, by pointing to CCW pattern
    __enable_interrupt();
  }*/
}

