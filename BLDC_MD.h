//3x half-bridge driver is controled by 3 pairs: U, V, W ports.
//Low are on/off W1Hile High atre PWM capable/

#ifndef __BLDC_MD_
#define __BLDC_MD_

	#include <avr/io.h>
	#include <avr/wdt.h>
	#include <avr/power.h>
	#include <avr/interrupt.h>
	#include <stdio.h>	
	#include <util/delay.h> //m.d. _delay_ms(...) function
	#include <avr/pgmspace.h>


	#include <LUFA/Drivers/Board/LEDs.h>

			/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
	#define LEDMASK_USB_NOTREADY      LEDS_LED1
	#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)
	#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)
	#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)
	#define LEDMASK_USB_BUSY          LEDS_LED2
	#define LEDMASK_USB_EASY          LEDS_LED4
	#define LEDMASK_USB_UCK          LEDS_LED3
	#define LEDMASK_USB_RST          (LEDS_LED1 | LEDS_LED2 | LEDS_LED3 | LEDS_LED4)

		
	//! \name Output pins to drive motor 1 "M1" - port B i C
	#define U1L  PB7
	#define V1L  PB6
	#define W1L  PB5
	#define U1H  PB4
	#define V1H  PC1
	#define W1H  PC0
	#define M1_BMASK   ( (1<<U1L)|(1<<V1L)|(1<<W1L)|(1<<U1H) )
	#define M1_CMASK   ( (1<<V1H)|(1<<W1H) )

	//! \name Output pins to drive motor 2, "M2" - port C
	#define U2L  PC6
	#define V2L  PC5
	#define W2L  PC4
	#define U2H  PC7
	#define V2H  PC3
	#define W2H  PC2
	#define M2_CMASK   ((1<<U2L)|(1<<V2L)|(1<<W2L)|(1<<U2H)|(1<<V2H)|(1<<W2H))

	//! \name Hall sensor input pins.
	#define HALL1 PB1
	#define HALL2 PB2
	#define HALL3 PB3
	#define HALL_BMASK  ((1<<HALL1)|(1<<HALL2)|(1<<HALL3))

	//! \name Port control registers for Hall sensor inputs.
	#define PORT_HALL PORTB
	#define DDR_HALL  DDRB
	#define PIN_HALL  PINB



	//! \name Port control registers for drive stages.
	//@{
	#define PORT_MC PORTD
	#define DDR_MC  DDRD
	#define PIN_MC  PIND
	//@}




	#define PATTERN_DRV_OFFSET  0 //!< Offset for drive pattern in table.
	#define PATTERN_COM0_OFFSET 6 //!< Offset for Timer 0 Output Compare config pattern in table.
	#define PATTERN_COM2_OFFSET 12 //!< Offset for Timer 2 Output Compare config pattern in table.



	/*! \name CCW driving configuration
	*
	*  IO driving configuration (driving enabled by setting
	*  IO to output)for counter clockwise rotation.
	*/

	//M1s5 - Motor1 state5, bit values under M1_BMASK and M1_CMASK
	//Bits have to be separated in code.
	///0B100010
	#define M1s1_CCW  ((1<<U1L)|(0<<V1L)|(0<<W1L)|(0<<U1H)  |  (1<<V1H)|(0<<W1H))
	///0B100001
	#define M1s2_CCW  ((1<<U1L)|(0<<V1L)|(0<<W1L)|(0<<U1H)  |  (0<<V1H)|(1<<W1H))
	///0B010001
	#define M1s3_CCW  ((0<<U1L)|(1<<V1L)|(0<<W1L)|(0<<U1H)  |  (0<<V1H)|(1<<W1H))
	///0B010100
	#define M1s4_CCW  ((0<<U1L)|(1<<V1L)|(0<<W1L)|(1<<U1H)  |  (0<<V1H)|(0<<W1H))
	///0B001100
	#define M1s5_CCW  ((0<<U1L)|(0<<V1L)|(1<<W1L)|(1<<U1H)  |  (0<<V1H)|(0<<W1H))
	///0B001010
	#define M1s6_CCW  ((0<<U1L)|(0<<V1L)|(1<<W1L)|(0<<U1H)  |  (1<<V1H)|(0<<W1H))

	//M2s3 - Motor 2, state 3...
	//Bits directly loaded into PortC with mask M2_MASK
	///0B100010
	#define M2s1_CCW  ((1<<U2L)|(0<<V2L)|(0<<W2L)|(0<<U2H)|(1<<V2H)|(0<<W2H))
	///0B100001
	#define M2s2_CCW  ((1<<U2L)|(0<<V2L)|(0<<W2L)|(0<<U2H)|(0<<V2H)|(1<<W2H))
	///0B010001
	#define M2s3_CCW  ((0<<U2L)|(1<<V2L)|(0<<W2L)|(0<<U2H)|(0<<V2H)|(1<<W2H))
	///0B010100
	#define M2s4_CCW  ((0<<U2L)|(1<<V2L)|(0<<W2L)|(1<<U2H)|(0<<V2H)|(0<<W2H))
	///0B001100
	#define M2s5_CCW  ((0<<U2L)|(0<<V2L)|(1<<W2L)|(1<<U2H)|(0<<V2H)|(0<<W2H))
	///0B001010
	#define M2s6_CCW  ((0<<U2L)|(0<<V2L)|(1<<W2L)|(0<<U2H)|(1<<V2H)|(0<<W2H))



	/*! \name CW driving configuration
	*
	*  IO driving configuration (driving enabled by setting
	*  IO to output)for clockwise rotation.
	*/
	//@{
	///001010
	#define PDP1_CW  ((0<<U1L)|(0<<V1L)|(1<<W1L)|(0<<U1H)|(1<<V1H)|(0<<W1H))
	///001100
	#define PDP2_CW  ((0<<U1L)|(0<<V1L)|(1<<W1L)|(1<<U1H)|(0<<V1H)|(0<<W1H))
	///010100
	#define PDP3_CW  ((0<<U1L)|(1<<V1L)|(0<<W1L)|(1<<U1H)|(0<<V1H)|(0<<W1H))
	///010001
	#define PDP4_CW  ((0<<U1L)|(1<<V1L)|(0<<W1L)|(0<<U1H)|(0<<V1H)|(1<<W1H))
	///100001
	#define PDP5_CW  ((1<<U1L)|(0<<V1L)|(0<<W1L)|(0<<U1H)|(0<<V1H)|(1<<W1H))
	///100010
	#define PDP6_CW  ((1<<U1L)|(0<<V1L)|(0<<W1L)|(0<<U1H)|(1<<V1H)|(0<<W1H))
	//@}

	#define OVERCURRENT_PIN PC1   //!< Over-current LED signalling (active low).

	//PWM output configuration CCW (for both Timer1 and Timer3)

	#define U1L_ON   ((1<<COM1C1)|(0<<COM1C0))
	#define U1L_OFF  ((0<<COM1C1)|(0<<COM1C0))
	#define V1L_ON   ((1<<COM1B1)|(0<<COM1B0))
	#define V1L_OFF  ((0<<COM1B1)|(0<<COM1B0))
	#define W1L_ON   ((1<<COM1A1)|(0<<COM1A0))
	#define W1L_OFF  ((0<<COM1A1)|(0<<COM1A0))
	#define WGMcoef ( (0<<WGM01)|(0<<WGM00))

	#define TC1s1_CCW  ((U1L_ON)|(V1L_OFF)|(W1L_OFF) | (WGMcoef))
	#define TC1s2_CCW  ((U1L_ON)|(V1L_OFF)|(W1L_OFF) | (WGMcoef))
	#define TC1s3_CCW  ((U1L_OFF)|(V1L_ON)|(W1L_OFF) | (WGMcoef))
	#define TC1s4_CCW  ((U1L_OFF)|(V1L_ON)|(W1L_OFF) | (WGMcoef))
	#define TC1s5_CCW  ((U1L_OFF)|(V1L_OFF)|(W1L_ON) | (WGMcoef))
	#define TC1s6_CCW  ((U1L_OFF)|(V1L_OFF)|(W1L_ON) | (WGMcoef))

	//Timer3 states are the same as Timer1
	//WCALE NIE!!!!!!!!!!!!!
	#define TC3s1_CCW  TC1s1_CCW
	#define TC3s2_CCW  TC1s2_CCW
	#define TC3s3_CCW  TC1s3_CCW
	#define TC3s4_CCW  TC1s4_CCW
	#define TC3s5_CCW	TC1s5_CCW
	#define TC3s6_CCW	TC1s6_CCW


	//PWM output configuration CW (for both Timer1 and Timer3)

	#define TC1s1_CW  ((U1L_OFF)|(V1L_OFF)|(W1L_ON) |(WGMcoef))
	#define TC1s2_CW  ((U1L_OFF)|(V1L_OFF)|(W1L_ON) | (WGMcoef))
	#define TC1s3_CW  ((U1L_OFF)|(V1L_ON)|(W1L_OFF) | (WGMcoef))
	#define TC1s4_CW  ((U1L_OFF)|(V1L_ON)|(W1L_OFF) | (WGMcoef))
	#define TC1s5_CW  ((U1L_ON)|(V1L_OFF)|(W1L_OFF) | (WGMcoef))
	#define TC1s6_CW  ((U1L_ON)|(V1L_OFF)|(W1L_OFF) | (WGMcoef))

	//Timer3 states are the same as Timer1
	#define TC3s1_CW 	TC1s1_CW
	#define TC3s2_CW 	TC1s2_CW
	#define TC3s3_CW 	TC1s3_CW
	#define TC3s4_CW 	TC1s4_CW
	#define TC3s5_CW	TC1s5_CW
	#define TC3s6_CW	TC1s6_CW






	#define HP1 ((1<<HALL1)|(0<<HALL2)|(1<<HALL3))
	#define HP2 ((1<<HALL1)|(0<<HALL2)|(0<<HALL3))
	#define HP3 ((1<<HALL1)|(1<<HALL2)|(0<<HALL3))
	#define HP4 ((0<<HALL1)|(1<<HALL2)|(0<<HALL3))
	#define HP5 ((0<<HALL1)|(1<<HALL2)|(1<<HALL3))
	#define HP6 ((0<<HALL1)|(0<<HALL2)|(1<<HALL3))
	//@}


	// Direction control
	#define CLOCKWISE         1 //!< Used by Set_Direction() for CW rotation.
	#define COUNTERCLOCKWISE  0 //!< Used by Set_Direction() for CCW rotation.



	//! \name ADC mU1Ltiplexing
	//@{
	#define ADC_MUX_SPEED_REF   ((0 << REFS1) | (0 << REFS0) | (1 << ADLAR))
	#define ADC_MUX_SHUNT_H     ((0 << REFS1) | (0 << REFS0) | (1 << ADLAR) | (1 << MUX2))
	#define ADC_MUX_SHUNT_L     ((0 << REFS1) | (0 << REFS0) | (1 << ADLAR) | (1 << MUX2) | (1 << MUX0))
	//@}


	// Current limiting
	#define MAX_CURRENT                     1.0f   //!< Max motor current in Amperes.
	#define CURRENT_SHUNT_RESISTANCE        0.22f  //!< Current shunt resistanse in Ohms.
	#define ADC_REF_VOLTAGE                 5.0f   //!< ADC reference voltage in Volts.

	#define VOLTS_PER_ADC_STEP              (ADC_REF_VOLTAGE / 256)  //!< Volts per LSB with 8-bit ADC.
	#define MAX_SHUNT_VOLTAGE               (MAX_CURRENT * CURRENT_SHUNT_RESISTANCE)  //!< Max current shunt voltage.
	#define MAX_CURRENT_ADC                 (signed int)(MAX_SHUNT_VOLTAGE / VOLTS_PER_ADC_STEP) //!< Calc max value for raw ADC reading.

	/* Function Prototypes: */
	void Init_MC_timers( void );
	void Init_MC_pin_change_interrupt( void );
	static void M1Set_Speed(unsigned char speed);
	static void M2Set_Speed(unsigned char speed);
	static void Set_Direction(unsigned char direction);

#endif