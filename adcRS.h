

/** \file
 *
 *  Header file for TempDataLogger.c.
 */

#ifndef _TEMP_mototrainer_H_
#define _TEMP_mototrainer_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <avr/pgmspace.h>
		#include <stdio.h>	
		#include <stdlib.h>	
		#include <util/delay.h> //m.d. _delay_ms(...) function
		

		#include <LUFA/Version.h>
		#include <LUFA/Drivers/Board/LEDs.h>
		#include <LUFA/Drivers/Board/Buttons.h>
		#include <LUFA/Drivers/Board/Joystick.h>
		
		#include <LUFA/Drivers/Peripheral/SerialStream.h>
		#include <LUFA/Drivers/Peripheral/ADC.h>
	#include "CDCSerial.c"

		
	/* Macros: */
		/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      LEDS_LED1

		/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

		/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

		/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

		/** LED mask for the library LED driver, to indicate that the USB interface is busy. */
		#define LEDMASK_USB_BUSY          LEDS_LED2
		
		/** Filename for the log data when written to the dataflash FAT filesystem. */
		#define LOG_FILENAME             "TEMPLOG.txt"
		
		/** Data log interval between samples, in tens of milliseconds */
		#define LOG_INTERVAL_10MS        1000
		
				
		
	/* Type Defines: */
		typedef struct
		{
			uint8_t Day;
			uint8_t Month;
			uint8_t Year;

			uint8_t Hour;
			uint8_t Minute;
			uint8_t Second;
			
			uint8_t LogInterval500MS;
		} Device_Report_t;

	/* Function Prototypes: */

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_UnhandledControlRequest(void);
		

		void Examp_4(void);
		void PWM_cfg(void);

		inline static void SetupHardware(void);
		inline static void Init_PWM_timer( void );
	

#endif
