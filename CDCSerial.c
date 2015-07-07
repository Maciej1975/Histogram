/** \file
 *
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "CDCSerial.h"

char*       UARTString    = NULL;

/** Contains the current baud rate and other settings of the virtual serial port. While this demo does not use
 *  the physical USART and thus does not use these settings, they must still be retained and returned to the host
 *  upon request or the host will assume the device is non-functional.
 *
 *  These values are set by the host via a class-specific request, however they are not required to be used accurately.
 *  It is possible to completely ignore these value or use other settings as the host is completely unaware of the physical
 *  serial link characteristics and instead sends and receives data in endpoint streams.
 */
CDC_Line_Coding_t LineEncoding = { .BaudRateBPS = 0,
                                   .CharFormat  = OneStopBit,
                                   .ParityType  = Parity_None,
                                   .DataBits    = 8            };

void SendStringUART(char* Stri) {
	if (USB_DeviceState != DEVICE_STATE_Configured)
		  return;
	if ( LineEncoding.BaudRateBPS ){
		Endpoint_SelectEndpoint(CDC_TX_EPNUM);
		Endpoint_Write_Stream_LE(Stri, strlen(Stri)); //strlen not including the terminating '\0'
		bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);
		Endpoint_ClearIN();
		if (IsFull)	{
			Endpoint_WaitUntilReady();
			Endpoint_ClearIN();
		}
	}	
	Endpoint_SelectEndpoint(CDC_RX_EPNUM);
	
	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived())
	  Endpoint_ClearOUT();
}

bool RecvStringUART(char* Stri) {
	if (USB_DeviceState != DEVICE_STATE_Configured)
		  return false;
	Endpoint_SelectEndpoint(CDC_RX_EPNUM);
	
	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived()){
		LEDs_SetAllLEDs(LEDS_LED1);
		//Endpoint_Read_Stream_LE(Stri, 16);
		Stri[0]=Endpoint_Read_Byte();
		Stri[1]=Endpoint_Read_Byte();
		Stri[2]=Endpoint_Read_Byte();
		Stri[3]=Endpoint_Read_Byte();
		Stri[4]=Endpoint_Read_Byte();
		Stri[5]=Endpoint_Read_Byte();
		Stri[6]='\r';
		Stri[7]='\n';
		Stri[7]='\0';
		Endpoint_ClearOUT();
		return true;
	}
	return false;
}



/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED3);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and CDC management tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Indicate USB connected and ready */
	LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);

	/* Setup CDC Notification, Rx and Tx Endpoints */
	if (!(Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPNUM, EP_TYPE_INTERRUPT,
		                             ENDPOINT_DIR_IN, CDC_NOTIFICATION_EPSIZE,
	                                 ENDPOINT_BANK_SINGLE)))
	{
		LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
	}
	
	if (!(Endpoint_ConfigureEndpoint(CDC_TX_EPNUM, EP_TYPE_BULK,
		                             ENDPOINT_DIR_IN, CDC_TXRX_EPSIZE,
	                                 ENDPOINT_BANK_SINGLE)))
	{
		LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
	}
	
	if (!(Endpoint_ConfigureEndpoint(CDC_RX_EPNUM, EP_TYPE_BULK,
		                             ENDPOINT_DIR_OUT, CDC_TXRX_EPSIZE,
	                                 ENDPOINT_BANK_SINGLE)))
	{
		LEDs_SetAllLEDs(LEDS_LED1 | LEDS_LED3);
	}
	
	/* Reset line encoding baud rate so that the host knows to send new values */
	LineEncoding.BaudRateBPS = 0;
}

/** Event handler for the USB_UnhandledControlRequest event. This is used to catch standard and class specific
 *  control requests that are not handled internally by the USB library (including the CDC control commands,
 *  which are all issued via the control endpoint), so that they can be handled appropriately for the application.
 */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{	
				/* Acknowledge the SETUP packet, ready for data transfer */
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_Line_Coding_t));
				
				/* Finalize the stream transfer to send the last packet or clear the host abort */
				Endpoint_ClearOUT();
			}
			
			break;
		case REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				/* Acknowledge the SETUP packet, ready for data transfer */
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_Line_Coding_t));

				/* Finalize the stream transfer to clear the last packet from the host */
				Endpoint_ClearIN();
			}
	
			break;
		case REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				/* Acknowledge the SETUP packet, ready for data transfer */
				Endpoint_ClearSETUP();
				
				/* NOTE: Here you can read in the line state mask from the host, to get the current state of the output handshake
				         lines. The mask is read in from the wValue parameter in USB_ControlRequest, and can be masked against the
						 CONTROL_LINE_OUT_* masks to determine the RTS and DTR line states using the following code:
				*/
				
				Endpoint_ClearStatusStage();
			}
	
			break;
	}
}

/** Function to manage CDC data transmission and reception to and from the host. */
void CDC_Task(void)
{
	char*       ReportString    = NULL;
	uint8_t     JoyStatus_LCL   = Joystick_GetStatus();
	static bool ActionSent      = false;
	
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Determine if a joystick action has occurred */
	if (JoyStatus_LCL & JOY_UP)
	  ReportString = "Joystick Up\r\n";
	else if (JoyStatus_LCL & JOY_DOWN)
	  ReportString = "Joystick Down\r\n";
	else if (JoyStatus_LCL & JOY_LEFT)
	  ReportString = "Joystick Left\r\n";
	else if (JoyStatus_LCL & JOY_RIGHT)
	  ReportString = "Joystick Right\r\n";
	else if (JoyStatus_LCL & JOY_PRESS)
	  ReportString = "Joystick Pressed\r\n";
	else
	  ActionSent = false;

	/* Flag management - Only allow one string to be sent per action */
	if ((ReportString != NULL) && (ActionSent == false) && LineEncoding.BaudRateBPS)
	{
		ActionSent = true;

		/* Select the Serial Tx Endpoint */
		Endpoint_SelectEndpoint(CDC_TX_EPNUM);

		/* Write the String to the Endpoint */
		Endpoint_Write_Stream_LE(ReportString, strlen(ReportString));
		
		/* Remember if the packet to send completely fills the endpoint */
		bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* If the last packet filled the endpoint, send an empty packet to release the buffer on 
		 * the receiver (otherwise all data will be cached until a non-full packet is received) */
		if (IsFull)
		{
			/* Wait until the endpoint is ready for another packet */
			Endpoint_WaitUntilReady();
			
			/* Send an empty packet to ensure that the host does not buffer data sent to it */
			Endpoint_ClearIN();
		}
	}

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPNUM);
	
	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived())
	  Endpoint_ClearOUT();
}
