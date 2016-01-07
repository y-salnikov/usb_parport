#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <LUFA/Drivers/USB/USB.h>
#include "led.h"
#include <stdint.h>
#include "parport.h"

extern void SetupHardware(void);
extern void Main_Task(void);



/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
		
}

void EVENT_USB_Device_ControlRequest(void)
{
	uint16_t data;
	if(((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_TYPE) ==          REQTYPE_VENDOR)
		&& ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_RECIPIENT) == REQREC_DEVICE))
	{
		if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_DIRECTION)  == REQDIR_HOSTTODEVICE)
		{
			switch(USB_ControlRequest.bRequest)
			{
				case 0xE0:
					/* marks the command as "accepted" by the application, so that LUFA does not process it: */
					Endpoint_ClearSETUP();
					/* mark the whole request as successful: */
					Endpoint_ClearStatusStage();
					/* process command parameters: */
					parport_init();
					led_on;
				break;
				case 0xE1:
					Endpoint_ClearSETUP();
					Endpoint_ClearStatusStage();
					parport_write_reg(USB_ControlRequest.wIndex,USB_ControlRequest.wValue);
				break;
				case 0xE2:
					Endpoint_ClearSETUP();
					Endpoint_ClearStatusStage();
					if(USB_ControlRequest.wValue) led_on;
					else led_off;
				break;
			}
		}
		else
		{
			switch(USB_ControlRequest.bRequest)
			{
				case 0xE1:
					data=parport_read_reg(USB_ControlRequest.wIndex);
					Endpoint_ClearSETUP();
					/* write data to endpoint */
					Endpoint_Write_Word_LE(data);
					/* send packet */
					Endpoint_ClearIN();
					/* and mark the whole request as successful: */
					Endpoint_ClearStatusStage();
				break;
			}
		}
	}
}




void init_usb(void)
{
		/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	
	USB_Init();
	
}

int main(void)
{
	SetupHardware();
	sei();

	for (;;)
	{
		Main_Task(); 
		USB_USBTask();
	}
}

