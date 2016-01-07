#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "usb.h"
#include "led.h"


uint8_t buf[64];
uint8_t ptr=0;


void Main_Task(void)
{

		if(status)
		{

		}
	
} 


void init_ports(void)
{
	led_off;
	led_init;
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	init_ports();
	init_usb();
}
