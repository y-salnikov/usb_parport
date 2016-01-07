#include <avr/io.h>

void parport_init(void)
{
	DDRD=0xff;		//data
	DDRC|=0xf0;		//control
	DDRB=0x00;		//status
	PORTB=0xFF;
}

void parport_write_reg(uint8_t reg, uint8_t d)
{
	switch(reg)
	{
		case 0: // data
			PORTD=d;
		break;
		
		case 1: // status
		break;

		case 2: // control
			PORTC=((PORTC & 0x0F) | (d<<4));
		break;
	}
}

uint8_t parport_read_reg(uint8_t reg)
{
	switch(reg)
	{
		case 0: // data
			return PORTD;
		break;
		
		case 1: // status
			return PINB & 0xF8;
		break;

		case 2: // control
			return (PINC >>4);
		break;
	}
}
