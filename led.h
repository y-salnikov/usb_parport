#define led_on PORTC&=~0x04
#define led_off PORTC|=0x04
#define led_init DDRC|=0x04


