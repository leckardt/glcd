
#include <avr/io.h>
#include <util/delay.h>
int main()
{
	DDRD |= (1<<PD6);
	while(1)
	{
		_delay_ms(100);
		PORTD |= (1<<PD6);	
		_delay_ms(100);
		PORTD &= ~(1<<PD6);
	}

}
