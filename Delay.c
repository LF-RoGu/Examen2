#include  "Delay.h"

void delay(uint16_t delay)
{
	volatile uint16_t counter;

	for(counter = delay; counter > 0; counter--)
	{
	}
}
