#include <avr/io.h>
#include <stdbool.h>

//A--PB1, B--PB4, C--PB5

#define SW_A_SHIFT (PB1)

_Bool push_button_A_is_pressed()
{
	return!(PINB & (1ul<<SW_A_SHIFT));
}
_Bool push_button_A_is_not_pressed()
{
	return!(push_button_A_is_pressed());
}
void wait_push_button_A_press_release()
{
	while(push_button_A_is_not_pressed());
	while(push_button_A_is_pressed());
}

void config_push_button_A()
{
	DDRB &= ~(1ul<<SW_A_SHIFT);
	PORTB |= (1ul<<SW_A_SHIFT);
	
	//DDRB 0, PORTB 1: input, pullup en, pin sources current when externally pulled low
}