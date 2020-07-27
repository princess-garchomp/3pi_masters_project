/*
schematic note:

	all boards have internal pull up resistors, so to trigger them they are looking for ground
		transistors are powered by output pins to allow input puns to see ground

	out PD7 powers RC car MR_IN
	out PD1 powers RC car MF_IN

	RC car MR_out powers in PB4
	RC car MF_out powers in PB5

3pi notes:
	
	PD7 is linked to green surface mount led
	PD1 is linekd to red surface mount led

	PB4 is linked to button B
	PB5 is lined to button C
*/


#include <avr/io.h>
#include <stdbool.h>
#include "radio_IO.h"


#define SEND_SIGNAL_1_SHIFT (PD7)
#define SEND_SIGNAL_2_SHIFT (PD1)

#define  RECEIVE_SIGNAL_CHECK_1_SHIFT (PB4)
#define  RECEIVE_SIGNAL_CHECK_2_SHIFT (PB5)

static void config_send_signal();
static void config_receive_signal();

static void turn_on_send_signal_1();
static void turn_off_send_signal_1();

static void turn_on_send_signal_2();
static void turn_off_send_signal_2();



void config_radio_io()
{
	config_send_signal();
	config_receive_signal();
	set_signal_output(SEND_SIGNAL_NONE);
	//set_signal_check_input(RECEIVE_SIGNAL_CHECK_NONE);
}

void set_signal_output(send_signal out)
{
	switch(out)
	{
		case(SEND_SIGNAL_NONE):
			turn_off_send_signal_1();
			turn_off_send_signal_2();
			break;
		case(SEND_SIGNAL_1):
			turn_on_send_signal_1();
			turn_off_send_signal_2();
			break;
		case(SEND_SIGNAL_2):
			turn_off_send_signal_1();
			turn_on_send_signal_2();
			break;
		case(SEND_BOTH):
			turn_on_send_signal_1();
			turn_on_send_signal_2();
			break;
	}
}
/*void set_signal_check_input(receive_signal_check in)
{
	switch(in)
	{
		case(RECEIVE_SIGNAL_CHECK_NONE):
		
	}
}

*/













static void config_send_signal()
{
	DDRD |= (1ul<<SEND_SIGNAL_1_SHIFT) | (1ul<<SEND_SIGNAL_2_SHIFT);
	set_signal_output(SEND_SIGNAL_NONE);
}
static void config_receive_signal()
{
	DDRB &= ~(1ul<<RECEIVE_SIGNAL_CHECK_1_SHIFT) & ~(1ul<<RECEIVE_SIGNAL_CHECK_2_SHIFT);
	PORTB |= (1ul<<RECEIVE_SIGNAL_CHECK_1_SHIFT) | (1ul<<RECEIVE_SIGNAL_CHECK_2_SHIFT); 
}

static void turn_on_send_signal_1()
{
	PORTD |= (1ul<<SEND_SIGNAL_1_SHIFT);
}
static void turn_off_send_signal_1()
{
	PORTD &= ~(1ul<<SEND_SIGNAL_1_SHIFT);
}

static void turn_on_send_signal_2()
{
	PORTD |= (1ul<<SEND_SIGNAL_2_SHIFT);
}
static void turn_off_send_signal_2()
{
	PORTD &= ~(1ul<<SEND_SIGNAL_2_SHIFT);
}




_Bool check_receive_signal_1_true()
{
	return!(PINB & (1ul<<RECEIVE_SIGNAL_CHECK_1_SHIFT));
}
_Bool check_receive_signal_1_false()
{
	return!(check_receive_signal_1_true());
}

_Bool check_receive_singal_2_true()
{
	return!(PINB & (1ul<<RECEIVE_SIGNAL_CHECK_2_SHIFT));
}
_Bool check_receive_signal_2_false()
{
	return!(check_receive_singal_2_true());
}