#include "motor.h"


#include <stdint.h>
#include <avr/io.h>

#define _abs(x) ( ((x)<0)?-(x):(x))

#define LEFT_MOTOR_A (PD6)
#define LEFT_MOTOR_B (PD5)
#define LEFT_MOTOR_TIMER 0

#define RIGHT_MOTOR_A (PB3)
#define RIGHT_MOTOR_B (PD3)
#define RIGHT_MOTOR_TIMER (2)

#define MINIMUM_DUTY_CYCLE 0x0A
//maximum dutu cucle
#define INITIAL_DUTY_CYCLE 0x80
#define INITIAL_STEER_POSITION 0
#define INITIAL_ROBOT_SHIFT_STATE STOP
#define LEFT_DUTY_CYCLE_OFFSET 0
#define RIGHT_DUTY_CYCLE_OFFSET 0

#define COMxA1 7
#define COMxA0 6
#define COMxB1 5
#define COMxB0 4

static uint8_t duty_cycle = INITIAL_DUTY_CYCLE;
static int8_t steering_position = INITIAL_STEER_POSITION;
static uint8_t robot_shift_state = INITIAL_ROBOT_SHIFT_STATE;

static void config_left();
static void left_stop();
static void left_coast();
static void left_forward();
static void left_reverse();
static void left_duty_cycle_set(uint8_t duty_cycle);

static void config_right();
static void right_stop();
static void right_coast();
static void right_forward();
static void right_reverse();
static void right_duty_cycle_set(uint8_t duty_cycle);

static void update_robot_controls();



void config_motors()
{
	config_left();
	config_right();
	update_robot_controls();
	return;
}

void set_duty_cycle(uint8_t value)
{
	if(value < MINIMUM_DUTY_CYCLE)
	duty_cycle = MINIMUM_DUTY_CYCLE;
	else
	duty_cycle = value;
	update_robot_controls();
}

uint8_t get_duty_cycle()
{
	return duty_cycle;
}

_Bool shift_robot(robot_shift new_state)
{
	if(robot_shift_state == new_state || robot_shift_state == STOP || robot_shift_state == COAST || new_state == STOP || new_state == COAST)
	{
		robot_shift_state = new_state;
		update_robot_controls();
		return 1;
	}
	else
	return 0;
}

robot_shift get_robot_shift_state()
{
	return robot_shift_state;
}

void steer (int8_t new_steer_position)
{
	steering_position = new_steer_position;
	update_robot_controls();
}

int8_t get_steering()
{
	return steering_position;
}

static void update_robot_controls()
{
	uint8_t steer = _abs (steering_position);
	int16_t duty_cycle_high = duty_cycle + steer;
	int16_t duty_cycle_low = duty_cycle - steer;
	if (duty_cycle_high > 255)
	{
		duty_cycle_high = 255;
		duty_cycle_low = duty_cycle_high - 2*steer;
	}
	else if (duty_cycle_low < MINIMUM_DUTY_CYCLE)
	{
		duty_cycle_low = MINIMUM_DUTY_CYCLE;
		duty_cycle_high = duty_cycle_low + 2*steer;
	}
	if(steering_position > 0 )
	{
		left_duty_cycle_set(duty_cycle_high);
		right_duty_cycle_set(duty_cycle_low);
	}
	else
	{
		left_duty_cycle_set(duty_cycle_low);
		right_duty_cycle_set(duty_cycle_high);
	}
	switch(robot_shift_state)
	{
		case STOP:
		left_stop();
		right_stop();
		break;
		case COAST:
		left_coast();
		right_coast();
		break;
		case FORWARD:
		left_forward();
		right_forward();
		break;
		case REVERSE:
		left_reverse();
		right_reverse();
		break;
		case CW:
		left_forward();
		right_reverse();
		break;
		case CCW:
		left_reverse();
		right_forward();
		break;
		
	}
	return;
}


//follows is left motor
static void config_left()
{
	left_stop();
	DDRD |= (1UL<<LEFT_MOTOR_A);
	DDRD |= (1ul<<LEFT_MOTOR_B);
	TCCR0A = 0x03;//fast pwm
	TCCR0B = 0x01;//no prescale
}
static void left_stop()
{
	TCCR0A &= ~( 1ul<<COMxA0 | 1ul<<COMxA1 | 1ul<<COMxB0 | 1ul<<COMxB1); // off waveform
	PORTD |= (1ul<<LEFT_MOTOR_A);
	PORTD |= (1ul<<LEFT_MOTOR_B);
}

static void left_coast()
{
	TCCR0A &= ~( 1ul<<COMxA0 | 1ul<<COMxA1 | 1ul<<COMxB0 | 1ul<<COMxB1); // off waveform
	PORTD &= ~(1ul<<LEFT_MOTOR_A);
	PORTD &= ~(1ul<<LEFT_MOTOR_B);
}
static void left_forward()
{
	left_stop();
	TCCR0A |= ( 1<<COMxB0 | 1<<COMxB1);
}
static void left_reverse()
{
	left_stop();
	TCCR0A |= ( 1<<COMxA0 | 1<<COMxA1);
}
static void left_duty_cycle_set(uint8_t duty_cycle)
{
	OCR0A = duty_cycle + LEFT_DUTY_CYCLE_OFFSET;
	OCR0B = duty_cycle + LEFT_DUTY_CYCLE_OFFSET;
}

//right
static void config_right()
{
	right_stop();
	DDRB |= (1UL<<RIGHT_MOTOR_A);
	DDRD |= (1ul<<RIGHT_MOTOR_B);
	TCCR2A = 0x03;//fast pwm
	TCCR2B = 0x01;//no prescale
}
static void right_stop()
{
	TCCR2A &= ~( 1ul<<COMxA0 | 1ul<<COMxA1 | 1ul<<COMxB0 | 1ul<<COMxB1); // off waveform
	PORTB |= (1ul<<RIGHT_MOTOR_A);
	PORTD |= (1ul<<RIGHT_MOTOR_B);
}

static void right_coast()
{
	TCCR0A &= ~( 1ul<<COMxA0 | 1ul<<COMxA1 | 1ul<<COMxB0 | 1ul<<COMxB1); // off waveform
	PORTB &= ~(1ul<<RIGHT_MOTOR_A);
	PORTD &= ~(1ul<<RIGHT_MOTOR_B);
}
static void right_forward()
{
	right_stop();
	TCCR2A |= ( 1<<COMxB0 | 1<<COMxB1);
}
static void right_reverse()
{
	right_stop();
	TCCR2A |= ( 1<<COMxA0 | 1<<COMxA1);
}
static void right_duty_cycle_set(uint8_t duty_cycle)
{
	OCR2A = duty_cycle + RIGHT_DUTY_CYCLE_OFFSET;
	OCR2B = duty_cycle + RIGHT_DUTY_CYCLE_OFFSET;
}