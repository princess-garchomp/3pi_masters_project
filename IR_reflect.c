#include "IR_reflect.h"

#include <avr/io.h>
#include <stdint.h>
#include <stdfix.h>

#define LEFT_MOST_SENSOR_SHIFT (PC0)
#define LEFT_MID_SENSOR_SHIFT (PC1)
#define CENTER_SENSOR_SHIFT (PC2)
#define RIGHT_MIND_SENSOR_SHIFT (PC3)
#define RIGHT_MOST_SENSOR_SHIFT (PC4)

#define SENSOR_MASK ((1ul<<LEFT_MOST_SENSOR_SHIFT)|(1ul<<LEFT_MID_SENSOR_SHIFT)|(1ul<<CENTER_SENSOR_SHIFT)|(1ul<<RIGHT_MIND_SENSOR_SHIFT)|(1ul<<RIGHT_MIND_SENSOR_SHIFT)|(1ul<<RIGHT_MOST_SENSOR_SHIFT))
#define PIN_C_MASK ((1ul<<LEFT_MOST_SENSOR_SHIFT)|(1ul<<LEFT_MID_SENSOR_SHIFT)|(1ul<<CENTER_SENSOR_SHIFT)|(1ul<<RIGHT_MIND_SENSOR_SHIFT)|(1ul<<RIGHT_MIND_SENSOR_SHIFT)|(1ul<<RIGHT_MOST_SENSOR_SHIFT)|(0<<5)|(0<<6)|(0<<7))
#define EMITTER_SHIFT (PC5)


void config_ir_reflect()
{
	DDRC &= ~(SENSOR_MASK);
	PORTC |= (SENSOR_MASK);
	DDRC |= (1ul<<EMITTER_SHIFT);
	ir_relect_on();
}

void ir_relect_on()
{
	PORTC |= (1ul<<EMITTER_SHIFT);
}
void ir_reflect_off()
{
	PORTC &= ~(1ul<<EMITTER_SHIFT);
}

signed short sat accum get_line_position()
{
	short sat signed accum sum=0;
	uint8_t num_sensors=0;
	uint8_t sensor=get_sensor_value();
	
	if(sensor == 0) return 0;
	
	if( sensor & (1ul<<LEFT_MOST_SENSOR_SHIFT) )
	{
		sum-=4; num_sensors++;
	}
	if( sensor & (1ul<<LEFT_MID_SENSOR_SHIFT) )
	{
		sum-=2; num_sensors++;
	}
	if( sensor & (1ul<<CENTER_SENSOR_SHIFT) )
	{
		sum+=0; num_sensors++;
	}
	if( sensor & (1ul<<RIGHT_MIND_SENSOR_SHIFT) )
	{
		sum+=2; num_sensors++;
	}
	if( sensor & (1ul<<RIGHT_MOST_SENSOR_SHIFT) )
	{
		sum+=4; num_sensors++;
	}
	return sum/num_sensors;
	
}
uint8_t get_sensor_value()
{
	return (SENSOR_MASK) & PINC;
}


_Bool sensor_is_over_line()
{
	return get_sensor_value() != 0;
}


_Bool intersection_met()
{
	return (PINC & (1ul<<RIGHT_MOST_SENSOR_SHIFT) && (PINC & (1ul<<LEFT_MOST_SENSOR_SHIFT)));

}

_Bool right_sesor_state()
{
	return (PINC & (1ul<<RIGHT_MOST_SENSOR_SHIFT));

}
_Bool left_sensor_state()
{
	return (PINC & (1ul<<LEFT_MOST_SENSOR_SHIFT));

}

_Bool center_sesnor_state()
{
	return (PINC & (1ul<<CENTER_SENSOR_SHIFT));

}