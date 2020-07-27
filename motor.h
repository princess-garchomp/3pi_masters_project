#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum{ STOP=0, COAST=1, FORWARD=2, REVERSE=3, CW=4, CCW=5}
	robot_shift;

void config_motors();

void set_duty_cycle(uint8_t duty_cycle);
uint8_t get_duty_cycle();
_Bool shift_robot(robot_shift);
robot_shift get_robot_shift_state();
void steer (int8_t delta_duty_cycle_left);
int8_t get_steering();


#endif