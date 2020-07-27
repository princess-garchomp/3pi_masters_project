
#include <stdfix.h>
#include <stdint.h>

#include "IR_reflect.h"
#include "motor.h"
#include "push_button_A.h"
#include "radio_IO.h"


#define COUNTER_CONSTANT_INTERSECTION_MET 30
#define COUNTER_CONSTATN_STOP_AT_LEFT 1500
#define COUNTER_CONSTATN_STOP_AT_RIGHT 1500
#define CONSTANT_COUNTER_IGNORE_STOP_AT_RIGHT_OR_LEFT 5000
#define CONSTANT_COUNTER_IGNORE_SENSORS_AFTER_INTERSECTION_ROTATION 5000

#define COUNTER_CONSTANT_SIGNAL_SEND_ACKNOWLEDGE 500

#define COUNTER_CONSTANT_SIGNAL_RIGH_RECIEVED 3
#define COUNTER_CONSTANT_SIGNAL_LEFT_REDIEVED 3
#define COUNTER_CONSTANT_SIGNAL_RECIEVE_ACKNOWLEDGE 3


#define CONSTANT_COUNTER_MOVE_THREW_LINE_TIME_BUFFER 100

#define INTERSECTION_LINE_LENGTH_MULTIPLYER 1

#define DUTY_CYCLE 20
#define STEER_SCALER 15


enum {ROTATION_FALSE, INITIAL_CHECK_FOR_LINE, MEASURE_LINE, PASSED_LINE, CENTERED_OVER_LINE}
	g_center_sensor_evaluation_for_roatation = ROTATION_FALSE;
enum {STOPPED, FOLLOW_LINE, FORWARD_NO_STEER, ROTATE_RIGH_AFTER_SIGNAL_RECIEVE, ROTATE_LEFT_AFTER_SIGNAL_RECIEVE, ROTATE_LEFT_TO_CENTER_OF_LINE, ROTATE_RIGHT_TO_CENTER_OF_LINE}
	g_movement_state = STOPPED;
enum {FOLLOWING_THE_LINE,PASS_THREW_AND_MEASURE_INTERSECTION, PASS_INTERSECTION_LINE_KNOWN_AMOUNT, WHEELS_OVER_INTERSECTION_LINE,COMMING_OUT_OF_INTERSECTION_AFTER_ROTATION_AND_MOVE_TO_NEXT_LINE,  STOP_AT_RIGHT, STOP_AT_LEFT, COMMING_OUT_OF_STOP_AT_RIGHT_OR_LEFT}
	g_position_in_maze_evaluation = FOLLOWING_THE_LINE;
enum {TYPE_OF_SIGNAL_FALSE, TYPE_OF_SIGNAL_TURN_RIGHT, TYPE_OF_SIGNAL_TURN_LEFT, TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED__RIGHT_TURN, TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED_LEFT_TURN}
	g_type_of_signal_recieved = TYPE_OF_SIGNAL_FALSE;

int8_t steer_bot = 0;

void movement_fsm();
void evaluate_ir_leds_for_position_in_maze_fsm();
void rotate_position_evaluation_fsm();
void send_signal_fsm();
void check_for_signal_fsm();

int main(void)
{
    config_ir_reflect();
	config_motors();
	config_push_button_A();
	config_radio_io();

	uint8_t speed = DUTY_CYCLE;
	set_duty_cycle(speed);

    while (1) 
    {
 		movement_fsm();
 		evaluate_ir_leds_for_position_in_maze_fsm();
  		rotate_position_evaluation_fsm();
  		send_signal_fsm();
 		check_for_signal_fsm();

    }
}



void movement_fsm()
{	
	switch(g_movement_state)
	{
		case(STOPPED):
			shift_robot(STOP);
			if (((push_button_A_is_pressed()) || (g_position_in_maze_evaluation == COMMING_OUT_OF_STOP_AT_RIGHT_OR_LEFT))){
				g_movement_state = FOLLOW_LINE;
			}
			if((g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE) && (g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_RIGHT)){
				g_movement_state = ROTATE_RIGH_AFTER_SIGNAL_RECIEVE;
			}
			if((g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE) && (g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_LEFT)){
				g_movement_state = ROTATE_LEFT_AFTER_SIGNAL_RECIEVE;
			}
		break;
		case(FOLLOW_LINE):
			shift_robot(FORWARD);
			steer_bot = get_line_position();
			steer(steer_bot*STEER_SCALER);	
			if(g_position_in_maze_evaluation == PASS_THREW_AND_MEASURE_INTERSECTION){
				g_movement_state = FORWARD_NO_STEER;
				steer(0);
			}
			if((g_position_in_maze_evaluation == STOP_AT_RIGHT)|| (g_position_in_maze_evaluation == STOP_AT_LEFT)){
				g_movement_state = STOPPED;
			}
		break;
		case(FORWARD_NO_STEER):
			shift_robot(FORWARD);
			steer(steer_bot*STEER_SCALER);
			if(g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE){
				g_movement_state = STOPPED;
			}
 			if(g_position_in_maze_evaluation == FOLLOWING_THE_LINE){
				g_movement_state = FOLLOW_LINE;
			}
		break;
		case(ROTATE_RIGH_AFTER_SIGNAL_RECIEVE):
			shift_robot(CW);
			if(g_center_sensor_evaluation_for_roatation == PASSED_LINE){
				shift_robot(STOP);
				g_movement_state = ROTATE_LEFT_TO_CENTER_OF_LINE;
			}
		break;
		case(ROTATE_LEFT_AFTER_SIGNAL_RECIEVE):
			shift_robot(CCW);
			if(g_center_sensor_evaluation_for_roatation == PASSED_LINE){
				shift_robot(STOP);
				g_movement_state = ROTATE_RIGHT_TO_CENTER_OF_LINE;
			}
		break;
		case(ROTATE_RIGHT_TO_CENTER_OF_LINE):
			shift_robot(CW);
			if(g_center_sensor_evaluation_for_roatation == CENTERED_OVER_LINE){
				shift_robot(STOP);
				g_movement_state = FOLLOW_LINE;
			}
		break;
		case(ROTATE_LEFT_TO_CENTER_OF_LINE):
			shift_robot(CCW);
			if(g_center_sensor_evaluation_for_roatation == CENTERED_OVER_LINE){
				shift_robot(STOP);
				g_movement_state = FOLLOW_LINE;
			}
		break;
	}
}

void evaluate_ir_leds_for_position_in_maze_fsm()
{
	static int intersection_met_timer = 0;
	static int right_turn_met_timer = 0;
	static int left_turn_met_timer = 0;
	static int measure_intersection_line_width_in_main_loops = 0;
	static int move_threw_line_time_buffer = 0;
	static int ignor_stops_after_intersection_rotation = 0;
	static int ignore_stop_at_right_or_left_timer = 0;
	
	switch(g_position_in_maze_evaluation)
	{
 		case(FOLLOWING_THE_LINE):
		if(intersection_met() && (g_movement_state == FOLLOW_LINE)){
			intersection_met_timer = intersection_met_timer + 1;
				if(intersection_met_timer > COUNTER_CONSTANT_INTERSECTION_MET){
					measure_intersection_line_width_in_main_loops = intersection_met_timer;
					intersection_met_timer = 0;
					g_position_in_maze_evaluation = PASS_THREW_AND_MEASURE_INTERSECTION;
				}
				else{}
		}
		else{intersection_met_timer = 0;}
		if(right_sesor_state() && (!intersection_met()) && (g_movement_state == FOLLOW_LINE)){
			right_turn_met_timer = right_turn_met_timer + 1;
			left_turn_met_timer = 0;
				if(right_turn_met_timer > COUNTER_CONSTATN_STOP_AT_RIGHT){
					right_turn_met_timer = 0;
					g_position_in_maze_evaluation = STOP_AT_RIGHT;
				}
				else{}
		}
		else{right_turn_met_timer = 0;}
		if(left_sensor_state() && (!intersection_met()) && (g_movement_state == FOLLOW_LINE)){
			left_turn_met_timer = left_turn_met_timer + 1;
			right_turn_met_timer = 0;
				if(left_turn_met_timer > COUNTER_CONSTATN_STOP_AT_LEFT){
					left_turn_met_timer = 0;
					g_position_in_maze_evaluation = STOP_AT_LEFT;
				}
				else{}
		}
		else{left_turn_met_timer = 0;}	
		break;
		case(PASS_THREW_AND_MEASURE_INTERSECTION):
			measure_intersection_line_width_in_main_loops = measure_intersection_line_width_in_main_loops + INTERSECTION_LINE_LENGTH_MULTIPLYER;
			if(!intersection_met()){
				move_threw_line_time_buffer = move_threw_line_time_buffer +1;
				if(move_threw_line_time_buffer > CONSTANT_COUNTER_MOVE_THREW_LINE_TIME_BUFFER){
					measure_intersection_line_width_in_main_loops = measure_intersection_line_width_in_main_loops - move_threw_line_time_buffer;
					move_threw_line_time_buffer = 0;
					g_position_in_maze_evaluation = PASS_INTERSECTION_LINE_KNOWN_AMOUNT;
				}
				else{}			
			}
			else{move_threw_line_time_buffer = 0;}
		break;
		case(PASS_INTERSECTION_LINE_KNOWN_AMOUNT):
			measure_intersection_line_width_in_main_loops = measure_intersection_line_width_in_main_loops - 1;	
			if(measure_intersection_line_width_in_main_loops < 2){
				measure_intersection_line_width_in_main_loops = 0;
				g_position_in_maze_evaluation = WHEELS_OVER_INTERSECTION_LINE;
			}
		break;
		case(WHEELS_OVER_INTERSECTION_LINE):
			if(g_center_sensor_evaluation_for_roatation == CENTERED_OVER_LINE){
				g_position_in_maze_evaluation = COMMING_OUT_OF_INTERSECTION_AFTER_ROTATION_AND_MOVE_TO_NEXT_LINE;
			}
		break;
		case(COMMING_OUT_OF_INTERSECTION_AFTER_ROTATION_AND_MOVE_TO_NEXT_LINE):
			ignor_stops_after_intersection_rotation = ignor_stops_after_intersection_rotation + 1;
			if (ignor_stops_after_intersection_rotation > CONSTANT_COUNTER_IGNORE_SENSORS_AFTER_INTERSECTION_ROTATION){
				ignor_stops_after_intersection_rotation = 0;
				g_position_in_maze_evaluation = FOLLOWING_THE_LINE;
			}
			else{}
		break;
		case(STOP_AT_RIGHT):
			if(g_type_of_signal_recieved == TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED__RIGHT_TURN){
				g_position_in_maze_evaluation = COMMING_OUT_OF_STOP_AT_RIGHT_OR_LEFT;
			}
		break;
		case(STOP_AT_LEFT):
			if(g_type_of_signal_recieved == TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED_LEFT_TURN){
				g_position_in_maze_evaluation = COMMING_OUT_OF_STOP_AT_RIGHT_OR_LEFT;
			}
		break;
		case(COMMING_OUT_OF_STOP_AT_RIGHT_OR_LEFT):
			ignore_stop_at_right_or_left_timer = ignore_stop_at_right_or_left_timer +1;
			if (ignore_stop_at_right_or_left_timer > CONSTANT_COUNTER_IGNORE_STOP_AT_RIGHT_OR_LEFT){
				ignore_stop_at_right_or_left_timer = 0;
				g_position_in_maze_evaluation = FOLLOWING_THE_LINE;
			}
		break;
	}
}
void rotate_position_evaluation_fsm(){
	static int measure_length_of_line_in_main_loops = 0;
	
	switch(g_center_sensor_evaluation_for_roatation){
		case(ROTATION_FALSE):
			if(((g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_RIGHT) || (g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_LEFT))){
			g_center_sensor_evaluation_for_roatation = INITIAL_CHECK_FOR_LINE;
			}
		break;
		case(INITIAL_CHECK_FOR_LINE):
			if(center_sesnor_state()){
				g_center_sensor_evaluation_for_roatation = MEASURE_LINE;
			}
		break;
		case(MEASURE_LINE):
			measure_length_of_line_in_main_loops = measure_length_of_line_in_main_loops + 1;
			if(!(center_sesnor_state())){
				g_center_sensor_evaluation_for_roatation = PASSED_LINE;
			}
		break;
		case(PASSED_LINE):
			measure_length_of_line_in_main_loops = measure_length_of_line_in_main_loops - 2;
			if(measure_length_of_line_in_main_loops < 2){
				measure_length_of_line_in_main_loops = 0;
				g_center_sensor_evaluation_for_roatation = CENTERED_OVER_LINE;
			}
		break;
		case(CENTERED_OVER_LINE):
			g_center_sensor_evaluation_for_roatation = ROTATION_FALSE;
		break;
	}
}
void send_signal_fsm()
{
	static enum {NOT_SEND_SIGNAL, SEND_SIGNAL_RIGHT, SEND_SIGNAL_LEFT, SEND_SIGNAL_ACKNOWLEDGE_RIHGT_TURN, SEND_SIGNAL_ACKNOWLEDGE_LEFT_TURN}
			local_signal_send_state = NOT_SEND_SIGNAL;
	static int send_acknowledge_signal_timer = 0;
	
	switch (local_signal_send_state){
		case(NOT_SEND_SIGNAL):
			set_signal_output(SEND_SIGNAL_NONE);
			if((g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE) &&  (g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_RIGHT)){
				local_signal_send_state = SEND_SIGNAL_ACKNOWLEDGE_RIHGT_TURN;
			}
			if((g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE) && ((g_type_of_signal_recieved == TYPE_OF_SIGNAL_TURN_LEFT) )){
				local_signal_send_state = SEND_SIGNAL_ACKNOWLEDGE_LEFT_TURN;
			}
			if(g_position_in_maze_evaluation == STOP_AT_RIGHT){
				local_signal_send_state = SEND_SIGNAL_RIGHT;
			}
			if(g_position_in_maze_evaluation == STOP_AT_LEFT){
				local_signal_send_state = SEND_SIGNAL_LEFT;
			}
		break;
		case(SEND_SIGNAL_ACKNOWLEDGE_RIHGT_TURN):
			set_signal_output (SEND_SIGNAL_2);
			send_acknowledge_signal_timer = send_acknowledge_signal_timer + 1;
			if(send_acknowledge_signal_timer > COUNTER_CONSTANT_SIGNAL_SEND_ACKNOWLEDGE){
				local_signal_send_state = NOT_SEND_SIGNAL;
				send_acknowledge_signal_timer = 0;
			}
		break;
		case(SEND_SIGNAL_ACKNOWLEDGE_LEFT_TURN):
		set_signal_output (SEND_SIGNAL_1);
		send_acknowledge_signal_timer = send_acknowledge_signal_timer + 1;
		if(send_acknowledge_signal_timer > COUNTER_CONSTANT_SIGNAL_SEND_ACKNOWLEDGE){
			local_signal_send_state = NOT_SEND_SIGNAL;
			send_acknowledge_signal_timer = 0;
		}
		break;
		case(SEND_SIGNAL_RIGHT):
			//set_signal_output(SEND_SIGNAL_1);//comment out this line to demmo with one car going threw maze
			if(g_type_of_signal_recieved == TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED__RIGHT_TURN){
				local_signal_send_state = NOT_SEND_SIGNAL;
			}
		break;
		case(SEND_SIGNAL_LEFT):
			//set_signal_output(SEND_SIGNAL_2);//comment out this line to demmo with one car going threw maze
			if(g_type_of_signal_recieved == TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED_LEFT_TURN){
				local_signal_send_state = NOT_SEND_SIGNAL;
			}
		break;	
	}
}
void check_for_signal_fsm()
{
	static enum {NOT_CHECKING_FOR_SIGNAL, CHECK_FOR_RIGHT_OR_LEFT_SIGNAL, CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_RIGHT, CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_LEFT}
		local_signal_check_state = NOT_CHECKING_FOR_SIGNAL;
	static int check_acknowledge_signal_timer = 0;
	static int check_signal_right_recieve_timer = 0;
	static int check_signal_left_recieve_timer = 0;
		
		switch(local_signal_check_state)
		{
			case(NOT_CHECKING_FOR_SIGNAL):
				g_type_of_signal_recieved = TYPE_OF_SIGNAL_FALSE;
				if(g_position_in_maze_evaluation == STOP_AT_RIGHT){
					local_signal_check_state = CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_RIGHT;
				}
				if(g_position_in_maze_evaluation == STOP_AT_LEFT){
					local_signal_check_state = CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_LEFT;
				}
				if(g_position_in_maze_evaluation == WHEELS_OVER_INTERSECTION_LINE){
					local_signal_check_state = CHECK_FOR_RIGHT_OR_LEFT_SIGNAL;
				}
			break;
			case(CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_RIGHT):
				if(check_receive_singal_2_true()){
					check_acknowledge_signal_timer = check_acknowledge_signal_timer + 1;
					if(check_acknowledge_signal_timer > COUNTER_CONSTANT_SIGNAL_RECIEVE_ACKNOWLEDGE){
						check_acknowledge_signal_timer = 0;
						local_signal_check_state = NOT_CHECKING_FOR_SIGNAL;
						g_type_of_signal_recieved = TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED__RIGHT_TURN;
					}
					else{}
				}
				else{check_acknowledge_signal_timer = 0;}	
			break;
			case(CHECK_FOR_PARTNER_ACKNOWLEDGE_SIGNAL_RECIEVED_LEFT):
			if(check_receive_signal_1_true()){
				check_acknowledge_signal_timer = check_acknowledge_signal_timer + 1;
				if(check_acknowledge_signal_timer > COUNTER_CONSTANT_SIGNAL_RECIEVE_ACKNOWLEDGE){
					check_acknowledge_signal_timer = 0;
					local_signal_check_state = NOT_CHECKING_FOR_SIGNAL;
					g_type_of_signal_recieved = TYPE_OF_SIGNAL_ACKNOWLEDGE_PARTNER_RECIEVED_LEFT_TURN;
				}
				else{}
			}
			else{check_acknowledge_signal_timer = 0;}
			break;
			case(CHECK_FOR_RIGHT_OR_LEFT_SIGNAL):
				if(check_receive_signal_1_true()){
					check_signal_right_recieve_timer = check_signal_right_recieve_timer + 1;
					check_signal_left_recieve_timer = 0;
					if(check_signal_right_recieve_timer > COUNTER_CONSTANT_SIGNAL_RIGH_RECIEVED){
							check_signal_right_recieve_timer = 0;
							local_signal_check_state = NOT_CHECKING_FOR_SIGNAL;
							g_type_of_signal_recieved = TYPE_OF_SIGNAL_TURN_RIGHT;
					}
					else{}
				}
				else{check_signal_right_recieve_timer = 0;}			
				if(check_receive_singal_2_true()){
					check_signal_left_recieve_timer = check_signal_left_recieve_timer + 1;
					check_signal_right_recieve_timer = 0;
						if(check_signal_left_recieve_timer > COUNTER_CONSTANT_SIGNAL_LEFT_REDIEVED){
							check_signal_left_recieve_timer = 0;
							local_signal_check_state = NOT_CHECKING_FOR_SIGNAL;
							g_type_of_signal_recieved = TYPE_OF_SIGNAL_TURN_LEFT;
				}
				else{}
				}
				else{check_signal_left_recieve_timer = 0;}			
			break;	
		}
}