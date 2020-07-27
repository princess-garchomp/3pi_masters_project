#ifndef RADIO_IO_H
#define RADIO_IO_H

typedef enum {SEND_SIGNAL_NONE=0, SEND_SIGNAL_1=1, SEND_SIGNAL_2=2, SEND_BOTH=3}
	send_signal;
//typedef enum {RECEIVE_SIGNAL_CHECK_NONE=0, RECEIVE_SIGNAL_CHECK_1=1, RECEIVE_SIGNAL_CHECK_2=2, RECEIVE_SIGNAL_CHECK_BOTH=3}
	//receive_signal_check;
	
void config_radio_io();

void set_signal_output(send_signal out);
//void set_signal_check_input(receive_signal_check in);

_Bool check_receive_signal_1_true();
_Bool check_receive_signal_1_false();

_Bool check_receive_singal_2_true();
_Bool check_receive_signal_2_false();



#endif