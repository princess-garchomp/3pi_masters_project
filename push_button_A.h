#ifndef PUSH_BUTTON_A_H
#define PUSH_BUTTON_A_H


#include <avr/io.h>
#include <stdbool.h>


_Bool push_button_A_is_pressed();
_Bool push_button_A_is_not_pressed();
void wait_push_button_A_press_release();

void config_push_button_A();

#endif