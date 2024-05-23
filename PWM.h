#ifndef PWM_H_
#define PWM_H_

#include <avr/io.h>

void PWM_init(void);
void servo_writeA(float valADC);
void servo_writeB(float valADC);
float map(float x, float in_min, float in_max, float out_min, float out_max);


#endif