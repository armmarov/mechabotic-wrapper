

// ensure this library description is only included once

#ifndef __MechabotRush_H__
#define __MechabotRush_H__

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


//MOTOR1-LEFT
const int PWM1 = 6;
const int dir1 = 2;
//MOTOR2-RIGHT
const int PWM2 = 5;
const int dir2 = 3;

//Button PIN



//LINE SENSOR PIN


//LED Init
//const int led1 = 13;
//const int led2 = 12;
//const int buzzer = 9;   //pin buzzer di tetapkan oleh block pin





//function prototype step by step updated

void init_output();
void init_input();
void init_motor();
void init_serial();

void LEDs(int l, char *stat);
void Buzz(int l, char *stat);


void motordrive(char *directionL, int powerL, char *directionR, int powerR);
void motorsteer(char *robotdirection, int power);


void read_button();
// void pid_lineB(int right_base, int left_base, int speedturn, int maxSpeed, float Kp, float Kd, int line_format, int senspeed);

// void activated_linefollow();

//variable for line follow




#endif
