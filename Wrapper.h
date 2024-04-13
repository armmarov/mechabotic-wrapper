

// ensure this library description is only included once

#ifndef __Wrapper_H__
#define __Wrapper_H__

#include <Servo.h>
#include "MechabotRush.h"
#include "QTRSensors.h"

#define SENSOR_CNT 5
#define ADDR_BASE 10

enum EN_JUNCTION {
    BLACK_T,
    BLACK_RIGHT,
    BLACK_LEFT,
    WHITE_T,
    WHITE_RIGHT,
    WHITE_LEFT,
    BLACK_LEFT_ONLY,
    BLACK_RIGHT_ONLY,
    BLACK_T_WHITE_CENTER,
    WHITE_T_BLACK_CENTER
};

enum EN_LINE_FORMAT {
    BLACK_LINE,
    WHITE_LINE
};

enum EN_ACTION {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_RIGHT,
    TURN_LEFT,
    RIGHT_AT_C,
    LEFT_AT_C,
    STOP
};

void initialize(bool debug);

void releaseGripper();

void closeGripper();

void calibrate();

void readButton();

int getButtonEvt(int idx);

void moveDelay(EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int delay);

void movePlan(EN_JUNCTION junction, EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int timerForward,
              int forwardSpeed, EN_ACTION action, int timerAction, int actionSpeed);

void stop();

void forward(int speed, int timer);

void reverse(int speed, int timer);

void forwardCustom(int speedL, int speedR, int timer);

#endif
