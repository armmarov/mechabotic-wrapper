#include "Wrapper.h"

float KP = 0.05;
float KD = 0.5;

void plan1() {
    delay(1000); // Delay 1 sec before start
    forward(100, 200); // Move forward 0.2 sec
    movePlan(BLACK_T, BLACK_LINE, 100, KP, KD, 10, 100, LEFT_AT_C, 120, 100);
    moveDelay(BLACK_LINE, 100, KP, KD, 100);
    // .... Add your own logics
    moveDelay(BLACK_LINE, 100, KP, KD, 200);
    movePlan(BLACK_T, BLACK_LINE, 100, KP, KD, 0, 0, STOP, 0, 0);
    stop();
}

void setup() {
    initialize(false);
}

void loop() {
    readButton();
    if (getButtonEvt(1) == 0) {
        plan1();
    } else if (getButtonEvt(2) == 0) {
        calibrate();
    }
    stop();
}