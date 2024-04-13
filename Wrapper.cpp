#include <EEPROM.h>
#include "Wrapper.h"

Servo servo8;
Servo servo7;
int button1, button2;
int currentmillis = 0;
QTRSensors qtr;
int lastError = 0;
const uint8_t SensorCount = SENSOR_CNT;
unsigned int sensorValues[SensorCount];
unsigned int position = 0;
int rightMotorPwm = 0;
int leftMotorPwm = 0;
bool isDebug = false;

/*
 Function: Save value to EEPROM by address
*/
void saveValue(int addr, uint16_t value) {
    EEPROM.write(addr, value >> 8);
    EEPROM.write(addr + 1, value & 0xFF);
}

/*
 Function: Read value from EEPROM by address
*/
uint16_t readValue(int addr) {
    return (EEPROM.read(addr) << 8) + EEPROM.read(addr + 1);
}

/*
 Function: Restore sensor values from the EEPROM
*/
void restoreSensorValue() {
    qtr.calibrate();
    for (int i = 0; i < SENSOR_CNT; i++) {
        int offset = i * 2;
        qtr.calibrationOn.minimum[i] = readValue(ADDR_BASE + offset);      // 10-11,12-13,14-15,16-17,18-19
        qtr.calibrationOn.maximum[i] = readValue(ADDR_BASE * 5 + offset);  // 50-51,52-53,54-55,56-57,58-59
        if (isDebug) {
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(qtr.calibrationOn.minimum[i]);
            Serial.print(", ");
            Serial.println(qtr.calibrationOn.maximum[i]);
        }
    }
}

/*
 Function: Store sensor values in the EEPROM
*/
void storeSensorValue() {
    for (int i = 0; i < SENSOR_CNT; i++) {
        int offset = i * 2;
        saveValue(ADDR_BASE + offset, qtr.calibrationOn.minimum[i]);      // 10-11,12-13,14-15,16-17,18-19
        saveValue(ADDR_BASE * 5 + offset, qtr.calibrationOn.maximum[i]);  // 50-51,52-53,54-55,56-57,58-59
        if (isDebug) {
            Serial.print(i);
            Serial.print(" - ");
            Serial.print(qtr.calibrationOn.minimum[i]);
            Serial.print(", ");
            Serial.println(qtr.calibrationOn.maximum[i]);
        }
    }
}

/*
 Function: Initialize function
*/
void initialize(bool debug) {

    isDebug = debug;

    servo8.attach(8);
    servo7.attach(7);
    init_motor();

    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(9, OUTPUT);
    if (isDebug) init_serial();
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]) {A6, A3, A2, A1, A0}, SensorCount);
    qtr.releaseEmitterPins();

    restoreSensorValue();
}

/*
 Function: Read button input
*/
void readButton() {
    int adcValue = analogRead(A7);
    if (adcValue < 100) {
        button1 = 0;
        button2 = 1;
    } else if (adcValue < 500) {
        button1 = 1;
        button2 = 0;
    } else {
        button1 = 1;
        button2 = 1;
    }
}

/*
 Function: Get button event
*/
int getButtonEvt(int idx) {
    return idx == 1 ? button1 : button2;
}

/*
 Function: Perform manual calibration
*/
void manualCalibration() {
    qtr.resetCalibration();
    for (int i = 0; i < 150; i++) {
        qtr.calibrate();
        delay(10);
    }
    storeSensorValue();
}

/*
 Function: Convert action to the library language
*/
char *selectAction(EN_ACTION action) {
    switch (action) {
        case MOVE_FORWARD:
            return "forward";
        case MOVE_BACKWARD:
            return "backward";
        case TURN_RIGHT:
            return "right";
        case TURN_LEFT:
            return "left";
        case RIGHT_AT_C:
            return "rightatC";
        case LEFT_AT_C:
            return "leftatC";
        default:
            return "stop";
            break;
    }
}

/*
 Function: PID calculation
*/
void pid_lineB(EN_LINE_FORMAT line_format, int speed_base, float Kp, float Kd) {
    if (line_format == BLACK_LINE) {
        position = qtr.readLineBlack(sensorValues);

        int error1 = position - 2000;
        int correct_speed1 = Kp * error1 + Kd * (error1 - lastError);
        lastError = error1;
        leftMotorPwm = speed_base + correct_speed1;
        rightMotorPwm = speed_base - correct_speed1;
    }

    if (line_format == WHITE_LINE) {
        position = qtr.readLineWhite(sensorValues);

        int error2 = position - 2000;
        int correct_speed2 = Kp * error2 + Kd * (error2 - lastError);
        lastError = error2;
        leftMotorPwm = speed_base + correct_speed2;
        rightMotorPwm = speed_base - correct_speed2;
    }

    if (leftMotorPwm > 255) leftMotorPwm = 255;
    if (rightMotorPwm > 255) rightMotorPwm = 255;
    if (leftMotorPwm < 0) leftMotorPwm = 0;
    if (rightMotorPwm < 0) rightMotorPwm = 0;

    constrain(leftMotorPwm, 0, 250);
    constrain(rightMotorPwm, 0, 250);
    motordrive("forward", leftMotorPwm, "forward", rightMotorPwm);
}


/*
 Function: Release gripper
*/
void releaseGripper() {
    servo8.write(30);
    servo7.write(80);
}


/*
 Function: Close gripper
*/
void closeGripper() {
    servo8.write(150);
    delay(500);
    servo7.write(65);
}


/*
 Function: Calibration
*/
void calibrate() {
    delay(1000);
    digitalWrite(13, HIGH);
    digitalWrite(9, HIGH);
    manualCalibration();
    digitalWrite(13, LOW);
    digitalWrite(9, LOW);
}

/*
 Function: Perform plan action with delay
*/
void actionPlanDelay(int timerForward, int forwardSpeed, EN_ACTION action, int timerAction, int actionSpeed) {
    motorsteer(selectAction(MOVE_FORWARD), forwardSpeed);
    delay(timerForward);
    motorsteer(selectAction(action), actionSpeed);
    delay(timerAction);
}

/*
 Function: Perform plan action by checking center
*/
void
actionPlanCenter(EN_LINE_FORMAT lineFormat, int timerForward, int forwardSpeed, EN_ACTION action, int actionSpeed) {
    motorsteer(selectAction(MOVE_FORWARD), forwardSpeed);
    delay(timerForward);

    int sensIndex = (action == TURN_RIGHT || action == RIGHT_AT_C) ? 3 : (action == TURN_LEFT || action == LEFT_AT_C)
                                                                         ? 1
                                                                         : 2;

    char *actionStr = selectAction(action);
    motorsteer(actionStr, 100);
    delay(100);

    int sensecenter = (qtr.calibrationOn.maximum[sensIndex] + qtr.calibrationOn.minimum[sensIndex]) / 2;
    if (lineFormat == BLACK_LINE) {
        position = qtr.readLineBlack(sensorValues);
        while (sensorValues[sensIndex] < sensecenter) {
            position = qtr.readLineBlack(sensorValues);
            motorsteer(actionStr, actionSpeed);
        }
    } else {
        position = qtr.readLineWhite(sensorValues);
        while (sensorValues[sensIndex] > sensecenter) {
            position = qtr.readLineWhite(sensorValues);
            motorsteer(actionStr, actionSpeed);
        }
    }
}

/*
 Function: Perform move delay
*/
void moveDelay(EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int delay) {
    currentmillis = millis();
    while (millis() - currentmillis < delay) {
        pid_lineB(lineFormat, speedBase, Kp, Kd);
    }
}

/*
 Function: Perform move plan
*/
void movePlan(EN_JUNCTION junction, EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int timerForward,
              int forwardSpeed, EN_ACTION action, int timerAction, int actionSpeed) {
    while (1) {
        int status = 0;
        pid_lineB(lineFormat, speedBase, Kp, Kd);

        int senMin[4];
        senMin[0] = (qtr.calibrationOn.maximum[0] + qtr.calibrationOn.minimum[0]) / 2;
        senMin[1] = (qtr.calibrationOn.maximum[1] + qtr.calibrationOn.minimum[1]) / 2;
        senMin[2] = (qtr.calibrationOn.maximum[2] + qtr.calibrationOn.minimum[2]) / 2;
        senMin[3] = (qtr.calibrationOn.maximum[3] + qtr.calibrationOn.minimum[3]) / 2;
        senMin[4] = (qtr.calibrationOn.maximum[4] + qtr.calibrationOn.minimum[4]) / 2;
        switch (junction) {
            case BLACK_T:
                if ((sensorValues[0] > senMin[0]) && (sensorValues[2] > senMin[2]) &&
                    (sensorValues[4] > senMin[4]))  //BLACK-T Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case BLACK_RIGHT:
                if ((sensorValues[0] < senMin[0]) && (sensorValues[2] > senMin[2]) &&
                    (sensorValues[4] > senMin[4]))  //BLACK-RIGHT Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case BLACK_LEFT:
                if ((sensorValues[0] > senMin[0]) && (sensorValues[2] > senMin[2]) &&
                    (sensorValues[4] < senMin[4]))  //BLACK-LEFT Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case WHITE_T:
                if ((sensorValues[0] < senMin[0]) && (sensorValues[2] < senMin[2]) &&
                    (sensorValues[4] < senMin[4]))  //WHITE-T Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case WHITE_RIGHT:
                if ((sensorValues[0] > senMin[0]) && (sensorValues[2] < senMin[2]) &&
                    (sensorValues[4] < senMin[4]))  //WHITE-RIGHT Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case WHITE_LEFT:
                if ((sensorValues[0] < senMin[0]) && (sensorValues[2] < senMin[2]) &&
                    (sensorValues[4] > senMin[4]))  //WHITE-LEFT Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case BLACK_LEFT_ONLY:
                if ((sensorValues[0] > senMin[0]))  //BLACK-LEFT ONLY Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case BLACK_RIGHT_ONLY:
                if ((sensorValues[4] > senMin[4]))  //BLACK-RIGHT ONLY Junction
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case BLACK_T_WHITE_CENTER:
                if ((sensorValues[0] > senMin[0]) && (sensorValues[2] < senMin[2]) &&
                    (sensorValues[4] > senMin[4]))  //BLACK-T Junction with White Center
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            case WHITE_T_BLACK_CENTER:
                if ((sensorValues[0] < senMin[0]) && (sensorValues[2] > senMin[2]) &&
                    (sensorValues[4] < senMin[4]))  //WHITE-T Junction with BLACK Center
                {
                    timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                                     : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
                    status = 1;
                }
                break;
            default:
                break;
        }
        if (status == 1) {
            status = 0;
            break;
        }
    }
}

void stop() {
    motorsteer(selectAction(STOP), 0);
}

void forward(int speed, int timer) {
    motorsteer(selectAction(MOVE_FORWARD), speed);
    delay(timer);
}

void reverse(int speed, int timer) {
    motorsteer(selectAction(MOVE_BACKWARD), speed);
    delay(timer);
}

void forwardCustom(int speedL, int speedR, int timer) {
    motordrive(selectAction(MOVE_FORWARD), speedL, selectAction(MOVE_FORWARD), speedR);
    delay(timer);
}
