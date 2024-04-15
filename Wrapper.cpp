#include <EEPROM.h>
#include "Wrapper.h"

Servo servo8;
Servo servo7;
int button1, button2;
QTRSensors qtr;
int lastError = 0;
const uint8_t SensorCount = SENSOR_CNT;
unsigned int sensorValues[SensorCount];
bool isDebug = false;
int sensAvg[SENSOR_CNT];

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
        sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;

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
        sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;

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
    }
}

/*
 Function: PID calculation
*/
void pid_lineB(EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd) {
    unsigned int position;
    if (lineFormat == BLACK_LINE) {
        position = qtr.readLineBlack(sensorValues);
    } else {
        position = qtr.readLineWhite(sensorValues);
    }

    int err = position - CENTER_VAL;
    int corrSpeed = Kp * err + Kd * (err - lastError);
    lastError = err;

    motordrive("forward", constrain(speedBase + corrSpeed, 0, 250),
               "forward", constrain(speedBase - corrSpeed, 0, 250));
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

    int senseCenter = (qtr.calibrationOn.maximum[sensIndex] + qtr.calibrationOn.minimum[sensIndex]) / 2;
    if (lineFormat == BLACK_LINE) {
        qtr.readLineBlack(sensorValues);
        while (sensorValues[sensIndex] < senseCenter) {
            qtr.readLineBlack(sensorValues);
            motorsteer(actionStr, actionSpeed);
        }
    } else {
        qtr.readLineWhite(sensorValues);
        while (sensorValues[sensIndex] > senseCenter) {
            qtr.readLineWhite(sensorValues);
            motorsteer(actionStr, actionSpeed);
        }
    }
}

/*
 Function: Perform move delay
*/
void moveDelay(EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int delay) {
    int currentMillis = millis();
    while (millis() - currentMillis < delay) {
        pid_lineB(lineFormat, speedBase, Kp, Kd);
    }
}

/*
 Function: Check junction condition
*/
bool checkCondition(EN_JUNCTION junction) {
    switch (junction) {
        case BLACK_T:
            return (sensorValues[0] > sensAvg[0]) && (sensorValues[2] > sensAvg[2]) &&
                   (sensorValues[4] > sensAvg[4]);  // BLACK-T Junction
        case BLACK_RIGHT:
            return (sensorValues[0] < sensAvg[0]) && (sensorValues[2] > sensAvg[2]) &&
                   (sensorValues[4] > sensAvg[4]);  //BLACK-RIGHT Junction

        case BLACK_LEFT:
            return (sensorValues[0] > sensAvg[0]) && (sensorValues[2] > sensAvg[2]) &&
                   (sensorValues[4] < sensAvg[4]);  //BLACK-LEFT Junction

        case WHITE_T:
            return (sensorValues[0] < sensAvg[0]) && (sensorValues[2] < sensAvg[2]) &&
                   (sensorValues[4] < sensAvg[4]);  //WHITE-T Junction

        case WHITE_RIGHT:
            return (sensorValues[0] > sensAvg[0]) && (sensorValues[2] < sensAvg[2]) &&
                   (sensorValues[4] < sensAvg[4]);  //WHITE-RIGHT Junction

        case WHITE_LEFT:
            return (sensorValues[0] < sensAvg[0]) && (sensorValues[2] < sensAvg[2]) &&
                   (sensorValues[4] > sensAvg[4]);  //WHITE-LEFT Junction

        case BLACK_LEFT_ONLY:
            return (sensorValues[0] > sensAvg[0]);  //BLACK-LEFT ONLY Junction

        case BLACK_RIGHT_ONLY:
            return (sensorValues[4] > sensAvg[4]);  //BLACK-RIGHT ONLY Junction

        case BLACK_T_WHITE_CENTER:
            return (sensorValues[0] > sensAvg[0]) && (sensorValues[2] < sensAvg[2]) &&
                   (sensorValues[4] > sensAvg[4]);  //BLACK-T Junction with White Center

        case WHITE_T_BLACK_CENTER:
            return (sensorValues[0] < sensAvg[0]) && (sensorValues[2] > sensAvg[2]) &&
                   (sensorValues[4] < sensAvg[4]);  //WHITE-T Junction with BLACK Center

        default:
            break;
    }
    return false;
}

/*
 Function: Perform move plan
*/
void movePlan(EN_JUNCTION junction, EN_LINE_FORMAT lineFormat, int speedBase, float Kp, float Kd, int timerForward,
              int forwardSpeed, EN_ACTION action, int timerAction, int actionSpeed) {

    while (1) {

        // Normal driving with PID
        pid_lineB(lineFormat, speedBase, Kp, Kd);

        if (checkCondition(junction)) {
            timerAction == 0 ? actionPlanCenter(lineFormat, timerForward, forwardSpeed, action, actionSpeed)
                             : actionPlanDelay(timerForward, forwardSpeed, action, timerAction, actionSpeed);
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
