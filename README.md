# Tutorial Guide

## Import wrapper library

```c++
#include "Wrapper.h"
```

## Initialize wrapper library

```c++
void setup() {
    /*
     * Initialization process initialize the motor, servo, serial and QTR sensor
     * The previously stored sensor calibration value will be loaded from EEPROM
     */
  initialize(false); // Set true to enable serial for debugging
}
```

## Button control

```c++
void loop() {
  readButton(); // Read analog value from button I/O
  if (getButtonEvt(1) == 0) { // get button 1 event
    // Do any process (Eg: plan 1)
  } else if (getButtonEvt(2) == 0) { // get button 2 event
    // Do any process (Eg: calibration)
  }
}
```

## Calibration

```c++
calibrate(); // perform calibration on QTR sensor and store the value in EEPROM
```

## Start the planning

### Enumeration

| Item        | Enum List                                                                                                                                         |
|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| Junction    | BLACK_T, BLACK_RIGHT, BLACK_LEFT, WHITE_T, WHITE_RIGHT, WHITE_LEFT, BLACK_LEFT_ONLY, BLACK_RIGHT_ONLY, BLACK_T_WHITE_CENTER, WHITE_T_BLACK_CENTER |
| Line Format | BLACK_LINE, WHITE_LINE                                                                                                                            |
| Action      | MOVE_FORWARD, MOVE_BACKWARD, TURN_RIGHT, TURN_LEFT, RIGHT_AT_C, LEFT_AT_C, STOP                                                                   |

### Use delay to move PID

- Usage: To move the car with delay using PID
- Parameters:
    - Line format `enum`
    - Speed base `int`
    - KP `float`
    - KD `float`
    - Delay `int`

```c++
// This function move car with base speed of 100, KP of 0.05, KD of 0.5 
// with delay of 700 along the black line
moveDelay(BLACK_LINE, 100, 0.05, 0.5, 700);
```

### Detect junction, perform action

- Usage: To move the car using PID, find the junction and perform action. Automatically checking the center if Timer
  action is 0
- Parameters:
    - Junction `enum`
    - Line format `enum`
    - Speed base `int`
    - KP `float`
    - KD `float`
    - Timer forward `int`
    - Forward speed `int`
    - Action `enum`
    - Timer action `int`
    - Action speed `int`

```c++
// This function move car with base speed of 100, KP of 0.05, KD of 0.5 along the black line
// Upon detecting the BLACK_T junction, it moves forward with delay of 10 and speed of 100
// Then, it turns left at center with delay of 120 and speed of 100
movePlan(BLACK_T, BLACK_LINE, 100, 0.05, 0.5, 10, 100, LEFT_AT_C, 120, 100);

// This function does exactly as above, but upon performing the action, 
// it automatically checks the center line and stop after finding the center line
// To enable auto center checking this, make sure to set Timer action to 0
movePlan(BLACK_T, BLACK_LINE, 100, 0.05, 0.5, 10, 100, LEFT_AT_C, 0, 100);
```

### Move forward

- Usage: To move the car forward
- Parameters:
    - Speed `int`
    - Timer `int`

```c++
// This function move forward with speed of 100 and delay of 200
forward(100, 200);
```

### Move backward

- Usage: To move the car backward
- Parameters:
    - Speed `int`
    - Timer `int`

```c++
// This function move backward with speed of 100 and delay of 200
reverse(100, 200);
```

### Move forward with custom speed

- Usage: To move the car forward with custom left and right speed
- Parameters:
    - Speed Left `int`
    - Speed Right `int`
    - Timer `int`

```c++
// This function move forward with left speed of 45, right speed of 100 and delay of 250
forwardCustom(45, 100, 250);
```

### Stop moving

- Usage: To stop the car

```c++
// This function stop the car movement
stop();
```

### Open gripper

- Usage: To release the object from gripper

```c++
// This function open the gripper to release object
releaseGripper();
```

### Close gripper

- Usage: To close the gripper

```c++
// This function close the gripper to hold any object
closeGripper();
```

## Example usage for Arduino

Please check [example.ino](example.ino) file 