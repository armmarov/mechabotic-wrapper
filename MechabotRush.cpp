/*
 * 
 */


#include "MechabotRush.h"
#include "QTRSensors.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif



//============================SETTLE DAN TELAH DI CUBA======================//


//tested
void init_input() {
  // initialize button
  // pinMode(buttonpin, INPUT);
}

//tested
void init_motor() {
  //initialize motor
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

//tested
void init_serial() {
  Serial.begin(9600);
}

//tested
void LEDs(int l, char *stat) {
  if (stat == "ON") {
    digitalWrite(l, HIGH);
  }

  else if (stat == "OFF") {
    digitalWrite(l, LOW);
  }
}

//tested
void Buzz(int l, char *stat) {
  if (stat == "ON") {
    digitalWrite(l, HIGH);
  }

  else if (stat == "OFF") {
    digitalWrite(l, LOW);
  }
}







//tested
void motordrive(char *directionL, int powerL, char *directionR, int powerR) {

  if (directionL == "forward") {
    digitalWrite(dir1, HIGH);
  }

  else if (directionL == "backward") {
    digitalWrite(dir1, LOW);
  }

  if (directionR == "forward") {
    digitalWrite(dir2, LOW);
  }

  else if (directionR == "backward") {
    digitalWrite(dir2, HIGH);
  }

  if (PWM1 == 0 && PWM2 == 0) {}

  analogWrite(PWM1, powerL);
  analogWrite(PWM2, powerR);
}

//tested
void motorsteer(char *robotdirection, int power) {
  if (robotdirection == "forward") {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power);
  }

  else if (robotdirection == "backward") {
    digitalWrite(dir1, LOW);
    analogWrite(PWM1, power);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM2, power);
  }

  else if (robotdirection == "right") {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power / 3);
  }

  else if (robotdirection == "rightatC") {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM2, power);
  }

  else if (robotdirection == "left") {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power / 3);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power);
  }

  else if (robotdirection == "leftatC") {
    digitalWrite(dir1, LOW);
    analogWrite(PWM1, power);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power);
  }

  else if (robotdirection == "stop") {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM2, 0);
  }

  else {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, 0);
  }
}



//============================DALAM PERCUBAAN======================//

















/* void pid_lineB(int right_base, int left_base, int speedturn, int maxSpeed, float Kp, float Kd, int line_format, int senspeed)
{
  if (line_format == 1)
  {
    // position = positionQtr;
    if (position > (4000 - senspeed)) { //3700
      motorsteer("rightatC", speedturn);
      return;
    }
    if (position < senspeed) { //300
      motorsteer("leftatC", speedturn);
      return;
    }
    int error1 = position - 2000;
    int correct_speed1 = (Kp * error1) + Kd * (error1 - last_error);
    last_error = error1;
    left_motor_pwm = right_base + correct_speed1;
    right_motor_pwm = left_base - correct_speed1;
    Serial.println("");

  }

  if (line_format == 0)
  {
    // position =   positionQtr;
    if (position > (4000 - senspeed)) { //3700
      motorsteer("rightatC", speedturn);
      return;
    }
    if (position < senspeed) { //300
      motorsteer("leftatC", speedturn);
      return;
    }
    int error2 = position - 2000;
    int correct_speed2 = (Kp * error2) + Kd * (error2 - last_error);
    last_error = error2;
    left_motor_pwm = right_base + correct_speed2;
    right_motor_pwm = left_base - correct_speed2;
  }


  if (left_motor_pwm > maxSpeed )
    left_motor_pwm = maxSpeed;
  if (right_motor_pwm > maxSpeed )
    right_motor_pwm = maxSpeed;

  if (left_motor_pwm < 0)
    left_motor_pwm = 0;
  if (right_motor_pwm < 0)
    right_motor_pwm = 0;

  // constrain(left_motor_pwm, 0, maxSpeed);
  // constrain(right_motor_pwm, 0, maxSpeed);

  motortank("forward", left_motor_pwm, "forward", right_motor_pwm);

  // Serial.print(position);

  // Serial.print("\t");
  // Serial.print(left_motor_pwm);
  // Serial.print(",");
  // Serial.print(right_motor_pwm);
  // Serial.println("");
}
 */










/* void pid_line(int right_base, int left_base, int speedturn, int maxSpeed, float Kp, float Kd, int line_format, int senspeed)
{
    // Position of robot through QTRlibrary
 
 position = readLine(sensorValues,1,line_format); //0 black on white, 1 white on black
 
 LEDsense(position); 
  if(position> (4000-senspeed)){  //3700
  	 
     turn_L(speedturn,speedturn);
   return;    
  }
  if(position< senspeed){    //300
     turn_R(speedturn,speedturn);
    return;
  }
    
  int error = position-2000;  // error

  //////////// speed correction via PID
  int correct_speed = Kp * error + Kd*(error - last_error);  //correction speed
  last_error = error; // lasterror = error
    
   //////////// Exact speed settings to be applied to the motors
   right_motor_pwm = right_base + correct_speed  ; //right motor
   left_motor_pwm = left_base - correct_speed  ; //left motor
  
   if (right_motor_pwm < 0)
    right_motor_pwm = 0;
  if (left_motor_pwm < 0)
    left_motor_pwm = 0;

    constrain(right_motor_pwm,0,maxSpeed);
    constrain(left_motor_pwm,0,maxSpeed);

      advance(right_motor_pwm,left_motor_pwm);
      
  //   Make the bottom line active to show error on serial monitor, right and left motor speed values
  //   Serial.print(error);  Serial.print(" "); Serial.print(right_motor_pwm); Serial.print(" "); Serial.println(left_motor_pwm); delay(100);  
}



void QTRSensors::LEDsense(int positionx)

{
 switch (positionx)
   {
      case 0:     digitalWrite(led1, LOW);
      break;
      case 1000:   digitalWrite(led2, LOW);
      break;
      case 2000:   digitalWrite(led3, LOW);
      break;
      case 3000:   digitalWrite(led4, LOW);
      break;
      case 4000:   digitalWrite(led5, LOW);
      break;
	  default:
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH);
    
    break;
      }
}
 */













/* int ultrasonic(byte trig,byte echo)
{
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
	
	digitalWrite(trig,LOW);
	delayMicroseconds(2);
	digitalWrite(trig,HIGH);
	delayMicroseconds(10);
	digitalWrite(trig,LOW);
	long duration= pulseIn(echo,HIGH);
	int distance= duration*0.034/2;
	if ((distance > 0 )&&(distance < 100 ))
	{
	return distance;
	}
 }
 
 
 int mechabot_LED(byte status, byte ledpin)
 {
	 digitalWrite(status,status);
	
 }
 
 
 float lm35_celsius(byte pin)
 {
	 return analogRead(pin)*0.48828125;
 } */




/* void input_read()
{

  as1 = analogRead(pin_s1);
  as2 = analogRead(pin_s2);
  as3 = analogRead(pin_s3);
  as4 = analogRead(pin_s4);
  as5 = analogRead(pin_s5);

  if (as1 > threshold)
  {
    s1 = 1;
  }
  else if (as1 < threshold)
  {
    s1 = 0;
  }

  if (as2 > threshold)
  {
    s2 = 1;
  }
  else if (as2 < threshold)
  {
    s2 = 0;
  }

  if (as3 > threshold)
  {
    s3 = 1;
  }
  else if (as3 < threshold)
  {
    s3 = 0;
  }

  if (as4 > threshold)
  {
    s4 = 1;
  }
  else if (as4 < threshold)
  {
    s4 = 0;
  }

  if (as5 > threshold)
  {
    s5 = 1;
  }
  else if (as5 < threshold)
  {
    s5 = 0;
  }


} */
