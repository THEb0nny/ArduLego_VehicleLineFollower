// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/

#include <Servo.h>
#include "GyverPID.h"

#define SERVO_MOT_L_PIN 2
#define SERVO_MOT_R_PIN 4

#define RESET_BTN_PIN D4

Servo lServoMot, rServoMot;

GyverPID regulator(0.1, 0.05, 0.01, 10);

void(* resetFunc) (void) = 0; // Soft reset function

void setup() {
  Serial.begin(9600);
  //pinMode(RESET_BTN, INPUT_PULLUP); // start/stop/reset button attachment
  lServoMot.attach(SERVO_MOT_L_PIN);
  rServoMot.attach(SERVO_MOT_R_PIN);
  motorSpeed(lServoMot, 0);
  motorSpeed(rServoMot, 0);
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
}

void loop() {
  //if (!digitalRead(RESET_BTN)) resetFunc();
  motorSpeed(lServoMot, 0);
}

void motorSpeed(Servo servoMot, int speed) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90);
  speed = map(speed, 0, 180, -90, 90);
  servoMot.write(speed);
}
