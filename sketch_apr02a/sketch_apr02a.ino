// https://www.youtube.com/watch?v=jW3cY4Irs50

#include <Servo.h>

#define SERVO_MOT_L_PIN 2
#define SERVO_MOT_R_PIN 4

#define RESET_BTN_PIN 6

Servo lServoMot, rServoMot;

void(* resetFunc) (void) = 0; // Soft reset function

void setup() {
  Serial.begin(9600);
  //pinMode(RESET_BTN, INPUT_PULLUP); // start/stop/reset button attachment
  lServoMot.attach(SERVO_MOT_L_PIN);
  rServoMot.attach(SERVO_MOT_R_PIN);
  motorSpeed(lServoMot, 0);
  motorSpeed(rServoMot, 0);
}

void loop() {
  //if (!digitalRead(RESET_BTN)) resetFunc();
  motorSpeed(lServoMot, 0);
}

void motorSpeed(Servo servoMot, int speed) {
  // servo, 0->FW, 90->stop, 180->BW
  speed = map(speed, 0, 180, -90, 90);
  servoMot.write(speed);
}
