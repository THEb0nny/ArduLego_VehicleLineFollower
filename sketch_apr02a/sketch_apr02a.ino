// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"

#define SERVO_MOT_L_PIN 2 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 4 // Пин правого серво мотора
#define RESET_BTN_PIN 7 // Кнопка для мягкого перезапуск

Servo lServoMot, rServoMot;

GyverPID regulator(1, 0, 0, 10); // Инициализируем коэффициенты регулятора

void(* softResetFunc) (void) = 0; // Soft reset function

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP); // Start/stop/reset button attachment
  lServoMot.attach(SERVO_MOT_L_PIN);
  rServoMot.attach(SERVO_MOT_R_PIN);
  //motorSpeed(lServoMot, 0); // При старте моторы выключаем
  //motorSpeed(rServoMot, 0);
  lServoMot.write(90);
  rServoMot.write(90);
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255); // Пределы регулятора
}

void loop() {
  if (!digitalRead(RESET_BTN_PIN)) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  regulator.setpoint = 0; // Передаём ошибку
  float u = regulator.getResultTimer();
  MotorsControl(-30, 50);
}

void MotorsControl(int dir, byte speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  int z = speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  MotorSpeed(lServoMot, lServoMotSpeed); MotorSpeed(rServoMot, rServoMotSpeed);
}

void MotorSpeed(Servo servoMot, int speed) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90);
  if (speed >= 0) {
    speed = map(speed, 0, 90, 90, 180);
  } else {
    speed = map(speed, 0, -90, 90, 0);
  }
  Serial.println(speed);
  servoMot.write(speed);
}
