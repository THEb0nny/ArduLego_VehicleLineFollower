// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"

#define RESET_BTN_PIN 7 // Кнопка для мягкого перезапуска

#define SERVO_MOT_L_PIN 2 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 4 // Пин правого серво мотора

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(10); // Инициализация объекта таймера

unsigned long currTime, prevTime, loopTime; // Время

float Kp = 1, Ki = 0, Kd = 0; // Коэффиценты регулятора при старте

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP); // Подключение кнопки Start/stop/reset
  lServoMot.attach(SERVO_MOT_L_PIN); rServoMot.attach(SERVO_MOT_R_PIN);
  MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE); // При старте моторы выключаем
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-90, 90); // Пределы регулятора
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  if (!digitalRead(RESET_BTN_PIN)) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    // Считывием и обрабатываем значения с датчиков линии
    int error = 0; // Нахождение ошибки
    regulator.setpoint = error; // Передаём ошибку
    regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    //MotorsControl(-30, 30);
    MotorSpeed(lServoMot, 90, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, -90, SERVO_MOT_R_DIR_MODE);
  }
}

// Управление двумя моторами
void MotorsControl(int dir, byte speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  Serial.println(z);
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  lServoMotSpeed = constrain(lServoMotSpeed, -90, 90), rServoMotSpeed = constrain(rServoMotSpeed, -90, 90);
  Serial.print(lServoMotSpeed); Serial.print(", "); Serial.println(rServoMotSpeed);
  MotorSpeed(lServoMot, lServoMotSpeed, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, rServoMotSpeed, SERVO_MOT_R_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int speed, bool rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90) * rotateMode;
  if (speed >= 0) {
    speed = map(speed, 0, 90, 90, 180);
  } else {
    speed = map(speed, 0, -90, 90, 0);
  }
  //Serial.println(speed);
  servoMot.write(speed);
}
