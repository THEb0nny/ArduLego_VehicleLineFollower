// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/
// https://alexgyver.ru/gyverbutton/
// https://disk.yandex.ru/i/8yCIjSCpGo80LA
// https://disk.yandex.ru/i/LAgGz3PMc98vSg
// https://disk.yandex.ru/i/Jad22IaI3DUQwj

/** Arduino I2C blobs example.
 * Settings: Blob detector, I2C, addr 51, Dynamixel API, 5V.
 * Wiring:
 *       Camera         Arduino Camera
 * 1-VF >|O O|  2-+5      SCL  -  IC0
 * 3-Gnd |O O|  4-Gnd     SDA  -  ID1
 * 5-TX   O O|  6-RX      5V   -  +5
 * 7-SCK |O O|  8-SNS     Gnd  -  Gnd
 * 9-IC0 |O O| 10-ID1
 */ 

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"
#include "GyverButton.h"
#include "TrackingCamI2C.h"

#define DEBUG true

#define RESET_BTN_PIN 7 // Пин кнопки для мягкого перезапуска
#define LED_PIN 11 // Пин светодиода

#define SERVO_MOT_L_PIN 2 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 4 // Пин правого серво мотора

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

#define LINE_FOLLOW_SET_POINT 160 // Значение уставки, к которому линия должна стремиться

#define MIN_SPEED_FOR_SERVO_MOT 10 // Минимальное значение для старта серво мотора

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(MS, 10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки
TrackingCamI2C trackingCam; // Инициализация объекта камеры

unsigned long currTime, prevTime, loopTime; // Время

float Kp_hard = 0.5, Kp_easy = 0.3;
float Kp = Kp_easy, Ki = 0, Kd = 0; // Начальные коэффиценты регулятора

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора и dt

int speedEasyLine = 45, speedHardLine = 35;
int speed = speedEasyLine;

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println();
  pinMode(LED_PIN, OUTPUT); // Настраиваем пин светодиода
  // Подключение кнопки start/stop/reset
  btn.setDebounce(50); // Настройка антидребезга кнопки (по умолчанию 80 мс)
  btn.setTimeout(300); // Настройка таймаута на удержание кнопки (по умолчанию 500 мс)
  btn.setClickTimeout(600); // Настройка таймаута между кликами по кнопке (по умолчанию 300 мс)
  btn.setType(HIGH_PULL); // HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC, LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  btn.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка, NORM_CLOSE - нормально-замкнутая кнопка
  btn.setTickMode(AUTO); // MANUAL - нужно вызывать функцию tick() вручную, AUTO - tick() входит во все остальные функции и опрашивается сама!
  // Моторы
  lServoMot.attach(SERVO_MOT_L_PIN, 500, 2500); rServoMot.attach(SERVO_MOT_R_PIN, 500, 2500); // Подключение моторов
  MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE); // При старте моторы выключаем
  //lServoMot.write(90); rServoMot.write(90);
  // Регулятор
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-90, 90); // Пределы регулятора
  trackingCam.init(51, 400000); // cam_id - 1..127, default 51, speed - 100000/400000, cam enables auto detection of master clock
  while (true) { // Ждём пока камера начнёт работать
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    Serial.println(nBlobs); // Выводим количество найденных blobs
    if (nBlobs == 1) break; // Если она нашла линию, то выбрасываем из цикла
    delay(500); // Задержка между проверками
  }
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Ready... Press btn");
  while (!btn.isClick()); // Цикл, в котором проверяем, что нажали на кнопку
  digitalWrite(LED_PIN, LOW);
  Serial.println("Go!!!");
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  if (Serial.available() > 2) {
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String command = Serial.readStringUntil('\n');    
    command.trim();
    String incoming = String(char(command[0])) + String(char(command[1]));
    command.remove(0, 1);
    float value = command.toFloat();
    switch (incoming) {
      case "pr":
        Kp_easy = value;
        break;
      case "ph":
        Kp_hard = value;
        break;
      case "ii":
        regulator.Ki = value;
        break;
      case "dd":
        regulator.Kd = value;
        break;
      case "se":
        speedEasyLine = value;
        break;
      case "sh":
        speedHardLine = value;
        break;
      default:
        break;
    }
  }
  if (btn.isClick()) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    int lineX = 0, lineBottom = 0, lineL = 0, lineR = 0;
    int maxArea = 0;
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    for(int i = 0; i < nBlobs; i++) // Печать информации о blobs
    {
      int area = trackingCam.blob[i].area;
      int cx = trackingCam.blob[i].cx;
      int bottom = trackingCam.blob[i].bottom;
      int left = trackingCam.blob[i].left;
      int right = trackingCam.blob[i].right;
      if (bottom > 230) { // Если линия начинается с нижней части картинки камеры
        if (maxArea < area) { // Если площадь фигуры-линии больше других
          maxArea = area;
          lineX = cx;
          lineBottom = bottom;
          lineL = left;
          lineR = right;
        }
      }
      if (DEBUG) {
        // Печать информации о фигуре
        Serial.print(cx, DEC); Serial.print(" ");
        Serial.print(bottom, DEC); Serial.print(" ");
        Serial.print(left, DEC); Serial.print(" ");
        Serial.print(right, DEC); Serial.print(" ");
        Serial.print(area, DEC); Serial.println();
      }
    }
    int lineArea = maxArea;
    // Считывием и обрабатываем значения с датчиков линии
    int error = (lineX == 0 ? 0 : lineX - LINE_FOLLOW_SET_POINT); // Нахождение ошибки, если линии нет, то значение направления 0
    regulator.setpoint = error; // Передаём ошибку
    if (lineL < 20 || lineR > 300) {
      Kp = Kp_hard;
      speed = speedHardLine;
    } else {
      speed = speedEasyLine;
      Kp = Kp_easy;
    }
    regulator.Kp = Kp;
    regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    if (DEBUG) {
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Line: "); // Пеяать информации о выбранной фигуре
      Serial.print(lineX, DEC); Serial.print(" ");
      Serial.print(lineBottom, DEC); Serial.print(" ");
      Serial.print(lineL, DEC); Serial.print(" ");
      Serial.print(lineR, DEC); Serial.print(" ");
      Serial.print(lineArea, DEC); Serial.println();
      Serial.print("error: "); Serial.println(error);
      Serial.print("u: "); Serial.println(u);
    }
    MotorsControl(u, speed);
    //MotorSpeed(lServoMot, 5, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 11, SERVO_MOT_R_DIR_MODE);
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  lServoMotSpeed = constrain(lServoMotSpeed, -90, 90), rServoMotSpeed = constrain(rServoMotSpeed, -90, 90);
  //Serial.print(lServoMotSpeed); Serial.print(", "); Serial.println(rServoMotSpeed);
  MotorSpeed(lServoMot, lServoMotSpeed, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, rServoMotSpeed, SERVO_MOT_R_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int speed, int rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90) * rotateMode;
  //Serial.print("servoMotSpeed "); Serial.print(speed); // Вывод скорости, которую передали параметром в функцию
  if (speed >= 0) speed = map(speed, 0, 90, 90, 180);
  else speed = map(speed, 0, -90, 90, 0);
  servoMot.write(speed);
  //Serial.print(" convertedMotSpeed "); Serial.println(speed); // Вывод обработанной скорости
}
