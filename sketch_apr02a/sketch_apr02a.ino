// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/
// https://alexgyver.ru/gyverbutton/
// https://disk.yandex.ru/i/8yCIjSCpGo80LA
// https://disk.yandex.ru/i/Jad22IaI3DUQwj
// https://disk.yandex.ru/i/LAgGz3PMc98vSg

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

#define PID_OPTIMIZED_I // Параметр для оптимизации суммы регулятора

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"
#include "GyverButton.h"
#include "TrackingCamI2C.h"

#define DEBUG_LEVEL 1 // Уровень дебага

#define RESET_BTN_PIN 7 // Пин кнопки для мягкого перезапуска
#define LED_PIN 11 // Пин светодиода

#define SERVO_MOT_L_PIN 2 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 4 // Пин правого серво мотора

#define GEEKSERVO_STEPPING_PULSE 1500 // Значение импулста для остановки мотора, нулевой скорости geekservo
#define GEEKSERVO_CW_LEFT_BOARD_PULSE_WIDTH 1595 // Левая граница ширины импульса вравщения по часовой geekservo
#define GEEKSERVO_CW_RIGHT_BOARD_PULSE_WIDTH 2500 // Правая граница ширины импульса вращения по часовой geekservo
#define GEEKSERVO_CCW_LEFT_BOARD_PULSE_WIDTH 500 // Минимальное значение ширины импульса вравщения против часовой geekservo
#define GEEKSERVO_CCW_RIGHT_BOARD_PULSE_WIDTH 1365 // Максимальное значение ширины импульса вращения против часовой geekservo

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

#define LINE_FOLLOW_SET_POINT 160 // Значение уставки, к которому линия должна стремиться - это центр кадра

#define LINE_HORISONTAL_POS_OFFSET_BORDER 20 // Значение отспупа слева/справ от края
#define LINE_HORISONTAL_POS_THERSHOLD_LEFT LINE_HORISONTAL_POS_OFFSET_BORDER // Левая граница определения сложного поворота
#define LINE_HORISONTAL_POS_THERSHOLD_RIGHT 320 - LINE_HORISONTAL_POS_OFFSET_BORDER // Правая граница определения сложного поворота

#define LINE_X_IN_CENTER_BORDER_VAL 15 // Значение отклонения прямого участка 
#define LINE_X_IN_CENTER_LEFT_BOARD LINE_FOLLOW_SET_POINT - LINE_X_IN_CENTER_BORDER_VAL // Определние линии в центре, левая граница
#define LINE_X_IN_CENTER_RIGHT_BOARD LINE_FOLLOW_SET_POINT + LINE_X_IN_CENTER_BORDER_VAL // Определние линии в центре, правая граница

#define LINE_Y_BOTTOM_START 230 // Значение от которого стоит отмечать, что мы нашли действительно линию

#define MAX_CAM_WAIT_IN_START 6000 // Максимальное время ожидания подключения камеры, это защитный параметр

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(MS, 10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки
TrackingCamI2C trackingCam; // Инициализация объекта камеры

unsigned long currTime, prevTime, loopTime; // Время

float Kp_easy = 0.3, Kp_hard = 2.7; // Пропрорциональные коэффиценты, при прямых участках и поворотах
float Kd_easy = 0.7, Kd_hard = 0; // Дифференциальные коэффициенты, при прямых участках и поворотах
float Kp = Kp_easy, Ki = 0.01, Kd = Kd_easy; // Начальные коэффиценты регулятора

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора и dt

int speedEasyLine = 70, speedStandartLine = 50, speedHardLine = 35; // Значения скорости на простом и сложном участке
int speed = speedEasyLine; // Скорость

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println();
  pinMode(LED_PIN, OUTPUT); // Настраиваем пин светодиода
  // Подключение кнопки start/stop/reset
  btn.setDebounce(80); // Настройка антидребезга кнопки (по умолчанию 80 мс)
  btn.setTimeout(500); // Настройка таймаута на удержание кнопки (по умолчанию 500 мс)
  btn.setClickTimeout(300); // Настройка таймаута между кликами по кнопке (по умолчанию 300 мс)
  btn.setType(HIGH_PULL); // HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC, LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  btn.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка, NORM_CLOSE - нормально-замкнутая кнопка
  btn.setTickMode(AUTO); // MANUAL - нужно вызывать функцию tick() вручную, AUTO - tick() входит во все остальные функции и опрашивается сама!
  // Моторы
  lServoMot.attach(SERVO_MOT_L_PIN); rServoMot.attach(SERVO_MOT_R_PIN); // Подключение моторов
  MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE); // При старте моторы выключаем
  // Регулятор
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-90, 90); // Пределы регулятора
  trackingCam.init(51, 400000); // cam_id - 1..127, default 51, speed - 100000/400000, cam enables auto detection of master clock
  while (true) { // Ждём пока камера начнёт работать
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    Serial.println(nBlobs); // Выводим количество найденных blobs
    if (nBlobs == 1 || millis() >= MAX_CAM_WAIT_IN_START) break; // Если она нашла 1 линию, то выбрасываем из цикла или выбрасываем в том случае, если прошлом максимальное время ожидания
    delay(500); // Задержка между проверками
  }
  digitalWrite(LED_PIN, HIGH); // Включаем светодио
  Serial.println("Ready... Press btn");
  while (!btn.isClick()); // Цикл, в котором проверяем, что нажали на кнопку
  digitalWrite(LED_PIN, LOW); // Выключаем светодиод
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
    command.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = command.length(); // Переменая для хронения индекса вхождения цифры в входной строке
    // Поиск первого вхождения цифры от 0 по 9 в подстроку
    for (byte i = 0; i < 10; i++) {
      byte index = command.indexOf(String(i));
      if (index < strIndex && index != 255) strIndex = index;
    }
    String incoming = command.substring(0, strIndex);
    String valueStr = command.substring(strIndex, command.length());
    float value = valueStr.toFloat();
    if (incoming == "pe") {
      Kp_easy = value;
      regulator.Kp = Kp_easy;
    } else if (incoming == "ph") {
      Kp_hard = value;
      regulator.Kp = Kp_hard;
    } else if (incoming == "i") {
      regulator.Ki = value;
      regulator.integral = 0;
    } else if (incoming == "de") {
      Kd_easy = value;
      regulator.Kd = Kd_easy;
    } else if (incoming == "dh") {
      Kd_hard = value;
      regulator.Kd = Kd_hard;
    } else if (incoming == "se") {
      speedEasyLine = value;
    } else if (incoming == "sh") {
      speedHardLine = value;
    } else if (incoming == "ss") {
      speedStandartLine = value;
    }
    if (DEBUG_LEVEL >= 1) { // Печать информации о фигуре
      Serial.print(incoming);
      Serial.print(" = ");
      Serial.println(value);
    }
  }
  if (btn.isClick()) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    int lineX = 0, lineY = 0, lineB = 0, lineL = 0, lineR = 0;
    int maxArea = 0;
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    for(int i = 0; i < nBlobs; i++) // Печать информации о blobs
    {
      int area = trackingCam.blob[i].area;
      int cx = trackingCam.blob[i].cx;
      int cy = trackingCam.blob[i].cy;
      int bottom = trackingCam.blob[i].bottom;
      int left = trackingCam.blob[i].left;
      int right = trackingCam.blob[i].right;
      if (bottom > LINE_Y_BOTTOM_START) { // Если линия начинается с нижней части картинки камеры
        if (maxArea < area) { // Если площадь текущей фигуры-линии больше других
          maxArea = area;
          lineX = cx; lineY = cy;
          lineB = bottom;
          lineL = left; lineR = right;
        }
      }
      if (DEBUG_LEVEL >= 2) { // Печать информации о фигуре
        Serial.print(cx, DEC); Serial.print(" "); Serial.print(cy, DEC); Serial.print(" ");
        Serial.print(bottom, DEC); Serial.print(" ");
        Serial.print(left, DEC); Serial.print(" "); Serial.print(right, DEC); Serial.print(" ");
        Serial.print(area, DEC); Serial.println();
      }
    }
    int lineArea = maxArea;
    // Считывием и обрабатываем значения с датчиков линии
    int error = (lineX == 0 ? 0 : lineX - LINE_FOLLOW_SET_POINT); // Нахождение ошибки, если линия не найдена, то значение направления 0
    regulator.setpoint = error; // Передаём ошибку регулятору
    // Если линия замечена с края кадра
    if (lineL < LINE_HORISONTAL_POS_THERSHOLD_LEFT || lineR > LINE_HORISONTAL_POS_THERSHOLD_RIGHT) {
      Kp = Kp_hard;
      Kd = Kd_hard;
      speed = speedHardLine;
    } else if (lineX > LINE_X_IN_CENTER_LEFT_BOARD || LINE_X_IN_CENTER_RIGHT_BOARD < lineX) { // Если линия близка к центру
      Kp = Kp_easy;
      Kd = Kd_easy;
      speed = speedEasyLine;
      regulator.integral = 0; // Обнуляем интегральную составляющую
    } else { // Простая линия
      Kp = Kp_easy;
      Kd = Kd_easy;
      speed = speedStandartLine;
    }
    if (regulator.Kp != Kp) regulator.Kp = Kp; // Установка значений Kp, если они были изменены
    if (regulator.Ki != Ki) regulator.Ki = Ki; // Установка значений Ki, если они были изменены
    if (regulator.Kd != Kd) regulator.Kd = Kd; // Установка значений Kd, если они были изменены
    regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    MotorsControl(u, speed);
    //MotorSpeed(lServoMot, -90, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, -90, SERVO_MOT_R_DIR_MODE); // Для тестирования моторов по отдельности
    if (DEBUG_LEVEL >= 2) {
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Line: "); // Пеяать информации о выбранной фигуре
      Serial.print(lineX, DEC); Serial.print(" ");
      Serial.print(lineY, DEC); Serial.print(" "); Serial.print(lineB, DEC); Serial.print(" ");
      Serial.print(lineL, DEC); Serial.print(" "); Serial.print(lineR, DEC); Serial.print(" ");
      Serial.print(lineArea, DEC); Serial.println();
    }
    if (DEBUG_LEVEL >= 1) {
      Serial.print("error: "); Serial.println(error);
      Serial.print("u: "); Serial.println(u);
    }
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  lServoMotSpeed = constrain(lServoMotSpeed, -90, 90), rServoMotSpeed = constrain(rServoMotSpeed, -90, 90);
  if (DEBUG_LEVEL >= 2) {
    Serial.print(lServoMotSpeed); Serial.print(", "); Serial.println(rServoMotSpeed);
  }
  MotorSpeed(lServoMot, lServoMotSpeed, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, rServoMotSpeed, SERVO_MOT_R_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int inputSpeed, int rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  inputSpeed = constrain(inputSpeed, -90, 90) * rotateMode;
  Serial.print("inputSpeed "); Serial.print(inputSpeed); Serial.print(", "); 
  int speed = map(inputSpeed, -90, 90, 0, 180);
  Serial.print("speed "); Serial.println(speed);
  if (inputSpeed > 0) speed = map(speed, 90, 180, GEEKSERVO_CW_LEFT_BOARD_PULSE_WIDTH, GEEKSERVO_CW_RIGHT_BOARD_PULSE_WIDTH);
  else if (inputSpeed < 0) speed = map(speed, 0, 90, GEEKSERVO_CCW_LEFT_BOARD_PULSE_WIDTH, GEEKSERVO_CCW_RIGHT_BOARD_PULSE_WIDTH);
  else speed = GEEKSERVO_STEPPING_PULSE;
  servoMot.writeMicroseconds(speed);
  if (DEBUG_LEVEL >= 2) {
    Serial.print("inputServoMotSpeed "); Serial.print(inputSpeed); Serial.print(" ");
    Serial.print("servoMotSpeed "); Serial.println(speed);
  }
}
