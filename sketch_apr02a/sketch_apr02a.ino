// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/
// https://github.com/GyverLibs/EncButton
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
#include <EncButton.h>
#include "TrackingCamI2C.h"

#define DEBUG_LEVEL 1 // Уровень отладки

#define RESET_BTN_PIN 7 // Пин кнопки для мягкого перезапуска
#define LED_PIN 11 // Пин светодиода

#define SERVO_L_PIN 2 // Пин левого серво мотора
#define SERVO_R_PIN 4 // Пин правого серво мотора

#define MAX_MIN_SERVO_COMAND 90 // Максимальное значение скорости вперёд/назад

#define GSERVO_STOP_PULSE 1500 // Значение импулста для остановки мотора, нулевой скорости geekservo
#define GSERVO_L_CW_L_BOARD_PULSE_WIDTH 1595 // Левая граница ширины импульса вравщения по часовой geekservo левого
#define GSERVO_L_CW_R_BOARD_PULSE_WIDTH 2500 // Левая граница ширины импульса вравщения по часовой geekservo левого
#define GSERVO_L_CCW_L_BOARD_PULSE_WIDTH 500 // Левая граница ширины импульса вравщения против часовой geekservo левого
#define GSERVO_L_CCW_R_BOARD_PULSE_WIDTH 1365 // Правая граница ширины импульса вращения против часовой geekservo левого

#define GSERVO_R_CW_L_BOARD_PULSE_WIDTH 1595 // Левая граница ширины импульса вравщения по часовой geekservo правого
#define GSERVO_R_CW_R_BOARD_PULSE_WIDTH 2500 // Правая граница ширины импульса вращения по часовой geekservo правого
#define GSERVO_R_CCW_L_BOARD_PULSE_WIDTH 500 // Левая граница ширины импульса вравщения против часовой geekservo правого
#define GSERVO_R_CCW_R_BOARD_PULSE_WIDTH 1365 // Правая граница ширины импульса вращения против часовой geekservo правого

#define GSERVO_L_DIR_MODE false // Режим реверса вращения левого сервомотора
#define GSERVO_R_DIR_MODE true // Режим реверса вращения правого сервомотора

#define CAM_WIDTH 320 // Ширина кадра от камеры

#define LINE_FOLLOW_SET_POINT CAM_WIDTH / 2 // Значение уставки, к которому линия должна стремиться - это центр кадра

#define CAM_X_CENTER_BORDER_OFFSET 15 // Околоцентральная граница
#define CAM_X_CENTER_L_TRESHOLD (CAM_WIDTH / 2) - CAM_X_CENTER_BORDER_OFFSET // Левое значение центральной границы
#define CAM_X_CENTER_R_TRESHOLD (CAM_WIDTH / 2) + CAM_X_CENTER_BORDER_OFFSET // Правое значение центральной границы

#define CAM_X_EXTREME_BORDER_OFFSET 20 // Значение крайних сегментов
#define CAM_X_EXTREME_L_TRESHOLD CAM_X_EXTREME_BORDER_OFFSET // Левая граница крайнего сегмента
#define CAM_X_EXTREME_R_TRESHOLD CAM_WIDTH - CAM_X_EXTREME_BORDER_OFFSET // Правая граница крайнего сегмента

#define LINE_Y_BOTTOM_START 230 // Значение от которого стоит отмечать, что мы нашли действительно линию

#define MAX_CAM_WAIT_AT_START 6000 // Максимальное время ожидания подключения камеры, это защитный параметр

unsigned long currTime, prevTime, loopTime; // Время
unsigned int delayLineFollowModeSwitch = 0; // Время задержки переключения режима движения по линии по зонам

float Kp_easy = 0.3, Kp_norm = 0.3, Kp_hard = 2.7; // Пропрорциональные коэффиценты, при прямых участках и поворотах
float Ki_easy = 0, Ki_norm = 0, Ki_hard = 0.01; // Интегральные коэффициенты, при прямых участках и поворотах
float Kd_easy = 0.5, Kd_norm = 0.5, Kd_hard = 2; // Дифференциальные коэффициенты, при прямых участках и поворотах
float Kp = Kp_easy, Ki = Ki_easy, Kd = Kd_easy; // Начальные коэффиценты регулятора

TrackingCamI2C trackingCam; // Инициализация объекта камеры
Servo lServoMot, rServoMot; // Инициализация объектов сервомоторов
EncButton<EB_TICK, RESET_BTN_PIN> btn; // Просто кнопка <KEY>
GTimer myTimer(MS, 10), lineFollowModeSwitchTime(MS); // Инициализация объекта таймера
GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора и dt

int speedEasyLine = 70, speedNormalLine = 50, speedHardLine = 35; // Значения скорости на простом, нормальном и сложном участке
int speed = speedEasyLine; // Скорость
byte lineFollowZone = 1;

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println();  
  pinMode(LED_PIN, OUTPUT); // Настраиваем пин светодиода
  lineFollowModeSwitchTime.setMode(TIMER_TIMEOUT); // Настраиваем режим таймера переключения мода движения по линии 
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-180, 180); // Пределы регулятора
  trackingCam.init(51, 400000); // cam_id - 1..127, default 51, speed - 100000/400000, cam enables auto detection of master clock
  while (true) { // Ждём пока камера начнёт работать
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    if (DEBUG_LEVEL >= 2) Serial.println(nBlobs); // Выводим количество найденных blobs
    if (nBlobs == 1 || millis() >= MAX_CAM_WAIT_AT_START) break; // Если она нашла 1 блоб, то выбрасываем из цикла или выбрасываем в том случае, если прошлом максимальное время ожидания
    delay(500); // Задержка между проверками
  }
  digitalWrite(LED_PIN, HIGH); // Включаем светодиод
  Serial.println("Ready... press btn");
  lServoMot.attach(SERVO_L_PIN); rServoMot.attach(SERVO_R_PIN); // Подключение сервомоторов
  // При старте моторы выключаем
  MotorsControl(0, 0);
  //MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE);
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.press()) { // Произошло нажатие
      Serial.println("Go!!!");
      digitalWrite(LED_PIN, LOW); // Выключаем светодиод
      break;
    }
  }
  // Записываем время перед стартом loop
  currTime = millis();
  prevTime = currTime;
}

void loop() {
  CheckBtnClick(); // Вызываем функцию опроса кнопки
  ParseSerialInputValues(); // Парсинг значений из Serial
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
    int lineX = 0, lineY = 0; // Линия по X и Y
    int lineB = 0, lineL = 0, lineR = 0; // Значение пятна линии снизу, слева и справа
    int maxArea = 0; // Максимальня площадь фиругы
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    for(int i = 0; i < nBlobs; i++) { // Печать информации о blobs
      int area = trackingCam.blob[i].area;
      int cx = trackingCam.blob[i].cx;
      int cy = trackingCam.blob[i].cy;
      int bottom = trackingCam.blob[i].bottom;
      int left = trackingCam.blob[i].left;
      int right = trackingCam.blob[i].right;
      if (bottom > LINE_Y_BOTTOM_START) { // Если линия начинается с нижней части картинки камеры
        if (maxArea < area) { // Если площадь текущей фигуры-линии больше других, то выбираем этот объект как линию
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
    int lineArea = maxArea; // Площадь фигуры с линией
    CheckBtnClick(); // Повторно вызываем функцию опроса кнопки
    int error = (nBlobs == 0 ? 0 : lineX - LINE_FOLLOW_SET_POINT); // Нахождение ошибки, если линия не найдена, то значение направления 0, ToDo сделать алгоритм возвращения на линию
    regulator.setpoint = error; // Передаём ошибку регулятору
    if (CAM_X_CENTER_L_TRESHOLD <= lineX && lineX <= CAM_X_CENTER_R_TRESHOLD) { // Центр фигуры линии X в центральной зоне кадра
      if (lineFollowZone != 1) { // Если зона не 1 - центральная
        delayLineFollowModeSwitch = 200;
        lineFollowModeSwitchTime.setTimeout(delayLineFollowModeSwitch);
        Serial.println("Set delayLineFollowModeSwitch: " + String(delayLineFollowModeSwitch));
      }
      if (lineFollowModeSwitchTime.isReady() && lineFollowZone != 1) {
        Serial.println("Set lineFollowZone: " + String(lineFollowZone));
        lineFollowZone = 1; // Установить новое значение зоны
        Kp = Kp_easy;
        Ki = Ki_easy;
        Kd = Kd_easy;
        regulator.integral = 0; // Обнуляем интегральную составляющую
        speed = speedEasyLine;
      }
    } else if (CAM_X_EXTREME_L_TRESHOLD <= lineL && lineR <= CAM_X_EXTREME_R_TRESHOLD) { // Крайние значения фигуры линии попадают зону до экстримальной зоны кадра
      Kp = Kp_norm;
      Ki = Ki_norm;
      Kd = Kd_norm;
      speed = speedNormalLine;
    } else { // Линия за крайней границой слева или справа
      Kp = Kp_hard;
      Ki = Ki_hard;
      Kd = Kd_hard;
      speed = speedHardLine;
    }
    /*
    if (lineL < CAM_X_EXTREME_L_TRESHOLD || lineR > CAM_X_EXTREME_R_TRESHOLD) { // Если линия замечена с края кадра
      Kp = Kp_hard;
      Kd = Kd_hard;
      speed = speedHardLine;
    } else if (CAM_X_CENTER_L_TRESHOLD <= lineX && lineX <= CAM_X_CENTER_R_TRESHOLD) { // Если линия близка к центру
      Kp = Kp_easy;
      Kd = Kd_easy;
      speed = speedStandartLine;
    } else { // Простая линия
      Kp = Kp_easy;
      Kd = Kd_easy;
      regulator.integral = 0; // Обнуляем интегральную составляющую
      speed = speedEasyLine;
    }
    */
    if (regulator.Kp != Kp) regulator.Kp = Kp; // Установка значений Kp, если они были изменены
    if (regulator.Ki != Ki) regulator.Ki = Ki; // Установка значений Ki, если они были изменены
    if (regulator.Kd != Kd) regulator.Kd = Kd; // Установка значений Kd, если они были изменены
    regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    if (DEBUG_LEVEL >= 0) {
      MotorsControl(u, speed); // Для запуска моторов
    }

    // Запустить моторы для проверки
    //MotorSpeed(lServoMot, 90, GSERVO_L_DIR_MODE, GSERVO_L_CW_L_BOARD_PULSE_WIDTH, GSERVO_L_CW_R_BOARD_PULSE_WIDTH, GSERVO_L_CCW_L_BOARD_PULSE_WIDTH, GSERVO_L_CCW_R_BOARD_PULSE_WIDTH);
    //MotorSpeed(rServoMot, 90, GSERVO_R_DIR_MODE, GSERVO_R_CW_L_BOARD_PULSE_WIDTH, GSERVO_R_CW_R_BOARD_PULSE_WIDTH, GSERVO_R_CCW_L_BOARD_PULSE_WIDTH, GSERVO_R_CCW_R_BOARD_PULSE_WIDTH);
    if (DEBUG_LEVEL >= 2) {
      Serial.print("Line: "); // Печаталь информации о выбранной фигуре
      Serial.print(lineX, DEC); Serial.print(" ");
      Serial.print(lineY, DEC); Serial.print(" "); Serial.print(lineB, DEC); Serial.print(" ");
      Serial.print(lineL, DEC); Serial.print(" "); Serial.print(lineR, DEC); Serial.print(" ");
      Serial.print(lineArea, DEC); Serial.println();
    }
    if (DEBUG_LEVEL >= 1 || DEBUG_LEVEL == -1) {
      Serial.print("loopTime: " + String(loopTime) + ", ");
      Serial.print("error: " + String(error) + ", ");
      Serial.println("u: " + String(u));
    }
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  MotorSpeed(lServoMot, lServoMotSpeed, GSERVO_L_DIR_MODE, GSERVO_L_CW_L_BOARD_PULSE_WIDTH, GSERVO_L_CW_R_BOARD_PULSE_WIDTH, GSERVO_L_CCW_L_BOARD_PULSE_WIDTH, GSERVO_L_CCW_R_BOARD_PULSE_WIDTH);
  MotorSpeed(rServoMot, rServoMotSpeed, GSERVO_R_DIR_MODE, GSERVO_R_CW_L_BOARD_PULSE_WIDTH, GSERVO_R_CW_R_BOARD_PULSE_WIDTH, GSERVO_R_CCW_L_BOARD_PULSE_WIDTH, GSERVO_R_CCW_R_BOARD_PULSE_WIDTH);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int inputSpeed, bool rotateMode, int gservoCWLBPulseW, int gservoCWRBPulseW, int gservoCCWLBPulseW, int gservoCCWRBPulseW) {
  // Servo, 0->FW, 90->stop, 180->BW
  if (DEBUG_LEVEL >= 2) {
    Serial.print("inputSpeed: " + String(inputSpeed) + ", ");
  }
  inputSpeed = constrain(inputSpeed, -MAX_MIN_SERVO_COMAND, MAX_MIN_SERVO_COMAND) * (rotateMode? -1 : 1); // Обрезать скорость и установить реверс, если есть такая установка
  int speed = map(inputSpeed, -90, 90, 0, 180); // Изменить диапазон, который понимает серво
  if (DEBUG_LEVEL >= 2) {
    Serial.println("speedConverted: " + String(speed));
  }
  // Перевести в диапазон шим сигнала  
  if (inputSpeed > 0) speed = map(speed, 90, 180, gservoCWLBPulseW, gservoCWRBPulseW); // Скорость, которая больше 0
  else if (inputSpeed < 0) speed = map(speed, 0, 90, gservoCCWLBPulseW, gservoCCWRBPulseW); // Скорость, которая ниже 0
  else speed = GSERVO_STOP_PULSE; // Нулевая скорость
  servoMot.writeMicroseconds(speed); // Установить сервомотору шим сигнал
  if (DEBUG_LEVEL >= 2) {
    Serial.println("outServoSpeed: " + String(speed));
  }
}

// Парсинг значений из Serial
void ParseSerialInputValues() {
  if (Serial.available() > 2) {
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Чистим символы
    inputStr.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = inputStr.length(); // Переменая для хронения индекса вхождения цифры в входной строке, изначально равна размеру строки
    for (byte i = 0; i < 10; i++) { // Поиск первого вхождения цифры от 0 по 9 в подстроку
      byte index = inputStr.indexOf(String(i)); // Узнаём индекс, где нашли цифру параметра цикла
      if (index < strIndex && index != 255) strIndex = index; // Если индекс цифры меньше strIndex, то обновляем strIndex 
    }
    String key = inputStr.substring(0, strIndex); // Записываем ключ с начала строки до первой цицры
    float value = inputStr.substring(strIndex, inputStr.length()).toFloat(); // Записываем значение с начала цифры до конца строки
    if (key == "pe") {
      Kp_easy = value;
      regulator.Kp = Kp_easy;
    } else if (key == "pn") {
      Kp_norm = value;
      regulator.Kp = Kp_norm;
    } else if (key == "ph") {
      Kp_hard = value;
      regulator.Kp = Kp_hard;
    } else if (key == "ie") {
      Ki_easy = value;
      regulator.Ki = Ki_easy;
      regulator.integral = 0;
    } else if (key == "in") {
      Ki_norm = value;
      regulator.Ki = Ki_norm;
      regulator.integral = 0;
    } else if (key == "ih") {
      Ki_hard = value;
      regulator.Ki = Ki_hard;
      regulator.integral = 0;
    } else if (key == "de") {
      Kd_easy = value;
      regulator.Kd = Kd_easy;
    } else if (key == "dn") {
      Kd_norm = value;
      regulator.Kd = Kd_norm;
    } else if (key == "dh") {
      Kd_hard = value;
      regulator.Kd = Kd_hard;
    } else if (key == "se") {
      speedEasyLine = value;
    } else if (key == "sn") {
      speedNormalLine = value;
    } else if (key == "sh") {
      speedHardLine = value;
    }
    if (DEBUG_LEVEL >= 1) { // Печать информации о ключе и значении
      Serial.println(key + " = " + String(value));
    }
  }
}

// Функция опроса о нажатии кнопки
void CheckBtnClick() {
  btn.tick(); // Опрашиваем кнопку в первый раз
  if (btn.press()) { // Произошло нажатие
    Serial.println("Btn press and reset");
    delay(50); // Нужна задержка иначе не выведет сообщение
    softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  }
}
