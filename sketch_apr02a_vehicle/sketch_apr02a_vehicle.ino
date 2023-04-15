// https://www.youtube.com/watch?v=jW3cY4Irs50
// https://alexgyver.ru/gyverpid/
// https://github.com/GyverLibs/TimerMs
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

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include <TimerMs.h>
#include <EncButton.h>
#include "TrackingCamI2C.h"

#define ON_GSERVO_CONTROL true // Включить управление серво
#define ON_GSERVO_FOR_TEST false // Включить серво для тертирования, ON_GSERVO_CONTROL должно быть false

#define SWITCH_ZONE_MODE_DEBUG true // Отладка обработки алгоритма о сменах зоны
#define PRINT_START_BLOB_N_FROM_CAM_DEBUG false // Отладка информации с камеры при старте
#define PRINT_FROM_CAM_DEBUG false // Отладка информации с камеры
#define PRINT_LINE_SEN_RAW_VAL_DEBUG false // Печать информации о сырых значений с датчиков линии
#define PRINT_LINE_SEN_VAL_DEBUG false // Печать информации о логических значений с датчиков линии
#define PRINT_INFO_ABOUT_OBJ_DEBUG false // Отладка информации о выбранном объекте в качестве линии
#define PRINT_DT_ERR_U_DEBUG true // Печать информации о loopTime, error, u

#define MOTORS_CONTROL_FUNC_DEBUG false // Отдалка функции MotorsControl
#define MOTOR_SPEED_FUNC_DEBUG false // Отдалка функции MotorsControl

#define RESET_BTN_PIN 8 // Пин кнопки для мягкого перезапуска
#define LED_PIN 11 // Пин светодиода

#define SERVO_L_PIN 9 // Пин левого серво мотора
#define SERVO_R_PIN 10 // Пин правого серво мотора

#define L_LINE_SEN_PIN A0 // Пин левого датчика линии
#define R_LINE_SEN_PIN A1 // Пин левого датчика линии

#define REF_RAW_BLACK_LS 600 // Сырое значение чёрного левого датчика линии
#define REF_RAW_WHITE_LS 27 // Сырое значение белого левого датчика линии
#define REF_RAW_BLACK_RS 600 // Сырое значение чёрного правого датчика линии
#define REF_RAW_WHITE_RS 27 // Сырое значение белого правого датчика линии

#define REF_LS_TRESHOLD 30 // Пороговое значение для определения чёрного

#define DELAY_LINE_FOLLOW_LIGHT_MODE 300 // Время для подтверждения лёгкой линии
#define DELAY_LINE_FOLLOW_HARD_MODE 150 // Время для подтверждения сложной линии линии

#define MAX_MIN_SERVO_COMMAND 100 // Максимальное значение скорости вперёд/назад серво

#define GSERVO_STOP_PWM 1500 // Значение импулста для остановки мотора, нулевой скорости geekservo
#define GSERVO_L_CW_L_BOARD_PWM 1591 // Левая граница ширины импульса вравщения по часовой geekservo левого
#define GSERVO_L_CW_R_BOARD_PWM 2500 // Левая граница ширины импульса вравщения по часовой geekservo левого
#define GSERVO_L_CCW_L_BOARD_PWM 500 // Левая граница ширины импульса вравщения против часовой geekservo левого
#define GSERVO_L_CCW_R_BOARD_PWM 1377 // Правая граница ширины импульса вращения против часовой geekservo левого

#define GSERVO_R_CW_L_BOARD_PWM 1589 // Левая граница ширины импульса вравщения по часовой geekservo правого
#define GSERVO_R_CW_R_BOARD_PWM 2500 // Правая граница ширины импульса вращения по часовой geekservo правого
#define GSERVO_R_CCW_L_BOARD_PWM 500 // Левая граница ширины импульса вравщения против часовой geekservo правого
#define GSERVO_R_CCW_R_BOARD_PWM 1376 // Правая граница ширины импульса вращения против часовой geekservo правого

#define GSERVO_L_DIR_MODE false // Режим реверса вращения левого сервомотора
#define GSERVO_R_DIR_MODE true // Режим реверса вращения правого сервомотора

#define CAM_WIDTH 310 // Ширина кадра от камеры
#define CAM_X_CENTER_BORDER_OFFSET 80 // Околоцентральная граница
#define CAM_X_CENTER_L_TRESHOLD (CAM_WIDTH / 2) - CAM_X_CENTER_BORDER_OFFSET // Левое значение центральной границы
#define CAM_X_CENTER_R_TRESHOLD (CAM_WIDTH / 2) + CAM_X_CENTER_BORDER_OFFSET // Правое значение центральной границы

#define LINE_FOLLOW_SET_POINT CAM_WIDTH / 2 // Значение уставки, к которому линия должна стремиться - это центр кадра
#define LINE_AREA_MIN 800 // Минимальное значение для определения блоба линии
#define LINE_Y_BOTTOM_START 230 // Значение начала блоба снизу от которого стоит отмечать, что мы нашли действительно линию

#define MAX_CAM_WAIT_AT_START 7000 // Максимальное время ожидания подключения камеры, это защитный параметр

unsigned long currTime, prevTime, loopTime; // Объявление переменных времени

float Kp_easy = 0.3, Kp_hard = 0.5; // Пропрорциональные коэффиценты, при прямых участках и поворотах
float Ki_easy = 0, Ki_hard = 0.01; // Интегральные коэффициенты, при прямых участках и поворотах
float Kd_easy = 1, Kd_hard = 2; // Дифференциальные коэффициенты, при прямых участках и поворотах

TrackingCamI2C trackingCam; // Инициализация объекта камеры
Servo lServoMot, rServoMot; // Инициализация объектов сервомоторов
EncButton<EB_TICK, RESET_BTN_PIN> btn; // Инициализация объекта простой кнопки
TimerMs regulatorTmr(10); // Инициализация объекта таймера цикла регулирования
TimerMs lineFollowEasyModeSwitchTmr, lineFollowHardModeSwitchTmr; // Инициализация объектов таймеров
GyverPID regulator(Kp_easy, Ki_easy, Kd_easy, 10); // Инициализируем коэффициенты регулятора и dt

int speedEasyLine = 80, speedHardLine = 35, speedReturnToLine = 35; // Переменная для хренения хначения скорости на простом, сложном участке и при слёте с линии
int speed = speedEasyLine; // Переменная для хранения скорости
int error = 0; // Переменная для хренения ошибки регулирования
float u = 0; // Переменная для хранения управляющего воздействия с регулятора
int lineFollowZone = 0; // Зона, в которой робот движется, изначально по центру

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println();
  pinMode(LED_PIN, OUTPUT); // Настраиваем пин светодиода
  pinMode(L_LINE_SEN_PIN, INPUT); // Настраиваем пин левого датчика линии
  pinMode(R_LINE_SEN_PIN, INPUT); // Настраиваем пин правого датчика линии
  regulatorTmr.setPeriodMode(); // Настроем режим условия регулирования на период
  lineFollowEasyModeSwitchTmr.setTimerMode(); // Настраиваем режим таймера переключения мода движения по линии
  lineFollowHardModeSwitchTmr.setTimerMode(); // Настраиваем режим таймера переключения мода движения по линии
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-180, 180); // Пределы регулятора
  trackingCam.init(51, 400000); // cam_id - 1..127, default 51, speed - 100000/400000, cam enables auto detection of master clock
  while (true) { // Ждём пока камера начнёт работать
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты
    if (PRINT_START_BLOB_N_FROM_CAM_DEBUG) Serial.println(nBlobs); // Выводим количество найденных блобах
    int maxArea = 0, bottom = 0; // Переменные для записи максимального размера и значения, где начинается линия по Y
    for(int i = 0; i < nBlobs; i++) { // Печать информации о блобах
      int area = trackingCam.blob[i].area;
      maxArea = max(maxArea, area); // Найти самый большой блоб по размеру
      bottom = trackingCam.blob[i].bottom;
    }
    // Если нашли блоб, который больше, чем минимальное значение и нижняя граница её по Y начинается с низу кадра, то выбрасываем из цикла, или выбрасываем или по максимальному времени
    if (maxArea > LINE_AREA_MIN && bottom > LINE_Y_BOTTOM_START || millis() >= MAX_CAM_WAIT_AT_START) break;
    delay(100); // Задержка между проверками
  }
  digitalWrite(LED_PIN, HIGH); // Включаем светодиод
  Serial.println("Ready... press btn");
  lServoMot.attach(SERVO_L_PIN); rServoMot.attach(SERVO_R_PIN); // Подключение сервомоторов
  ChassisControl(0, 0); // При старте моторы выключаем
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.press()) { // Произошло нажатие
      Serial.println("Go!!!");
      digitalWrite(LED_PIN, LOW); // Выключаем светодиод
      break;
    }
  }
  regulatorTmr.start(); // Запускаем таймер цикла регулирования
  currTime = millis(); // Записываем время перед стартом loop
  prevTime = currTime;
}

void loop() {
  regulatorTmr.tick(); // Обработка таймера регулирования
  CheckBtnClick(); // Вызываем функцию опроса с кнопки 
  ParseSerialInputValues(); // Парсинг значений из Serial

  if (regulatorTmr.ready()) { // Раз в 10 мсек выполнять
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
    int lineX = 0, lineY = 0; // Инициализация переменной для значения пятна линии по X и Y
    int lineB = 0, lineL = 0, lineR = 0; // Инициализация переменной для значения пятна линии снизу, слева и справа
    int lineArea = LINE_AREA_MIN; // Инициализации переменной для площади пятна линии, которая изначально будет равна LINE_AREA_MIN
    uint8_t nBlobs = trackingCam.readBlobs(); // Считать найденные объекты с камеры
    for(int i = 0; i < nBlobs; i++) { // Обработать все объекты
      int area = trackingCam.blob[i].area;
      int cx = trackingCam.blob[i].cx;
      int cy = trackingCam.blob[i].cy;
      int bottom = trackingCam.blob[i].bottom;
      int left = trackingCam.blob[i].left;
      int right = trackingCam.blob[i].right;
      // Если линия начинается с нижней части картинки камеры и если площадь текущего блоба больше других (но учитывая минимальные LINE_AREA_MIN), то выбираем этот объект как линию
      if (bottom > LINE_Y_BOTTOM_START && area > lineArea) {
        lineArea = area; // Записываем новое значение площади блоба (линии)
        lineX = cx; lineY = cy; // Записываем cx, cy этого блоба (линии)
        lineB = bottom; // Записываем откуда началась блоб (линия) снизу
        lineL = left; lineR = right; // Записываем координаты слева и справа блоба (линии)
      }
      if (PRINT_FROM_CAM_DEBUG) { // Печать информации о фигуре
        Serial.print(cx, DEC); Serial.print("\t"); Serial.print(cy, DEC); Serial.print("\t");
        Serial.print(bottom, DEC); Serial.print("\t");
        Serial.print(left, DEC); Serial.print("\t"); Serial.print(right, DEC); Serial.print("\t");
        Serial.print(area, DEC); Serial.println();
      }
    }
    
    CheckBtnClick(); // Повторно вызываем функцию опроса с кнопки

    int lLineSenRawVal = analogRead(L_LINE_SEN_PIN); // Сырое значение отражения с левого датчика линии
    int rLineSenRawVal = analogRead(R_LINE_SEN_PIN); // Сырое значение отражения с правого датчика линии
    int lLineSenVal = constrain(map(lLineSenRawVal, REF_RAW_BLACK_LS, REF_RAW_WHITE_LS, 0, 100), 0, 100); // Обработаные значения отражения левого датчика линии
    int rLineSenVal = constrain(map(rLineSenRawVal, REF_RAW_BLACK_RS, REF_RAW_WHITE_RS, 0, 100), 0, 100); // Обработаные значения отражения правого датчика линии
    bool lLineSen = (lLineSenVal < REF_LS_TRESHOLD ? true : false); // Логическое значение о переходе с белого на чёрное левого датчика линии
    bool rLineSen = (rLineSenVal < REF_LS_TRESHOLD ? true : false); // Логическое значение о переходе с белого на чёрное правого датчика линии
    if (PRINT_LINE_SEN_RAW_VAL_DEBUG) Serial.println("lLineSenRawVal: " + String(lLineSenRawVal) + "\t" + "rLineSenRawVal: " + String(rLineSenRawVal));
    if (PRINT_LINE_SEN_VAL_DEBUG) Serial.println("lLineSen: " + String(lLineSen) + "\t" + "rLineSen: " + String(rLineSen));

    // Проверка на датчики защиты слёта с линии
    if (lLineSen == true && rLineSen == false) { // Если левый датчик среагировал на линию, а правый нет
      if (abs(lineFollowZone) != 2) { // Только, если до этого значение зоны было другим, тогда обновляем зону
        if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Flew off the line side LEFT");
        SetZoneParam(Kp_hard, Ki_hard, Kd_hard, true, speedReturnToLine); // Установить новые значения параметров зоны
        lineFollowZone = -2; // Установить новое значение зоны слева
      }
    } else if (lLineSen == false && rLineSen == true) { // Если правый датчик среагировал на линию, а левый нет
      if (abs(lineFollowZone) != 2) { // Один раз обновить значения
        if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Flew off the line side RIGHT");
        SetZoneParam(Kp_hard, Ki_hard, Kd_hard, true, speedReturnToLine); // Установить новые значения параметров зоны
        lineFollowZone = 2; // Установить новое значение зоны справа
      }
    }
    
    // Иначе, если зона не равна -2 или 2, то едем по камере
    if (abs(lineFollowZone) < 2) {
      error = lineX - LINE_FOLLOW_SET_POINT; // Нахождение ошибки
      regulator.setpoint = error; // Передаём ошибку регулятору
    } else if (abs(lineFollowZone) == 2) { // Если сейчас зона равна -2 или 2 и робот в поиске линии...
      if (lineX != 0) { // Проверяем, что камера нашла линию
        if (lineFollowZone == -2) lineFollowZone = -1; // Если слёт был слева, тогда устанавливаем зону -1
        else if (lineFollowZone == 2) lineFollowZone = 1; // Если слёт был справа, тогда устанавливаем зону 1
      }
    }

    // Проверка в какой зоне находится линия
    if (abs(lineFollowZone) < 2) { // Если не крайние значения зоны, когда был зафиксирован слёт
      if (CAM_X_CENTER_L_TRESHOLD <= lineX && lineX <= CAM_X_CENTER_R_TRESHOLD) { // Центр фигуры линии в центральной зоне 0
        if (lineFollowZone != 0 && !lineFollowEasyModeSwitchTmr.active()) { // Если текущая зона не 0, т.е. не центральная и таймер не активен, то выполняем один раз
          lineFollowEasyModeSwitchTmr.setTime(DELAY_LINE_FOLLOW_LIGHT_MODE); // Время для подтверждения, что камера видит простую зону
          lineFollowEasyModeSwitchTmr.start(); // Запустить таймер
          if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Set delayLineFollowModeSwitch: " + String(DELAY_LINE_FOLLOW_LIGHT_MODE));
        }
        if (lineFollowEasyModeSwitchTmr.elapsed() && lineFollowZone != 0) { // Время вышло, подтверждаем центральную зону 0 ...
          lineFollowZone = 0; // Установить новое значение зоны - центр
          if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Confirm set lineFollowZone EASY (0)");
          SetZoneParam(Kp_easy, Ki_easy, Kd_easy, true, speedEasyLine); // Установить новые значения параметров зоны
        }
      } else { // Линия за центральной границой слева или справа
        if (abs(lineFollowZone) != 1 && !lineFollowHardModeSwitchTmr.active()) { // Если текущая зона не -1 или 1, т.е. не центральная и таймер не активен, то выполняем один раз
          lineFollowHardModeSwitchTmr.setTime(DELAY_LINE_FOLLOW_HARD_MODE); // Записать время для подтверждения, что камера видит сложную зону
          lineFollowHardModeSwitchTmr.start(); // Запустить таймер
          if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Set delayLineFollowModeSwitch: " + String(DELAY_LINE_FOLLOW_HARD_MODE));
        }
        if (lineFollowHardModeSwitchTmr.elapsed() && abs(lineFollowZone) != 1) { // Время вышло, подтверждаем зону с крайней левой/правой стороны -1 или 1
          if (lineX >= LINE_FOLLOW_SET_POINT) lineFollowZone = 1; // Установить новое значение зоны справа
          else lineFollowZone = -1; // Иначе зона слева
          if (SWITCH_ZONE_MODE_DEBUG) Serial.println("Confirm set lineFollowZone HARD (" + String(lineFollowZone) + ")");
          SetZoneParam(Kp_hard, Ki_hard, Kd_hard, true, speedHardLine); // Установить новые значения параметров зоны
        }
      }
      regulator.setDt(loopTime != 0 ? loopTime : 10); // Установка dt для регулятора
      u = regulator.getResult(); // Управляющее воздействие с регулятора
      if (ON_GSERVO_CONTROL) ChassisControl(u, speed); // Для управления моторами регулятором
    } else if (lineFollowZone == -2) { // Если линия была потеряна слева
      if (ON_GSERVO_CONTROL) ChassisControl(-100, speedReturnToLine); // Для управления моторами
    } else if (lineFollowZone == 2) { // Если линия была потеряна справа
      if (ON_GSERVO_CONTROL) ChassisControl(100, speedReturnToLine); // Для управления моторами      
    }

    // Запустить моторы для проверки
    if (ON_GSERVO_FOR_TEST) ChassisControl(0, 0);
    
    // Печаталь информации о выбранной фигуре
    if (PRINT_INFO_ABOUT_OBJ_DEBUG) {
      Serial.print("Line: ");
      Serial.print(lineX, DEC); Serial.print("\t"); Serial.print(lineY, DEC); Serial.print("\t");
      Serial.print(lineB, DEC); Serial.print("\t");
      Serial.print(lineL, DEC); Serial.print("\t"); Serial.print(lineR, DEC); Serial.print("\t");
      Serial.print(lineArea, DEC); Serial.println();
    }
    // Для отладки основной информации о регулировании
    if (PRINT_DT_ERR_U_DEBUG) {
      Serial.print("lineFollowZone: " + String(lineFollowZone) + "\t");
      Serial.print("loopTime: " + String(loopTime) + "\t");
      Serial.print("error: " + String(error) + "\t");
      Serial.println("u: " + String(u));
    }
  }
}

// Установка новых параметров езды по линии
void SetZoneParam(float newKp, float float, float newKd, bool resetRegIntegral, int newSpeed) {
  regulator.Kp = newKp;
  regulator.Ki = newKi;
  regulator.Kd = newKd;
  if (resetRegIntegral) regulator.integral = 0; // Обнуляем интегральную составляющую, если требуется
  speed = newSpeed; // Установить новую скорость  
}

// Управление двумя моторами
void ChassisControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  if (MOTORS_CONTROL_FUNC_DEBUG) Serial.print("lServoMot ->\t");  
  MotorSpeed(lServoMot, lServoMotSpeed, GSERVO_L_DIR_MODE, GSERVO_L_CW_L_BOARD_PWM, GSERVO_L_CW_R_BOARD_PWM, GSERVO_L_CCW_L_BOARD_PWM, GSERVO_L_CCW_R_BOARD_PWM);
  if (MOTORS_CONTROL_FUNC_DEBUG) Serial.print("rServoMot ->\t");  
  MotorSpeed(rServoMot, rServoMotSpeed, GSERVO_R_DIR_MODE, GSERVO_R_CW_L_BOARD_PWM, GSERVO_R_CW_R_BOARD_PWM, GSERVO_R_CCW_L_BOARD_PWM, GSERVO_R_CCW_R_BOARD_PWM);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int inputSpeed, bool rotateMode, int gservoCWLBoardPWM, int gservoCWRBoardPWM, int gservoCCWLBoardPWM, int gservoCCWRBoardPWM) {
  if (MOTOR_SPEED_FUNC_DEBUG) Serial.print("inputSpeed: " + String(inputSpeed) + "\t\t");
  inputSpeed = constrain(inputSpeed, -MAX_MIN_SERVO_COMMAND, MAX_MIN_SERVO_COMMAND) * (rotateMode? -1 : 1); // Обрезать скорость и установить реверс, если есть такая установка
  if (MOTOR_SPEED_FUNC_DEBUG) Serial.print("inputSpeedProcessed " + String(inputSpeed) + "\t\t");
  int speed = 0; // Инициализируем переменную, которую передадим сервоприводу
  // Перевести в диапазон шим сигнала
  if (inputSpeed > 0) speed = map(inputSpeed, 0, MAX_MIN_SERVO_COMMAND, gservoCWLBoardPWM, gservoCWRBoardPWM); // Скорость, которая больше 0
  else if (inputSpeed < 0) speed = map(inputSpeed, -MAX_MIN_SERVO_COMMAND, 0, gservoCCWLBoardPWM, gservoCCWRBoardPWM); // Скорость, которая ниже 0
  else speed = GSERVO_STOP_PWM; // Нулевая скорость
  servoMot.writeMicroseconds(speed); // Установить сервомотору шим сигнал
  if (MOTOR_SPEED_FUNC_DEBUG) Serial.println("speedConverted: " + String(speed));
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
    } else if (key == "ph") {
      Kp_hard = value;
      regulator.Kp = Kp_hard;
    } else if (key == "ie") {
      Ki_easy = value;
      regulator.Ki = Ki_easy;
      regulator.integral = 0;
    } else if (key == "ih") {
      Ki_hard = value;
      regulator.Ki = Ki_hard;
      regulator.integral = 0;
    } else if (key == "de") {
      Kd_easy = value;
      regulator.Kd = Kd_easy;
    } else if (key == "dh") {
      Kd_hard = value;
      regulator.Kd = Kd_hard;
    } else if (key == "se") {
      speedEasyLine = value;
    } else if (key == "sh") {
      speedHardLine = value;
    } else if (key == "sr") {
      speedReturnToLine = value;
    }
    Serial.println(key + " = " + String(value)); // Печать информации о ключе и значении
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