#include <Servo.h>
#include <NewPing.h>

// Пины для ультразвуковых датчиков
#define TRIG_PIN_FRONT 12
#define ECHO_PIN_FRONT 11
#define TRIG_PIN_LEFT 10
#define ECHO_PIN_LEFT 9
#define TRIG_PIN_RIGHT 8
#define ECHO_PIN_RIGHT 7

// Пины для двигателей (исправлена нумерация)
#define MOTOR_A_EN 5
#define MOTOR_A_IN1 4
#define MOTOR_A_IN2 3
#define MOTOR_B_EN 6
#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 1

// Пины для сервопривода и датчиков
#define SERVO_PIN 13
#define PIR_SENSOR A0
#define IR_SENSOR A1
#define BUZZER_PIN A2
#define LED_PIN A3

// Максимальное расстояние для датчиков (см)
#define MAX_DISTANCE 200

// Создание объектов для датчиков
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

Servo headServo;

// Структура для хранения данных сенсоров
struct SensorData {
  int frontDist;
  int leftDist;
  int rightDist;
  int pirValue;
  int irValue;
  unsigned long timestamp;
};

// Переменные состояния
int currentAngle = 90;
int searchDirection = 1;
unsigned long lastDetectionTime = 0;
unsigned long lastScanTime = 0;
unsigned long lastMoveTime = 0;
bool personDetected = false;
int detectionCount = 0;
int searchPhase = 0;

// Пороговые значения (настроены)
const int PERSON_DISTANCE = 100;    // см
const int CLOSE_DISTANCE = 50;      // см для близкого обнаружения
const int OBSTACLE_DISTANCE = 25;   // см (уменьшено для безопасности)
const int HEAT_THRESHOLD = 500;     // значение ИК-датчика
const unsigned long SCAN_INTERVAL = 200;  // мс
const unsigned long MOVE_INTERVAL = 300;  // мс

// Фильтр Калмана для датчиков расстояния
class SimpleKalman {
  private:
    float Q = 0.1;   // шум процесса
    float R = 0.1;   // шум измерения
    float P = 1.0;   // ошибка оценки
    float X = 0.0;   // оценка
  public:
    SimpleKalman(float processNoise = 0.1, float sensorNoise = 0.1, float estimatedError = 1.0) {
      Q = processNoise;
      R = sensorNoise;
      P = estimatedError;
    }
    
    float update(float measurement) {
      // Прогноз
      P = P + Q;
      // Коррекция
      float K = P / (P + R);
      X = X + K * (measurement - X);
      P = (1 - K) * P;
      return X;
    }
};

SimpleKalman kalmanFront(0.1, 0.1, 1.0);
SimpleKalman kalmanLeft(0.1, 0.1, 1.0);
SimpleKalman kalmanRight(0.1, 0.1, 1.0);

void setup() {
  Serial.begin(115200); 
  Serial.println("=== Робот-поисковик УЛУЧШЕННЫЙ запущен! ===");
  
  // Настройка пинов двигателей
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  // Настройка пинов датчиков и сигналов
  pinMode(PIR_SENSOR, INPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Настройка сервопривода
  headServo.attach(SERVO_PIN);
  headServo.write(currentAngle);
  
  // Калибровка датчиков
  calibrateSensors();
  
  // Тестовый сигнал
  startupSignal();
  
  Serial.println("Калибровка завершена. Начинаю поиск...");
}

void calibrateSensors() {
  Serial.println("Калибровка датчиков...");
  for(int i = 0; i < 10; i++) {
    getFilteredDistance(sonarFront); // Прогрев фильтра
    delay(50);
  }
}

void startupSignal() {
  // Улучшенный сигнал запуска
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000 + i*200, 300);
    delay(400);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Сканирование окружающей среды с фиксированным интервалом
  if (currentTime - lastScanTime >= SCAN_INTERVAL) {
    scanEnvironment();
    lastScanTime = currentTime;
  }
  
  // Управление движением с фиксированным интервалом
  if (currentTime - lastMoveTime >= MOVE_INTERVAL) {
    if (personDetected) {
      pursuitMode();
    } else {
      searchMode();
    }
    lastMoveTime = currentTime;
  }
  
  // Отладочная информация
  if (currentTime % 2000 == 0) {
    printDebugInfo();
  }
}

void scanEnvironment() {
  // Плавное движение головой для сканирования
  currentAngle += (15 * searchDirection);
  if (currentAngle >= 160 || currentAngle <= 20) {
    searchDirection *= -1;
  }
  headServo.write(currentAngle);
  
  // Чтение и фильтрация данных с датчиков
  SensorData sensors = readSensors();
  
  // Улучшенная логика обнаружения
  bool ultrasonicDetection = 
    (sensors.frontDist < PERSON_DISTANCE && sensors.frontDist > 5) ||
    (sensors.leftDist < PERSON_DISTANCE && sensors.leftDist > 5) ||
    (sensors.rightDist < PERSON_DISTANCE && sensors.rightDist > 5);
  
  bool sensorDetection = (sensors.pirValue == HIGH) || (sensors.irValue > HEAT_THRESHOLD);
  
  // Взвешенное обнаружение с приоритетами
  int detectionScore = 0;
  if (ultrasonicDetection) detectionScore += 2;
  if (sensorDetection) detectionScore += 1;
  if (sensors.frontDist < CLOSE_DISTANCE) detectionScore += 1;
  
  if (detectionScore >= 3) { // Повышенный порог для надежности
    detectionCount++;
    if (detectionCount >= 3) {
      if (!personDetected) {
        personDetected = true;
        lastDetectionTime = millis();
        triggerAlarm();
        Serial.println("🚨 ЧЕЛОВЕК ОБНАРУЖЕН! Активация режима преследования");
      }
    }
  } else {
    detectionCount = max(0, detectionCount - 1);
  }
  
  // Сброс обнаружения через 15 секунд
  if (personDetected && (millis() - lastDetectionTime > 15000)) {
    personDetected = false;
    Serial.println("🔍 Поиск возобновлен");
    stopMotors();
  }
}

SensorData readSensors() {
  SensorData data;
  data.frontDist = getFilteredDistance(sonarFront);
  data.leftDist = getFilteredDistance(sonarLeft);
  data.rightDist = getFilteredDistance(sonarRight);
  data.pirValue = digitalRead(PIR_SENSOR);
  data.irValue = analogRead(IR_SENSOR);
  data.timestamp = millis();
  return data;
}

int getFilteredDistance(NewPing &sonar) {
  unsigned int distance = sonar.ping_median(3); // Уменьшено количество измерений
  int cm = sonar.convert_cm(distance);
  if (cm <= 0 || cm > MAX_DISTANCE) return MAX_DISTANCE;
  
  // Применение фильтра Калмана
  if (&sonar == &sonarFront) return (int)kalmanFront.update(cm);
  if (&sonar == &sonarLeft) return (int)kalmanLeft.update(cm);
  if (&sonar == &sonarRight) return (int)kalmanRight.update(cm);
  return cm;
}

void searchMode() {
  SensorData sensors = readSensors();
  
  // Улучшенное избегание препятствий
  if (sensors.frontDist < OBSTACLE_DISTANCE && sensors.frontDist > 0) {
    avoidObstacle(sensors);
  } else {
    // Умный паттерн поиска
    switch (searchPhase) {
      case 0: // Движение вперед
        moveForward(180);
        if (random(0, 100) > 80) searchPhase = 1;
        break;
      case 1: // Плавный поворот
        if (sensors.leftDist > sensors.rightDist) {
          smoothTurn(-120, 1000);
        } else {
          smoothTurn(120, 1000);
        }
        searchPhase = 0;
        break;
    }
  }
}

void pursuitMode() {
  SensorData sensors = readSensors();
  
  // Индикация преследования
  digitalWrite(LED_PIN, (millis() % 400) < 200);
  if (millis() % 1000 < 100) {
    tone(BUZZER_PIN, 1600, 100);
  }
  
  // Улучшенная логика преследования
  if (sensors.frontDist < PERSON_DISTANCE && sensors.frontDist > OBSTACLE_DISTANCE + 10) {
    // Преследование по прямой
    int speed = map(sensors.frontDist, OBSTACLE_DISTANCE + 10, PERSON_DISTANCE, 100, 255);
    speed = constrain(speed, 100, 255);
    moveForward(speed);
  } else if (sensors.leftDist < sensors.rightDist && sensors.leftDist > OBSTACLE_DISTANCE) {
    // Плавный поворот налево
    smoothTurn(-150, 300);
  } else if (sensors.rightDist > OBSTACLE_DISTANCE) {
    // Плавный поворот направо
    smoothTurn(150, 300);
  } else {
    // Поиск при потере цели
    advancedSearchPattern();
  }
}

void avoidObstacle(SensorData sensors) {
  Serial.println("🛑 Избегание препятствия");
  
  stopMotors();
  delay(300);
  
  // Выбор направления с учетом истории
  if (sensors.leftDist - sensors.rightDist > 20) {
    smoothTurn(-180, 600);
  } else if (sensors.rightDist - sensors.leftDist > 20) {
    smoothTurn(180, 600);
  } else {
    // Отъезд назад при равных расстояниях
    moveBackward(150, 500);
    smoothTurn(200, 800);
  }
}

void smoothTurn(int speed, int duration) {
  if (speed < 0) {
    turnLeft(abs(speed));
  } else {
    turnRight(speed);
  }
  delay(duration);
  stopMotors();
  delay(100);
}

void advancedSearchPattern() {
  Serial.println("🌀 Активный поиск цели");
  
  // Спиральный поиск
  for (int i = 0; i < 4; i++) {
    smoothTurn(150, 400 + i*100);
    stopMotors();
    delay(300);
    scanEnvironment();
    
    smoothTurn(-150, 200);
    stopMotors();
    delay(300);
    scanEnvironment();
  }
}

void triggerAlarm() {
  Serial.println("🚨 АКТИВАЦИЯ СИГНАЛА ТРЕВОГИ!");
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1800, 400);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    delay(200);
  }
}

void moveBackward(int speed, int duration) {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_A_EN, speed);
  analogWrite(MOTOR_B_EN, speed);
  delay(duration);
  stopMotors();
}

// Функции управления двигателями (оставлены без изменений)
void moveForward(int speed) {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_EN, speed);
  analogWrite(MOTOR_B_EN, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_EN, speed);
  analogWrite(MOTOR_B_EN, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_A_EN, speed);
  analogWrite(MOTOR_B_EN, speed);
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);
}

void printDebugInfo() {
  SensorData sensors = readSensors();
  Serial.println("=== ДЕБАГ ИНФОРМАЦИЯ ===");
  Serial.print("Режим: ");
  Serial.println(personDetected ? "ПРЕСЛЕДОВАНИЕ" : "ПОИСК");
  Serial.print("Дистанции: F=");
  Serial.print(sensors.frontDist);
  Serial.print(" L=");
  Serial.print(sensors.leftDist);
  Serial.print(" R=");
  Serial.println(sensors.rightDist);
  Serial.print("PIR: ");
  Serial.print(sensors.pirValue);
  Serial.print(" IR: ");
  Serial.println(sensors.irValue);
  Serial.print("Угол сервы: ");
  Serial.println(currentAngle);
  Serial.println("======================");
}
