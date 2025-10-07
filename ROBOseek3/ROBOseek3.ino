// Робот-поисковик людей на Arduino
#include <Servo.h>
#include <NewPing.h>

// Пины для ультразвуковых датчиков
#define TRIG_PIN_FRONT 12
#define ECHO_PIN_FRONT 11
#define TRIG_PIN_LEFT 10
#define ECHO_PIN_LEFT 9
#define TRIG_PIN_RIGHT 8
#define ECHO_PIN_RIGHT 7

// Пины для двигателей
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

// Переменные состояния
int currentAngle = 90;
int searchDirection = 1;
unsigned long lastDetectionTime = 0;
bool personDetected = false;
int detectionCount = 0;

// Пороговые значения
const int PERSON_DISTANCE = 100;  // см
const int OBSTACLE_DISTANCE = 30; // см
const int HEAT_THRESHOLD = 500;   // значение ИК-датчика

void setup() {
  Serial.begin(9600);
  Serial.println("Робот-поисковик запущен!");
  
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
  
  // Тестовый сигнал
  startupSignal();
}

void startupSignal() {
  // Сигнал запуска
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000, 200);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  // Сканирование окружающей среды
  scanEnvironment();
  
  if (personDetected) {
    // Режим преследования
    pursuitMode();
  } else {
    // Режим поиска
    searchMode();
  }
  
  delay(100);
}

void scanEnvironment() {
  // Движение головой для сканирования
  currentAngle += (10 * searchDirection);
  headServo.write(currentAngle);
  
  if (currentAngle >= 180 || currentAngle <= 0) {
    searchDirection *= -1;
  }
  
  // Чтение данных с датчиков
  int frontDist = getFilteredDistance(sonarFront);
  int leftDist = getFilteredDistance(sonarLeft);
  int rightDist = getFilteredDistance(sonarRight);
  int pirValue = digitalRead(PIR_SENSOR);
  int irValue = analogRead(IR_SENSOR);
  
  // Проверка обнаружения человека
  bool ultrasonicDetection = (frontDist < PERSON_DISTANCE && frontDist > 0) ||
                            (leftDist < PERSON_DISTANCE && leftDist > 0) ||
                            (rightDist < PERSON_DISTANCE && rightDist > 0);
  
  bool sensorDetection = (pirValue == HIGH) || (irValue > HEAT_THRESHOLD);
  
  if (ultrasonicDetection && sensorDetection) {
    detectionCount++;
    if (detectionCount >= 2) { // Подтверждение обнаружения
      personDetected = true;
      lastDetectionTime = millis();
      triggerAlarm();
      Serial.println("ЧЕЛОВЕК ОБНАРУЖЕН!");
      Serial.print("Дистанция: ");
      Serial.print(frontDist);
      Serial.println(" см");
    }
  } else {
    detectionCount = 0;
  }
  
  // Сброс обнаружения через 10 секунд
  if (personDetected && (millis() - lastDetectionTime > 10000)) {
    personDetected = false;
    Serial.println("Поиск возобновлен");
  }
}

int getFilteredDistance(NewPing &sonar) {
  // Получение отфильтрованного расстояния
  unsigned int distance = sonar.ping_median(5);
  return sonar.convert_cm(distance);
}

void searchMode() {
  // Режим поиска - движение по паттерну
  int frontDist = getFilteredDistance(sonarFront);
  int leftDist = getFilteredDistance(sonarLeft);
  int rightDist = getFilteredDistance(sonarRight);
  
  // Избегание препятствий
  if (frontDist < OBSTACLE_DISTANCE && frontDist > 0) {
    avoidObstacle();
  } else {
    // Движение вперед с зигзагообразной траекторией
    moveForward(150);
    delay(1000);
    
    // Случайный поворот для поиска
    if (random(0, 100) > 70) {
      if (leftDist > rightDist) {
        turnLeft(200);
      } else {
        turnRight(200);
      }
      delay(500);
    }
  }
}

void pursuitMode() {
  // Режим преследования обнаруженного человека
  int frontDist = getFilteredDistance(sonarFront);
  int leftDist = getFilteredDistance(sonarLeft);
  int rightDist = getFilteredDistance(sonarRight);
  
  // Мигание LED и звуковой сигнал
  digitalWrite(LED_PIN, (millis() % 500) < 250);
  tone(BUZZER_PIN, 1500, 100);
  
  if (frontDist < PERSON_DISTANCE && frontDist > OBSTACLE_DISTANCE) {
    // Движение к цели
    moveForward(200);
  } else if (leftDist < rightDist && leftDist > OBSTACLE_DISTANCE) {
    // Поворот налево к цели
    turnLeft(180);
  } else if (rightDist > OBSTACLE_DISTANCE) {
    // Поворот направо к цели
    turnRight(180);
  } else {
    // Поиск цели
    searchPattern();
  }
}

void avoidObstacle() {
  Serial.println("Избегание препятствия");
  
  stopMotors();
  delay(500);
  
  int leftDist = getFilteredDistance(sonarLeft);
  int rightDist = getFilteredDistance(sonarRight);
  
  // Выбор направления с большим пространством
  if (leftDist > rightDist) {
    turnLeft(200);
    delay(800);
  } else {
    turnRight(200);
    delay(800);
  }
  
  moveForward(150);
  delay(1000);
}

void searchPattern() {
  // Паттерн поиска при потере цели
  Serial.println("Выполнение поискового паттерна");
  
  for (int i = 0; i < 3; i++) {
    turnLeft(150);
    delay(300);
    stopMotors();
    delay(500);
    
    turnRight(150);
    delay(600);
    stopMotors();
    delay(500);
  }
}

void triggerAlarm() {
  // Активация сигнала тревоги
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000, 300);
    delay(400);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Функции управления двигателями
void moveForward(int speed) {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_EN, speed);
  analogWrite(MOTOR_B_EN, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
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