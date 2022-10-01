// AgriDoc Arduino Mega firmware
// Runs the rear wheels through the L298N, counts the encoders, reads the BNO055,
// passes on the NEO-M8N GPS and switches the pump relay. The Jetson talks to it over USB at 115200 baud.
//
// Jetson to Mega, at least every 0.5 s:
//   C <left wheel rad/s> <right wheel rad/s> <pump 0 or 1>
// Mega to Jetson, every 20 ms:
//   E <millis> <left ticks> <right ticks>
//   I <qw> <qx> <qy> <qz> <gx> <gy> <gz> <ax> <ay> <az>   (rad/s and m/s^2)
// Mega to Jetson, once a second:
//   $GNGGA,...   the GGA sentence from the GPS as it came, the Jetson parses it
// Wheels and pump stop when no command comes for 0.5 s.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// L298N, take the ENA and ENB jumpers off
const int LEFT_EN = 9;
const int LEFT_IN1 = 8;
const int LEFT_IN2 = 7;
const int RIGHT_EN = 10;
const int RIGHT_IN3 = 11;
const int RIGHT_IN4 = 12;

// encoder A needs an interrupt pin, B can be any pin
const int LEFT_ENC_A = 2;
const int LEFT_ENC_B = 4;
const int RIGHT_ENC_A = 3;
const int RIGHT_ENC_B = 5;

// NEO-M8N TX goes to RX1. Its RX stays unconnected, it keeps its defaults (9600 baud, 1 fix a second)
const unsigned long GPS_BAUD = 9600;

const int RELAY = 22;
// the relay module switches on when IN is pulled low
const bool RELAY_ACTIVE_LOW = true;

// change these after the wheels up test if a wheel or its count goes the wrong way
const bool LEFT_MOTOR_INVERT = false;
const bool RIGHT_MOTOR_INVERT = true;
const bool LEFT_ENC_INVERT = false;
const bool RIGHT_ENC_INVERT = true;

// 7 pulses per motor turn, both edges of A are counted, 99.5:1 gearbox
const float TICKS_PER_REV = 1393.0;
// 50 rpm at 12 V, the L298N drops about 2.5 V so the wheels top out near 38 rpm (4 rad/s)
const float MAX_SPEED = 4.0;
// the gear motors don't start turning below about this PWM
const float MIN_PWM = 40.0;
const float KP = 10.0;
const float KI = 40.0;

const unsigned long LOOP_MS = 20;
const unsigned long TIMEOUT_MS = 500;

struct Wheel {
  int en;
  int in1;
  int in2;
  bool invert;
  float target;
  float integral;
  long lastTicks;
};

Wheel leftWheel = {LEFT_EN, LEFT_IN1, LEFT_IN2, LEFT_MOTOR_INVERT, 0, 0, 0};
Wheel rightWheel = {RIGHT_EN, RIGHT_IN3, RIGHT_IN4, RIGHT_MOTOR_INVERT, 0, 0, 0};

volatile long leftTicks = 0;
volatile long rightTicks = 0;

Adafruit_BNO055 bno(55, 0x28);
bool imuOk = false;

char line[48];
int lineLen = 0;
// NMEA sentences are at most 82 characters
char gpsLine[90];
int gpsLineLen = 0;
unsigned long lastCommand = 0;
unsigned long lastLoop = 0;

void leftPulse() {
  if (digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B)) leftTicks++;
  else leftTicks--;
}

void rightPulse() {
  if (digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B)) rightTicks++;
  else rightTicks--;
}

void setPump(bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY, level ? HIGH : LOW);
}

void drive(Wheel &w, float pwm) {
  if (w.invert) pwm = -pwm;
  digitalWrite(w.in1, pwm > 0 ? HIGH : LOW);
  digitalWrite(w.in2, pwm < 0 ? HIGH : LOW);
  analogWrite(w.en, (int)constrain(fabs(pwm), 0, 255));
}

void control(Wheel &w, long ticks, float dt) {
  float speed = (ticks - w.lastTicks) * 2.0 * PI / TICKS_PER_REV / dt;
  w.lastTicks = ticks;
  if (w.target == 0.0) {
    w.integral = 0.0;
    drive(w, 0);
    return;
  }
  // feed forward from the top speed and the start PWM, plus a PI correction from the encoder
  float feed = MIN_PWM + (255.0 - MIN_PWM) * fabs(w.target) / MAX_SPEED;
  if (w.target < 0) feed = -feed;
  float error = w.target - speed;
  w.integral = constrain(w.integral + error * dt, -2.5, 2.5);
  drive(w, feed + KP * error + KI * w.integral);
}

void handleLine() {
  if (line[0] != 'C') return;
  char *left = strtok(line + 1, " ");
  char *right = strtok(NULL, " ");
  char *pump = strtok(NULL, " ");
  if (left == NULL || right == NULL || pump == NULL) return;

  leftWheel.target = constrain(atof(left), -MAX_SPEED, MAX_SPEED);
  rightWheel.target = constrain(atof(right), -MAX_SPEED, MAX_SPEED);
  setPump(atoi(pump) == 1);
  lastCommand = millis();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[lineLen] = '\0';
      if (lineLen > 0) handleLine();
      lineLen = 0;
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

// Only the GGA sentence is passed on (position, fix quality, satellites and HDOP).
// The talker can be GP, GN or GL depending on the satellites in use
void readGps() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '$') {
      gpsLineLen = 0;
    }
    if (c == '\n' || c == '\r') {
      gpsLine[gpsLineLen] = '\0';
      if (gpsLineLen > 6 && gpsLine[0] == '$' && strncmp(gpsLine + 3, "GGA", 3) == 0) {
        Serial.println(gpsLine);
      }
      gpsLineLen = 0;
    } else if (gpsLineLen < (int)sizeof(gpsLine) - 1) {
      gpsLine[gpsLineLen++] = c;
    }
  }
}

void printValue(float value, int digits) {
  Serial.print(' ');
  Serial.print(value, digits);
}

void sendImu() {
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print('I');
  printValue(q.w(), 4);
  printValue(q.x(), 4);
  printValue(q.y(), 4);
  printValue(q.z(), 4);
  // the library gives the gyro in deg/s
  printValue(gyro.x() * DEG_TO_RAD, 4);
  printValue(gyro.y() * DEG_TO_RAD, 4);
  printValue(gyro.z() * DEG_TO_RAD, 4);
  printValue(accel.x(), 2);
  printValue(accel.y(), 2);
  printValue(accel.z(), 2);
  Serial.println();
}

void setup() {
  // relay off before the pin becomes an output so the pump doesn't kick at start up
  setPump(false);
  pinMode(RELAY, OUTPUT);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightPulse, CHANGE);

  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);

  // gyro and accel fusion only, the magnetometer is useless next to the motors
  imuOk = bno.begin(OPERATION_MODE_IMUPLUS);
  // the Adafruit board has a 32 kHz crystal, it keeps the fusion steadier
  if (imuOk) bno.setExtCrystalUse(true);
  Serial.println(imuOk ? "READY" : "READY NO_IMU");
}

void loop() {
  readSerial();
  readGps();

  unsigned long now = millis();
  if (now - lastCommand > TIMEOUT_MS) {
    leftWheel.target = 0.0;
    rightWheel.target = 0.0;
    setPump(false);
  }

  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  noInterrupts();
  long left = leftTicks;
  long right = rightTicks;
  interrupts();
  if (LEFT_ENC_INVERT) left = -left;
  if (RIGHT_ENC_INVERT) right = -right;

  control(leftWheel, left, dt);
  control(rightWheel, right, dt);

  Serial.print("E ");
  Serial.print(now);
  Serial.print(' ');
  Serial.print(left);
  Serial.print(' ');
  Serial.println(right);

  if (imuOk) sendImu();
}
