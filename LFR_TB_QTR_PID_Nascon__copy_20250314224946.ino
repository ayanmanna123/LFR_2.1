/* Line Follower using
   - QTR Array
   - TB6612FNG Motor Driver
   - PID Controller
*/

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Motor Driver Pins
#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int max_speed = 60;
int error = 0;
int adj = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

int P, I, D, lastError = 0;
uint16_t position;

// Function Declarations
void customForward(int L, int R);
void sharp_right();
void sharp_left();
void PID_control();

void setup() {
  Serial.begin(9600);
  brake(motor1, motor2);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }

  Serial.println("Calibration Done");
}

void loop() {
  PID_control();
}

void PID_control() {
  position = qtr.readLineWhite(sensorValues);
  Serial.println(position);

  error = 3500 - position;

  if (position > 3000 && position < 4300) {
     
    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;

    adj = P * Kp + I * Ki + D * Kd;

    int L = max_speed + adj;
    int R = max_speed - adj;

    if (L > max_speed) L = max_speed;
    if (R > max_speed) R = max_speed;
    if (L < 0) L = 0;
    if (R < 0) R = 0;

    customForward(L, R);
  } else if (position >= 1300 && position <= 3000) {
    sharp_right();
  } else if (position >= 4300 && position <= 6990){
    sharp_left();
  }
   
}

void customForward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}

void sharp_right() {
  motor1.drive(-190);
  motor2.drive(255);
}

void sharp_left() {
  motor1.drive(255);
  motor2.drive(-190);
}
void stop() {
  motor1.drive(0);
  motor2.drive(0);
}
