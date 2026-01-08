#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SBUS.h>
#include <Servo.h>

/* ================= OBJECTS ================= */
MPU6050 imu;
SBUS sbus(Serial1);

Servo m1, m2, m3, m4;
Servo flower;

/* ================= SBUS ================= */
uint16_t ch[16];
bool failsafe, lostFrame;

/* ================= IMU ================= */
float roll, pitch;
float gyroX, gyroY, gyroZ;

/* ================= PID ================= */
float kp = 4.5;
float ki = 0.02;
float kd = 0.08;

float rollI = 0, pitchI = 0;
float lastRollErr = 0, lastPitchErr = 0;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial1.begin(100000, SERIAL_8E2);
  sbus.begin();

  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 FAIL");
    while (1);
  }

  m1.attach(3);
  m2.attach(5);
  m3.attach(6);
  m4.attach(9);
  flower.attach(10);

  stopMotors();
  flower.writeMicroseconds(1000);

  delay(3000);
  Serial.println("Quad Ready");
}

/* ================= LOOP ================= */
void loop() {
  if (sbus.read(&ch[0], &failsafe, &lostFrame)) {

    bool armed = ch[4] > 1500;

    readIMU();

    if (!armed || failsafe) {
      stopMotors();
      return;
    }

    float throttle = map(ch[2], 172, 1811, 1000, 2000);
    float rollSet  = map(ch[0], 172, 1811, -20, 20);
    float pitchSet = map(ch[1], 172, 1811, -20, 20);
    float yawRate  = map(ch[3], 172, 1811, -150, 150);

    float rollPID  = pid(rollSet, roll, rollI, lastRollErr);
    float pitchPID = pid(pitchSet, pitch, pitchI, lastPitchErr);

    mixMotors(throttle, rollPID, pitchPID, yawRate);
    handleFlower();
  }
}

/* ================= IMU ================= */
void readIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;

  roll  = atan2(ay, az) * 57.3;
  pitch = atan2(-ax, az) * 57.3;
}

/* ================= PID ================= */
float pid(float setpoint, float measured, float &iTerm, float &lastErr) {
  float err = setpoint - measured;
  iTerm += err * ki;
  float d = (err - lastErr);
  lastErr = err;
  return kp * err + iTerm + kd * d;
}

/* ================= MIXER ================= */
void mixMotors(float t, float r, float p, float y) {
  int m1v = t + p + r - y;
  int m2v = t + p - r + y;
  int m3v = t - p - r - y;
  int m4v = t - p + r + y;

  m1.writeMicroseconds(constrain(m1v, 1000, 2000));
  m2.writeMicroseconds(constrain(m2v, 1000, 2000));
  m3.writeMicroseconds(constrain(m3v, 1000, 2000));
  m4.writeMicroseconds(constrain(m4v, 1000, 2000));
}

/* ================= FLOWER ================= */
void handleFlower() {
  if (ch[5] > 1500)
    flower.writeMicroseconds(1600);
  else
    flower.writeMicroseconds(1000);
}

/* ================= SAFETY ================= */
void stopMotors() {
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
}
