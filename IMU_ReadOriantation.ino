#include <Wire.h>

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#include "Tilt_Compensation.h"
#include "math.h"
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

uint32_t timestamp;


unsigned long previousTime = 0;
const unsigned long interval = 1;  // Update interval in milliseconds

// Read accelerometer and gyroscope data
int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;

// Calibrated coefficients
float Cal_Accel_X;
float Cal_Accel_Y;
float Cal_Accel_Z;

float Cal_Gyro_Roll;
float Cal_Gyro_Pitch;
float Cal_Gyro_Yaw;

// Calibrated form raw data
double CalibratedAx;
double CalibratedAy;
double CalibratedAz;

double CalibratedGx;
double CalibratedGy;
double CalibratedGz;

// Raw Orientation form accelerometer
double Pitch_Accel;
double Roll_Accel;

double Roll_Gyro;
double Pitch_Gyro;
double Yaw_Gyro;
// Kalman
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 4;  // std dev = 2
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 4;  // std dev = 2
float KalmanOutput[] = { 0, 0 };        // angle prediction, uncertainty of the prediction
float Yaw_TiltCom;
int Round;
float Yaw_prev;
int Initial_count = 0;
float Initial_Encoder_Value = 0;

//Encoder Start
#define EN_Pin_A 3
#define EN_Pin_B 2

int Encoder_count = 0;
int A = 0;
int B = 0;

float Encoder_Deg = 0;
//Encoder End

//tilt compensation
TiltCom TiltCom_Var = { 0 };


void setup() {
  Serial.begin(115200);

  //Encoder Setup Start
  pinMode(EN_Pin_A, INPUT_PULLUP);
  pinMode(EN_Pin_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EN_Pin_A), count_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_Pin_B), count_B, CHANGE);
  //Encoder Setup End

  //Wire.setSDA(0);
  //Wire.setSCL(1);
  //Wire.begin();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  timestamp = millis();

  Wire.setClock(400000);  // 400KHz
}

void loop() {
  // Raw Data
  float ax, ay, az;
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) >= interval) {
    timestamp = millis();
    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);
    // Accelearometer
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Orientation form Accelerometer
    Roll_Accel = atan(ay / sqrt(ax * ax + az * az)) * 180.0 / 3.142;
    Pitch_Accel = -atan(ax / sqrt(ay * ay + az * az)) * 180.0 / 3.142;

    // Orientaation form Gyrometer
    float dt = interval / 1000.0;  // Convert interval to seconds
    Roll_Gyro += gx * dt;
    Pitch_Gyro += gy * dt;
    Yaw_Gyro += gz * dt;
    DiscreteKalmanFilter(KalmanAngleRoll, CalibratedGx, Roll_Accel, KalmanUncertaintyAngleRoll);
    KalmanAngleRoll = KalmanOutput[0];
    KalmanUncertaintyAngleRoll = KalmanOutput[1];

    DiscreteKalmanFilter(KalmanAnglePitch, CalibratedGy, Pitch_Accel, KalmanUncertaintyAnglePitch);
    KalmanAnglePitch = KalmanOutput[0];
    KalmanUncertaintyAnglePitch = KalmanOutput[1];

    TiltCom_magnetoSetup(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, &TiltCom_Var);
    TiltCom_rollpitchSetup(KalmanAngleRoll * PI / 180.0, KalmanAnglePitch * PI / 180.0, &TiltCom_Var);

    TileCom_Cal(&TiltCom_Var);
    UnwrappingTiltCom(TiltCom_Var.yawOut * 180.0 / PI);

    Encoder_Deg = Encoder_count * 360.0 / 2400.0; //+ Initial_Encoder(Yaw_TiltCom);

    // Serial.print(Roll_Accel);
    // Serial.print(" ");
    // Serial.print(Pitch_Accel);
    // Serial.print(" ");
    //Serial.print("Roll deg");
    Serial.print(KalmanAngleRoll);
    Serial.print(" ");
    //Serial.print("Pitch deg");
    Serial.print(KalmanAnglePitch);
    // Serial.print(" ");
    // //Serial.print("mx");
    // Serial.print(mag.magnetic.x, 4);
    // Serial.print(" ");
    // //Serial.print("my");
    // Serial.print(mag.magnetic.y, 4);
    // Serial.print(" ");
    // //Serial.print("mz");
    // Serial.print(mag.magnetic.z, 4);
    Serial.print(" ");
    Serial.print(180);
    Serial.print(" ");
    Serial.print(-180);
    // Serial.print(" ");
    // Serial.print(Round);
    // Serial.print(" ");
    // Serial.print(Yaw_prev);
    // Serial.print(" ");
    //Serial.print("Yaw in deg");
    // Serial.print(TiltCom_Var.yawOut * 180.0 / PI, 4);
    Serial.print(" ");
    Serial.println(Encoder_Deg);
    // Serial.print(" ");
    // Serial.println(Yaw_TiltCom, 4);
  }
}

void DiscreteKalmanFilter(float KalmanState, float KalmanInput, float KalmanMeasurement, float KalmanUncertainty) {  // KalmanInput = rotation rate, KalmanMeasurement = accelerometer angle
  KalmanState = KalmanState + interval / 1000.0 * KalmanInput;                                                       // Xk = Ax + Bu
  KalmanUncertainty = KalmanUncertainty + 1 * 1;                                                                     // Pk = APkAT + Q
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 10 * 10);                                              // Pk = PCT/CPCT + R
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);                                        // Xk = Xk + K(y - Cx)
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;                                                          // Pk = (I - KC)Pk

  KalmanOutput[0] = KalmanState;
  KalmanOutput[1] = KalmanUncertainty;
}

void UnwrappingTiltCom(float Yaw) {

  if (Yaw - Yaw_prev > 300) {
    Round--;
  } else if (Yaw - Yaw_prev < -300) {
    Round++;
  }

  Yaw_TiltCom = Yaw + Round * 360;
  Yaw_prev = Yaw;
}

float Initial_Encoder(float Yaw){
  if(Initial_count == 0){
    Initial_Encoder_Value = Yaw;
  }
  Initial_count = 1;

  return Initial_Encoder_Value;
}

void count_A() {
  if (digitalRead(EN_Pin_A) == digitalRead(EN_Pin_B)) {
    Encoder_count--;
  } else if (digitalRead(EN_Pin_A) != digitalRead(EN_Pin_B)) {
    Encoder_count++;
  }
}

void count_B() {
  if (digitalRead(EN_Pin_A) != digitalRead(EN_Pin_B)) {
    Encoder_count--;
  } else if (digitalRead(EN_Pin_A) == digitalRead(EN_Pin_B)) {
    Encoder_count++;
  }
}
