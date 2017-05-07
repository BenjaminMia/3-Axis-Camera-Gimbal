// Author: Ben Mia
// Pennsylvania State University
// Electrical Engineering 
// 2017

#include <Wire.h>
#include <math.h>
#include <Servo.h>

//Define Gyro Registers
#define X_LOW_GYRO 0x28
#define X_HIGH_GYRO 0x29
#define Y_LOW_GYRO 0x2A
#define Y_HIGH_GYRO 0x2B
#define Z_LOW_GYRO 0x2C
#define Z_HIGH_GYRO 0x2D
#define PWR_REGISTER_GYRO 0x20
#define HIGH_PASS 0x21
#define GYRO_SENSITIVITY 0x23

//Define Accelerometer Registers
#define X_LOW_ACC 0x32
#define X_HIGH_ACC 0x33
#define Y_LOW_ACC 0x34
#define Y_HIGH_ACC 0x35 
#define Z_LOW_ACC 0x36
#define Z_HIGH_ACC 0x37
#define PWR_REGISTER_ACC 0x2D

//Define Gyro Variables
int X_GYRO_LOW;
int X_GYRO_HIGH;
int16_t RAW_X_GYRO;
float angle_X = 0;

int Y_GYRO_LOW;
int Y_GYRO_HIGH;
int16_t RAW_Y_GYRO;
float angle_Y = 0;

int Z_GYRO_LOW;
int Z_GYRO_HIGH;
int16_t RAW_Z_GYRO;
float angle_Z = 0;

//Define Accelerometer Variables
int X_ACC_LOW;
int X_ACC_HIGH;
int16_t RAW_X_ACC;

int Y_ACC_LOW;
int Y_ACC_HIGH;
int16_t RAW_Y_ACC;

int Z_ACC_LOW;
int Z_ACC_HIGH;
int16_t RAW_Z_ACC;

//Define I2C Addresses
const int Gyro = 0x68;
const int Acc = 0x1D;

//Define loop variables
//Math Declarations

float pi = 3.14159265359;
float dt = 0.015;
float tau = 0.01;
float alpha;
float abs_acc;
  
// Timing
unsigned long start = 0, finish, elapsed;

float angle_gyro_x = 0;
float angle_gyro_y = 0;
float angle_gyro_z = 0;

//PID
byte PWM;
float PID, Accum, Error;

//Define Servo
Servo servo;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(100);

  //Set up servo
  servo.attach(10);
  servo.write(90);
  
  //Turn on accelerometer
  Wire.beginTransmission(Acc);
  Wire.write(PWR_REGISTER_ACC);
  Wire.write(0x08);
  Wire.endTransmission();

  //Turn on gyro
  Wire.beginTransmission(Gyro); 
  Wire.write(PWR_REGISTER_GYRO);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.beginTransmission(Gyro);
  Wire.write(0x23);   // CTRL_REG4 - Sensitivity, Scale Selection
  Wire.write(0x10);   // 500dps: 48d - 00010000b
  Wire.endTransmission();

}

void loop() {
  //Begin Loop
  start = millis();
  
  read_X();
  read_Y();
  read_Z();

  //Convert Raw Gyro Data to Angular Velocity
  float gyro_Rate_X = .0175 * float(RAW_X_GYRO) + .7;
  float angle_Xc = gyro_Rate_X * dt;
  float gyro_Rate_Y = .0175 * float(RAW_Y_GYRO) - .25;
  float angle_Yc = gyro_Rate_Y * dt;
  float gyro_Rate_Z = .0175 * float(RAW_Z_GYRO) + .3;
  float angle_Zc = gyro_Rate_Z * dt;
  
  //Convert Raw Accelerometer Data to Acceleration in G's (9.8 M/s^2)
  float acc_Rate_X = float(RAW_X_ACC) / 256 - .07;
  float acc_Rate_Y = float(RAW_Y_ACC) / 256 - .04;
  float acc_Rate_Z = float(RAW_Z_ACC) / 256 + .07;

  //Determine angle according to gyroscope
  angle_gyro_x += angle_Xc;
  angle_gyro_y += angle_Yc;
  angle_gyro_z += angle_Zc;

  //Convert Acceleration into tilt angle
  float angle_X_Acc = atan2(acc_Rate_Y, acc_Rate_Z) * 180 / pi;
  float angle_Y_Acc = atan2(acc_Rate_X, acc_Rate_Z) * 180 / pi;
  float angle_Z_Acc = atan2(acc_Rate_Y, acc_Rate_X) * 180 / pi;
  
  //Determine absolute acceleration
  abs_acc = sqrt((acc_Rate_X * acc_Rate_X) + (acc_Rate_Y * acc_Rate_Y) + (acc_Rate_Z * acc_Rate_Z));
  
  //Complementary Filter
  if (abs_acc <= 1.25) { 
    alpha = tau / (tau + dt);

    //Filter X
    angle_X = angle_X + angle_Xc;
    float error_X = angle_X_Acc - angle_X;
    angle_X = angle_X + (1 - alpha) * error_X;

    //Filter Y
    angle_Y = angle_Y + angle_Yc;
    float error_Y = angle_Y_Acc - angle_Y;
    angle_Y = angle_Y + (1 - alpha) * error_Y;

    angle_Z += angle_Zc;
  }

  else {
    angle_X += angle_Xc;
    angle_Y += angle_Yc;
    angle_Z += angle_Zc;
  }
  
  //Convert angular data to servo position
  float servo_angle = map(angle_Y, -90, 90, 0, 179);
  servo.write(servo_angle);
  
  Serial.print("X: ");
  Serial.print(angle_X);
  Serial.print("\t \t Y: ");
  Serial.print(angle_Y);
  Serial.print(" \t \t Z: ");
  Serial.println(angle_Z);

  finish = millis();
  elapsed = finish - start;
  dt = elapsed / 1000.0;
  start = elapsed = 0;
  
}

//Read Sensors

void read_X() {
  //Gyro 
  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(X_LOW_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  X_GYRO_LOW = Wire.read();

  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(X_HIGH_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  X_GYRO_HIGH = Wire.read();
  
  RAW_X_GYRO = (X_GYRO_HIGH << 8) + X_GYRO_LOW;

  //Accelerometer
  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(X_LOW_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  X_ACC_LOW = Wire.read();

  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(X_HIGH_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  X_ACC_HIGH = Wire.read();
  
  RAW_X_ACC = (X_ACC_HIGH << 8) + X_ACC_LOW;
}

void read_Y() {
  //Gyro
  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(Y_LOW_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  Y_GYRO_LOW = Wire.read();

  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(Y_HIGH_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  Y_GYRO_HIGH = Wire.read();
  
  RAW_Y_GYRO = (Y_GYRO_HIGH << 8) + Y_GYRO_LOW;

  //Accelerometer
  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(Y_LOW_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  Y_ACC_LOW = Wire.read();

  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(Y_HIGH_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  Y_ACC_HIGH = Wire.read();
  
  RAW_Y_ACC = (Y_ACC_HIGH << 8) + Y_ACC_LOW;
}

void read_Z() {
  //Gyro
  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(Z_LOW_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  Z_GYRO_LOW = Wire.read();

  Wire.beginTransmission(Gyro);     // Begin transmission to sensor
  Wire.write(Z_HIGH_GYRO);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Gyro, 1);
  Z_GYRO_HIGH = Wire.read();
  
  RAW_Z_GYRO = (Z_GYRO_HIGH << 8) + Z_GYRO_LOW;

  //Accelerometer
  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(Z_LOW_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  Z_ACC_LOW = Wire.read();

  Wire.beginTransmission(Acc);     // Begin transmission to sensor
  Wire.write(Z_HIGH_ACC);  //Ask the particular registers for data
  Wire.endTransmission();
  Wire.requestFrom(Acc, 1);
  Z_ACC_HIGH = Wire.read();
  
  RAW_Z_ACC = (Z_ACC_HIGH << 8) + Z_ACC_LOW;  
}



