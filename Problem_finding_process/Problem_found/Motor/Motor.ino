#include "MPU6050.h"
#include "Kalman.h"
#include <Servo.h>
#include <math.h>

//#define DEBUG_ANGLE


Kalman kalmanX;
Kalman kalmanY;
MPU6050 mpu;

/////
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_rawXf, Gyro_rawYf, Gyro_rawZf;
float Acceleration_angle[2] = {0, 0};
float Gyro_angle[2] = {0, 0};
float Total_angle[2] = {0, 0};
float rad_to_deg = 180/3.141592654;

const float COEF_GYRO_COMP = 0.99;

const long serialFreq = 115200;
float elapsedTime, time, timePrev;
float timeDiffMicros;
const int wireTimeout = 3000;

//int16_t offsets[6] = {-3854, 1265, 956, 11, -61, 48};
//int16_t offsets[6] = {-4683, -345, 856, 66, -82, -8}; // где проблема
//int16_t offsets[6] = {-478, -4909, 1212, 136, 31, -78};

int16_t offsets[6] = {-2542, -2394, 1002, 20, -36, 40};   // наметрво спаянный
//int16_t offsets[6] = {-2762, -2308, 1130, 22, -40, 29};

// int16_t offsets[6] = {1231, 132, 1232, -28, -94, -24}; // MOTOR
////



byte servoPin = 9;

Servo servo;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroXangle = 180; // Angle calculate using the gyro
float gyroYangle = 180;

uint32_t timer;
uint32_t timerController;

float dt; // passed time in seconds in loop

float accXangle; // Angle calculate using the accelerometer
float accYangle;

float gyroYrate;

float kalAngleX; // Calculate the angle using a Kalman filter
float kalAngleY;

float filterCoef = 0.8;
// float filtY = -75;

float errorInt = 0.0;
float errorDiff = NAN;
float lastError;

float kp = 2.5000;
float ki = 0.5000;
float kd = 0.8500;
uint16_t throttle = 1340; // new bldc from 1080 start


const int MIN_PWM = 1000;
const int MAX_PWM = 2000;


void setup() {
  // put your setup code here, to run once:

  delay(2000);

  servo.attach(servoPin);

  ////!!!!!!!!!!!!!!!!!! МОЖЕТ ЭТО ИСПРАВИТ ОТ ЗАВИСАНИЯ

  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

 // Wire.begin();
  
  
  Serial.begin(serialFreq);
  mpu.initialize();
  // состояние соединения
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  delay(1000);

  setOffsets();

  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  kalmanX.setAngle(180);
  kalmanY.setAngle(180);

  timer = micros();
  float error;
  for (int i = 0; i < 1000; i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    calculateAngles();
    error = kalAngleY;
    Serial.println(error);
  }

  timer = micros();

  Serial.println("Starting motor...");
  servo.writeMicroseconds(MIN_PWM);
  Serial.println("Waiting a second");
  delay(1000);

  bool waiting = true;
  char data;
  Serial.println("Coefficients:");
  Serial.println("kp = " + String(kp));
  Serial.println("ki = " + String(ki));
  Serial.println("kd = " + String(kd));  
  Serial.println("th = " + String(throttle));  
  Serial.println("Connect battery and type 0 to start");

  while (waiting) {  
  if (Serial.available()) {    
    data = Serial.read();

    switch(data) {
      case 48: {
        waiting = false;
        Serial.println("Starting in three seconds...");
        delay(3000);
        break;
      }
    }
  } 
  }

  // GO!
  timerController = micros();
}


bool stop = false;

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0){ switchWorking(Serial.read()); } // read command to stop/start

  if (stop == true) {
    servo.writeMicroseconds(MIN_PWM);
    delay(200);
    return;
  }

  dt = ((float)(micros() - timerController) / 1000000);  
  timerController = micros();

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  calculateAngles();
  /* getAngle(); */

  // filtY = filterCoef * filtY + (1 - filterCoef) * kalAngleY;

  float error = -kalAngleY ; // Total_angle[1];
  //float error = -filtY;


  errorInt += error * dt;

  if (isnan(errorDiff)) {
    Serial.println("nandif");
    errorDiff = 0;
  } else {    
    errorDiff = (error - lastError) / dt;
  }

  errorDiff = gyroYrate;  // errorDiff = -Gyro_rawYf;

  lastError = error;

  int pwmVal = map(kalAngleY, -90, 90, 1100, 1900);

  float control = error * kp + errorInt * ki + errorDiff * kd;

  uint16_t pwm = constrain(throttle + control, MIN_PWM, MAX_PWM);

  // safe sat
  if (pwm > 1520) pwm = 1520;

  if (isnan(error)) {
    Serial.println("\n\n\nNAN err\n");
   // error = 0;
  }

    // safe error if problem and nan
  if (error > 68 || error < -68 || error != error){
    pwm = MIN_PWM;
    stop = true;
  }


  servo.writeMicroseconds(pwm);

  /* error | P | I | D | pwm | dt */

  Serial.print(error); Serial.print(','); Serial.print(error * kp); Serial.print(',');
  Serial.print(errorInt * ki); Serial.print(','); Serial.print(errorDiff * kd); Serial.print(',');
  Serial.print(pwm); Serial.print(','); Serial.println(String(dt, 6)); 
  //Serial.println("err:" + String(error) + ",P:" + String(error * kp) + ",I:" + String(errorInt * ki) + ",D:" + String(errorDiff * kd) + ",U:" + String(pwm) + ",dt:" + String(dt, 6));
}

void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */
#ifdef DEBUG_ANGLE
  Serial.println("Try get angle");
#endif

  accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;
  accXangle = (atan2(ay, az) + PI) * RAD_TO_DEG;
  float gyroXrate = (float)gx / 131.0;
  gyroYrate = -((float)gy / 131.0);
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
  gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);
  /*
    // Calculate gyro angle using the unbiased rate
    gyroXangle += kalmanX.getRate() * ((float)(micros() - timer) / 1000000);
    gyroYangle += kalmanY.getRate() * ((float)(micros() - timer) / 1000000);
  */
  /*
    // Calculate the angle using a Complimentary filter
    compAngleX = ((float)(1 - COMPL_K) * (compAngleX + (gyroXrate * (float)(micros() - timer) / 1000000))) + (COMPL_K * accXangle);
    compAngleY = ((float)(1 - COMPL_K) * (compAngleY + (gyroYrate * (float)(micros() - timer) / 1000000))) + (COMPL_K * accYangle);
  */
  
  // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros() - timer) / 1000000) - 180;
  kalAngleY = -(kalmanY.getAngle(accYangle, gyroYrate, (float)(micros() - timer) / 1000000) - 180);

  timer = micros();
}

inline void getAngle()
{

  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt((Acc_rawX/16384.0)*(Acc_rawX/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;  /*---X---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt((Acc_rawY/16384.0)*(Acc_rawY/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg; /*---Y---*/
  //Acceleration_angle[0] -= AccErrorX_calc;Acceleration_angle[1] -= AccErrorY_calc;

  Gyro_rawXf = (Gyr_rawX/131.0);// - GyroErrorX_calc;   /*---X---*/
  Gyro_rawYf = (Gyr_rawY/131.0);// - GyroErrorY_calc;    /*---Y---*/

  //Gyro_angle[0] = Gyro_angle[0] + Gyro_rawXf * elapsedTime;   /*---X---*/
  Gyro_angle[1] = Gyro_angle[1] + Gyro_rawYf * elapsedTime;    /*---Y---*/

  Gyro_angle[1] = COEF_GYRO_COMP * Gyro_angle[1] + (1-COEF_GYRO_COMP) * Acceleration_angle[1];
  Total_angle[1] = Gyro_angle[1];
}

void setOffsets()
{
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}





void switchWorking(int ch) {
    switch(ch)
    {
      case '0': // STOP STOP STOP
        stop = true;
        servo.writeMicroseconds(1300);
        break;   
    }
}
