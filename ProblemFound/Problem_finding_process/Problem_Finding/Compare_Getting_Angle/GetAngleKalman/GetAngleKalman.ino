#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"

#include <math.h>


Kalman kalmanX;
Kalman kalmanY;
MPU6050 mpu;

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

const long serialFreq = 115200;
float elapsedTime, time, timePrev;
float timeDiffMicros;
const int wireTimeout = 3000;


int16_t offsets[6] = {-3854, 1265, 956, 11, -61, 48};
//int16_t offsets[6] = {-4860, -446, 877, 72, -67, -6};
// -3854, 1265, 956, 11, -61, 48


void setup() {
  delay(2000);

  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // put your setup code here, to run once:
  Serial.begin(serialFreq);


  mpu.initialize();
  // состояние соединения
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  delay(1000);

 setOffsets();

/*
mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
*/

//  mpu.setXAccelOffset(1231);
//  mpu.setYAccelOffset(132);
//  mpu.setZAccelOffset(1232);
 // mpu.setXGyroOffset(-28);
 // mpu.setYGyroOffset(-94);
 // mpu.setZGyroOffset(-24);

 // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  kalmanX.setAngle(180);
  kalmanY.setAngle(180);

  timer = micros();

  timePrev = micros();

    Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0

  delay(5000);

}

void loop() {
  time = micros();
  timeDiffMicros = (time - timePrev);
  elapsedTime = timeDiffMicros / 1000000.0;
  timePrev = time;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  calculateAngles();

  Serial.print(timeDiffMicros); Serial.print(' '); 
  Serial.print(gyroYrate); Serial.print(' ');
  Serial.println(kalAngleY);


}


void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */
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



void setOffsets()
{
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}