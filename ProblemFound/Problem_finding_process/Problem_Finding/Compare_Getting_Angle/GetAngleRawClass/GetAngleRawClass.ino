#include <Wire.h>
#include <MPU6050.h>
#include <math.h>


MPU6050 mpu;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_rawXf, Gyro_rawYf, Gyro_rawZf;
float Acceleration_angle[2] = {0, 0};
float Gyro_angle[2] = {0, 0};
float Total_angle[2] = {0, 0};
float rad_to_deg = 180/3.141592654;
const float COEF_GYRO_COMP = 0.99;
uint32_t timer;
uint32_t timerController;

float dt; // passed time in seconds in loop

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

  //resetOffsets();
  //setOffsets();

//  mpu.setXAccelOffset(1231);
//  mpu.setYAccelOffset(132);
//  mpu.setZAccelOffset(1232);
 // mpu.setXGyroOffset(-28);
 // mpu.setYGyroOffset(-94);
 // mpu.setZGyroOffset(-24);
 // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  timer = micros();

  timePrev = micros();

    Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); 
    Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); 
    Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); 
    Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); 
    Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); 
    Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); 

  delay(3000);

}

void loop() {
  time = micros();
  timeDiffMicros = (time - timePrev);
  elapsedTime = timeDiffMicros / 1000000.0;
  timePrev = time;

  mpu.getMotion6(&Acc_rawX, &Acc_rawY, &Acc_rawZ, &Gyr_rawX, &Gyr_rawY, &Gyr_rawZ);
  getAngle();

  Serial.print(timeDiffMicros); 
  Serial.print(' '); Serial.print(Gyro_rawYf); 
  Serial.print(' ');
  Serial.println(Total_angle[1]);
}


void calculateAngles() {
  /* Calculate the angls based on the different sensors and algorithm */

//  gyroYrate = -((float)gy / 131.0);
  // Calculate gyro angle without any filter
 // gyroXangle += gyroXrate * ((float)(micros() - timer) / 1000000);
//  gyroYangle += gyroYrate * ((float)(micros() - timer) / 1000000);

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

void resetOffsets()
{
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
}


void setOffsets(int16_t* offsets)
{
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}

