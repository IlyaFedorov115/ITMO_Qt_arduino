#include <Wire.h>

const float AccErrorX_calc = -0.01;
const float AccErrorY_calc = -3.24;
const float GyroErrorX_calc = -0.36;
const float GyroErrorY_calc = 2.27;
const float GyroErrorZ_calc = -1.64;

const float COEF_GYRO_COMP = 0.99;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_rawXf, Gyro_rawYf, Gyro_rawZf;

float Acceleration_angle[2] = {0, 0};
float Gyro_angle[2] = {0, 0};
float Total_angle[2] = {0, 0};
float rad_to_deg = 180/3.141592654;

const long serialFreq = 115200;
float elapsedTime, time, timePrev;
float timeDiffMicros;
const int wireTimeout = 3000;




void setup() {
  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // put your setup code here, to run once:
  Serial.begin(serialFreq);

  timePrev = micros();

}

void loop() {
  time = micros();
  timeDiffMicros = (time - timePrev);
  elapsedTime = timeDiffMicros / 1000000.0;
  timePrev = time;

  getAngle();

  Serial.println(timeDiffMicros);
  Serial.println(Total_angle[1]);


}



inline void getAngle()
{

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                     //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 

  Acc_rawX=Wire.read()<<8|Wire.read(); //  needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

 // Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;    /*---X---*/
 // Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg; /*---Y---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt((Acc_rawX/16384.0)*(Acc_rawX/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;  /*---X---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt((Acc_rawY/16384.0)*(Acc_rawY/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg; /*---Y---*/
  Acceleration_angle[0] -= AccErrorX_calc;
  Acceleration_angle[1] -= AccErrorY_calc;

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); //Just 6 registers  

  Gyr_rawX = Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read()<<8|Wire.read();
  Gyr_rawZ = Wire.read()<<8|Wire.read();
  Gyro_rawXf = (Gyr_rawX/131.0) - GyroErrorX_calc;   /*---X---*/
  Gyro_rawYf = (Gyr_rawY/131.0) - GyroErrorY_calc;    /*---Y---*/

  //Gyro_angle[0] = Gyro_angle[0] + Gyro_rawXf * elapsedTime;   /*---X---*/
  Gyro_angle[1] = Gyro_angle[1] + Gyro_rawYf * elapsedTime;    /*---Y---*/

  Gyro_angle[1] = COEF_GYRO_COMP * Gyro_angle[1] + (1-COEF_GYRO_COMP) * Acceleration_angle[1];
  Total_angle[1] = Gyro_angle[1];
}
