#include <Wire.h>
//#include <MPU6050.h>

#include "bldc_api.h"
const long serialFreq = 115200;

/*** MPU ***/
//int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acc_rawX_filt, Acc_rawY_filt, Acc_rawZ_filt, Gyr_rawX_filt, Gyr_rawY_filt, Gyr_rawZ_filt;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float Acceleration_angle_filt[2];
float Gyro_angle_filt[2];
float Total_angle_filt[2];
float rad_to_deg = 180/3.141592654;

namespace SERVO_DATA {
  const int ESC_PIN = 9;  //D9
  const int ESC_MIL_MIN = 1200;
  const int ESC_MIL_MAX = 2000;
  BLDC_API* servoAPI;
}

int currPWM = SERVO_DATA::ESC_MIL_MIN;
bool changePWM = false;
int step = 5;
int minStart = 1450;
const float filtK = 0.26;
long last_time = 0;
long time_step = 500;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(serialFreq);
  SERVO_DATA::servoAPI = new BLDC_API(SERVO_DATA::ESC_PIN, 
                                      SERVO_DATA::ESC_MIL_MIN, 
                                      SERVO_DATA::ESC_MIL_MAX,
                                      1700);
  //SERVO_DATA::servoAPI->writeMicroseconds(SERVO_DATA::ESC_MIL_MIN);   
  SERVO_DATA::servoAPI->writeMicroseconds(SERVO_DATA::ESC_MIL_MIN);  
  changePWM = false;                       
}



void loop() {
  // put your main code here, to run repeatedly:
  //SERVO_DATA::servoAPI->manualCalibration();
  if (Serial.available() > 0){ switchWorking(Serial.read()); } // read command to stop/start

  if (changePWM) {
    if (currPWM >= 1680) currPWM = 1640;
    SERVO_DATA::servoAPI->writeMicroseconds(currPWM);
    changePWM = false;
  }
  getAngle();
  getAngleFilt();

  if (millis() - last_time > time_step) {
     Serial.print("angle:"); Serial.print(Total_angle[1]); Serial.print(", "); 
     Serial.print("angle_filt:"); Serial.print(Total_angle_filt[1]); Serial.print(", "); 
     Serial.print("currPWM":); Serial.println(currPWM);
     last_time = millis();
  }

  //SERVO_DATA::servoAPI->autoCalibration();   
}

void switchWorking(int ch) {
    //Serial.read(); Serial.read();
    //FLAGS_WORK::startWorking = !FLAGS_WORK::startWorking;
    switch(ch)
    {
      case '1':
        changePWM = true;
        currPWM += step;
        break;
      case '2':
        changePWM = true;
        currPWM -= step;
      case '3': // STOP SIMPLE
        changePWM = true;
        currPWM = minStart;
        break;
      case '0': // STOP STOP STOP
        changePWN = true;
        currPWM = SERVO_DATA::ESC_MIL_MIN;
        break;
      //case '9':
        // плавная останова
      
    }
}







void getAngle()
{
   /* ========= read angle ============= */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 

  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();


 /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers  

  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  // offset
 // Gyr_rawX -= offsetMPU[3];
 // Gyr_rawY -= offsetMPU[4];

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;

/*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
}



void getAngleFilt()
{
  static SimpleLowPass filter1(filtK);
  static SimpleLowPass filter2(filtK);
  static SimpleLowPass filter3(filtK);
   /* ========= read angle ============= */

  Acc_rawX_filt = filter1.filter(Acc_rawX);
  Acc_rawY_filt = filter2.filter(Acc_rawY);
  Acc_rawZ_filt = filter3.filter(Acc_rawZ);

  Acceleration_angle_filt[0] = atan((Acc_rawY_filt/16384.0)/sqrt(pow((Acc_rawX_filt/16384.0),2) + pow((Acc_rawZ_filt/16384.0),2)))*rad_to_deg;
  Acceleration_angle_filt[1] = atan(-1*(Acc_rawX_filt/16384.0)/sqrt(pow((Acc_rawY_filt/16384.0),2) + pow((Acc_rawZ_filt/16384.0),2)))*rad_to_deg;


 Total_angle_filt[1] = 0.98 *(Total_angle_filt[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle_filt[1];
}









