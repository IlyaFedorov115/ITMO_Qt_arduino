#include <Wire.h>
//#include <MPU6050.h>

#include "bldc_api.h"
#include "filters.h"

#define DEBUG_PRINT
//#define DEBUG_WRITE


const float AccErrorX_calc = -0.01;
const float AccErrorY_calc = -3.24;
const float GyroErrorX_calc = -0.36;
const float GyroErrorY_calc = 2.27;
const float GyroErrorZ_calc = -1.64;

const float AccErrorX_calcFilt = -0.01;
const float AccErrorY_calcFilt = -3.24;
const float GyroErrorX_calcFilt = -0.36;
const float GyroErrorY_calcFilt = 2.27;
const float GyroErrorZ_calcFilt = -1.64;

const float COEF_GYRO_COMP = 0.99;


const long serialFreq = 115200;

/*** MPU ***/
//int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
//int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_rawXf, Gyro_rawYf, Gyro_rawZf;
float Acc_rawX_filt, Acc_rawY_filt, Acc_rawZ_filt, Gyr_rawX_filt, Gyr_rawY_filt, Gyr_rawZ_filt;

float Acceleration_angle[2] = {0, 0};
float Gyro_angle[2] = {0, 0};
float Total_angle[2] = {0, 0};

float Acceleration_angle_filt[2];
float Gyro_angle_filt[2];
float Total_angle_filt[2];
float rad_to_deg = 180/3.141592654;

namespace SERVO_DATA {
  const int ESC_PIN = 9;  //D9
  const int ESC_MIL_MIN = 1000;
  const int ESC_MIL_MAX = 2000;
  Servo servo;
}

int currPWM = SERVO_DATA::ESC_MIL_MIN;
bool changePWM = false;
int step = 5;
int minStart = 1400;
const float filtK = 0.26;
long last_time = 0;
long time_step = 15;


float elapsedTime, time, timePrev;
const int wireTimeout = 3000;

union FloatType {
  byte buf[4];
  float val;
};

FloatType sendData[5];
uint8_t sendBuffer[sizeof(sendData)];

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
  SERVO_DATA::servo.attach(SERVO_DATA::ESC_PIN);


  SERVO_DATA::servo.writeMicroseconds(SERVO_DATA::ESC_MIL_MIN);  


  bool waiting = true;
  char data;

  Serial.println("Connect battery and type 0 to start");

  while (waiting) {  
  if (Serial.available()) {    
    data = Serial.read();

    switch(data) {
      case 48: {
        waiting = false;
        Serial.println("Starting in two seconds...");
        delay(2000);
        break;
      }
    }
  } 
  }



  changePWM = false;                       
}



void loop() {

  //SERVO_DATA::servoAPI->manualCalibration();
  if (Serial.available() > 0){ switchWorking(Serial.read()); } // read command to stop/start

  timePrev = time;  
  time = millis();  
  elapsedTime = (time - timePrev) / 1000.0; 

  // write when update in button
  if (changePWM) {
    if (currPWM >= 1620) currPWM = 1620;
    SERVO_DATA::servo.writeMicroseconds(currPWM);
    changePWM = false;
  }


  getAngle();
  getAngleFilt();

  if ((millis() - last_time) > time_step) {
    #ifdef DEBUG_WRITE
    sendData[0].val = Total_angle[1]; sendData[1].val = Total_angle_filt[1]; sendData[2].val = currPWM; 
    sendData[3].val = 0; sendData[4].val = time_step;
    memcpy(sendBuffer, (void*)sendData, sizeof(sendBuffer));
    Serial.write('#'); Serial.write('#');
    Serial.write(sendBuffer, sizeof(sendBuffer));

    #endif
    /*
     Serial.print("angle1:"); Serial.print(Total_angle[1]); Serial.print(", "); 
     Serial.print("angle2:"); Serial.print(Total_angle[0]); Serial.print(", "); 
     Serial.print("angle_filt:"); Serial.print(Total_angle_filt[1]); Serial.print(", "); 
     Serial.print("currPWM:"); Serial.println(currPWM);
     */
     #ifdef DEBUG_PRINT
     Serial.print("currPWM: "); Serial.print(currPWM); Serial.print(" angle: "); Serial.println(Total_angle[1]);
     #endif
     last_time = millis();
  }

}



void switchWorking(int ch) {
    //Serial.read(); Serial.read();
    //FLAGS_WORK::startWorking = !FLAGS_WORK::startWorking;
    static bool smooth = false;
    switch(ch)
    {
      case '1':           // + step
        changePWM = true;
        currPWM += step;
        break;
      case '2':
        changePWM = true; // - step
        if (currPWM > (SERVO_DATA::ESC_MIL_MIN+2*step)) currPWM -= step;
        break;
      case '3': // SET MIN START
        changePWM = true;
        currPWM = minStart;
        break;
      case '0': // STOP STOP STOP
        changePWM = true;
        currPWM = SERVO_DATA::ESC_MIL_MIN;
        SERVO_DATA::servo.writeMicroseconds(currPWM);
        break;
      case '4':
        if (smooth) return;
        smooth = true;
        smoothStop();
        smooth = false;
        break;
      
    }
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


void smoothStop() {
  static const long step_time_decrease = 250;
  static const int stop_pwm = SERVO_DATA::ESC_MIL_MAX*1.1;
  static const int step_pwm_decrease = 5;
  static long last_time_decrease;

  // start loop (maybe switchWorking make with return to break loop)
  last_time_decrease = millis();
  if (currPWM > 1610) {
    currPWM = 1610;
    SERVO_DATA::servo.writeMicroseconds(currPWM);
  }

  while(true)
  {
    if (Serial.available() > 0){ switchWorking(Serial.read()); } 

    if ((millis() - last_time_decrease) > step_time_decrease)
    {
      Serial.println("Decrease");
      currPWM -= step_pwm_decrease;
      SERVO_DATA::servo.writeMicroseconds(currPWM);
      last_time_decrease = millis();
      
      if (currPWM < stop_pwm){
        currPWM = SERVO_DATA::ESC_MIL_MIN;
        SERVO_DATA::servo.writeMicroseconds(currPWM);
        return;
      }

    }
  }

}








