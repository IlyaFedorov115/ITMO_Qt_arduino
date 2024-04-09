#include <Wire.h>
#include <Servo.h>
#include "filters.h"
#include "flags.h"

const bool DEBUG_SWITCH_GET = true;

/* ============= WORKING VARIABLES =============== */
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acc_rawX_filt, Acc_rawY_filt, Acc_rawZ_filt, Gyr_rawX_filt, Gyr_rawY_filt, Gyr_rawZ_filt;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float Acceleration_angle_filt[2];
float Gyro_angle_filt[2];
float Total_angle_filt[2];

float elapsedTime, time, timePrev;
const float rad_to_deg = 180/3.141592654;
float timeStartRead;
float pwmLeft, error, previous_error;

// ================ ФЛАГИ РАБОТЫ И ПЕРЕКЛЮЧЕНИЯ ======= //
namespace FLAGS_WORK {
  bool startWorking = false;  
  bool HARDWARE_STATUS = false;
}
const int wireTimeout = 3000;
const long serialFreq = 250000;

long timeReadAngle = 0;

union FloatType {
  byte buf[4];
  float val;
};

FloatType sendData[5];
uint8_t sendBuffer[sizeof(sendData)];

union Data {
  char buffer[5*sizeof(float)];
  struct {
    float angle;
    float angle_filt;
    float pwm;
    float inter;
    float dt;

    
  };
};

Data data;

void setup() {
    Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(serialFreq);

  time = millis();
  // add checking if angle < 0 and add checking if WIRE mpu working or ignore
  // in infinity loop
  // for(;;)

  FLAGS_WORK::startWorking = true;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  timePrev = time;  
  time = millis();  
  elapsedTime = (time - timePrev) / 1000.0; 

    // read command to stop/start
  if (Serial.available() > 0){ switchWorking(Serial.read()); } 

   // was set flag to do nothing STOP
  if (!FLAGS_WORK::startWorking) return;

  timeReadAngle = micros();

  getAngle();
  getAngleFilt();


  
   if (USE_FILT_ANGLE){
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle_filt[1]) : (Total_angle_filt[1] - desired_angle);
  } else {
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle[1]) : (Total_angle[1] - desired_angle);
  } 
  PID_Step(error, elapsedTime);
 
 // Serial.print("Read and calc filtering: "); Serial.println(micros() - timeReadAngle);


  timeReadAngle = micros();

  
  data.angle = Total_angle[1]; data.angle_filt = Total_angle_filt[1];
  data.pwm = EXPR_VARS::control_signal; data.inter = EXPR_VARS::total_integral;
  data.dt = elapsedTime*1000;
  //Serial.write(data.buffer, sizeof(data));

  sendData[0].val = Total_angle[1]; sendData[1].val = Total_angle_filt[1]; sendData[2].val = EXPR_VARS::control_signal; 
  sendData[3].val = EXPR_VARS::total_integral; sendData[4].val = elapsedTime*1000;
  memcpy(sendBuffer, (void*)sendData, sizeof(sendBuffer));
  Serial.write(sendBuffer, sizeof(sendBuffer));
 /*
  Serial.print(data.dt); Serial.print(', '); 
   Serial.print(data.angle); Serial.print(', ');
    Serial.print(data.angle_filt); Serial.print(', ');
     Serial.print(data.pwm); Serial.print(', ');
      Serial.println(data.inter); 
*/

 // Serial.print(Total_angle[1]); Serial.print(','); Serial.print(Total_angle_filt[1]); Serial.print(','); Serial.print(EXPR_VARS::control_signal); Serial.print(',');
  //   Serial.print(EXPR_VARS::total_integral); Serial.print(','); Serial.println(elapsedTime*1000); 

  /*
  Serial.write((char*)&Total_angle[1], sizeof(Total_angle[1])); Serial.write(',');
  Serial.write((char*)&Total_angle_filt[1], sizeof(Total_angle_filt[1])); Serial.write(',');
  Serial.write((char*)&EXPR_VARS::control_signal, sizeof(EXPR_VARS::control_signal)); Serial.write(',');
  Serial.write((char*)&EXPR_VARS::total_integral, sizeof(EXPR_VARS::total_integral)); Serial.write(',');
  Serial.write((char*)&elapsedTime, sizeof(elapsedTime)); Serial.write('\n');
  */

  //Serial.print("WRITE TIME: ");  Serial.println(micros() - timeReadAngle);


}



void switchWorking(int ch) {
    //Serial.read(); Serial.read();
    switch(ch)
    {
      case START_BUTTON:
        if (FLAGS_WORK::startWorking) return;   // already start
        FLAGS_WORK::startWorking = true;
        if(DEBUG_SWITCH_GET) Serial.println("GET START");
        break;
      case STOP_BUTTON:
        if (!FLAGS_WORK::startWorking ) return;  // already stop
        FLAGS_WORK::startWorking = false;
        //if(DEBUG_SWITCH_GET) Serial.println("GET STOP");
        break;
      case PANIC_BUTTON1:                       // PANIC STOP BLDC !!!!
      case PANIC_BUTTON2:
        if(DEBUG_SWITCH_GET) Serial.println("PANIC GET");
        FLAGS_WORK::startWorking  = false;
        break;
      case RESET_PID_BUTTON:
        if(DEBUG_SWITCH_GET) Serial.println("RESET GET");
        EXPR_VARS::resetPid();
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

  // offset
//  Acc_rawX -= offsetMPU[0];
//  Acc_rawY -= offsetMPU[1];
//  Acc_rawZ -= offsetMPU[2];
 // Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;    /*---X---*/
 // Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg; /*---Y---*/

  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt((Acc_rawX/16384.0)*(Acc_rawX/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;  /*---X---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt((Acc_rawY/16384.0)*(Acc_rawY/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg; /*---Y---*/

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers  

  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

  // offset
 // Gyr_rawX -= offsetMPU[3];
 // Gyr_rawY -= offsetMPU[4];

  Gyro_angle[0] = Gyr_rawX/131.0;   /*---X---*/
  Gyro_angle[1] = Gyr_rawY/131.0;    /*---Y---*/

  Total_angle[0] = COEF_ACCEL_COMP *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + (1-COEF_ACCEL_COMP) * Acceleration_angle[0];  /*---X axis angle---*/
  Total_angle[1] = COEF_ACCEL_COMP *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + (1-COEF_ACCEL_COMP) * Acceleration_angle[1];  /*---Y axis angle---*/
}





void getAngleFilt()
{
  static SimpleLowPass filter1(FILTER_COEF_ACCEL);
  static SimpleLowPass filter2(FILTER_COEF_ACCEL);
  static SimpleLowPass filter3(FILTER_COEF_ACCEL);

  Acc_rawX_filt = filter1.filter(Acc_rawX);
  Acc_rawY_filt = filter2.filter(Acc_rawY);
  Acc_rawZ_filt = filter3.filter(Acc_rawZ);

  Acceleration_angle_filt[0] = atan((Acc_rawY_filt/16384.0)/sqrt((Acc_rawX_filt/16384.0)*(Acc_rawX_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;
  Acceleration_angle_filt[1] = atan(-1*(Acc_rawX_filt/16384.0)/sqrt((Acc_rawY_filt/16384.0)*(Acc_rawY_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;
 // Acceleration_angle_filt[0] = filter1.filter(Acceleration_angle[0]);
 // Acceleration_angle_filt[1] = filter2.filter(Acceleration_angle[1]);
  //   Total_angle_filt[0] = 0.98 *(Total_angle_filt[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle_filt[0]; /*---X axis angle---*/
 //  Total_angle_filt[1] = 0.98 *(Total_angle_filt[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle_filt[1]; /*---Y axis angle---*/
  //Total_angle_filt[0] = filter1.filter(Total_angle[0]);

  /**** !!!!! НЕ ДОБАВЛЯТЬ OFFSET В ИТОГОВЫЙ ФИЛЬТР ИНАЧЕ УГОЛ СОВСЕМ ДРУГОЙ 90 вместо +40 ***/
 Total_angle_filt[1] = COEF_ACCEL_COMP*(Total_angle_filt[1] + Gyro_angle[1]*elapsedTime) + (1-COEF_ACCEL_COMP)*Acceleration_angle_filt[1];
}


void PID_Step(double error_, double dt)
{
  if (dt < 0.0001) {  return; }

  EXPR_VARS::total_integral += error_ * dt;   

  EXPR_VARS::control_signal = pid_Kp * error_ + 
        pid_Ki * EXPR_VARS::total_integral +
        pid_Kd * (error_ - EXPR_VARS::last_error)/dt;

  EXPR_VARS::last_error = error_;

  // limit control
  if (EXPR_VARS::control_signal > max_PID_control) {
    EXPR_VARS::control_signal = max_PID_control;
  }
  if (EXPR_VARS::control_signal < min_PID_control) {
    EXPR_VARS::control_signal = min_PID_control;
  }

  #ifdef DEBUG_PID
    Serial.print(F("pid\t"));
    Serial.print(EXPR_VARS::control_signal); Serial.print("\t");
    Serial.print(error); Serial.print("\t");
    Serial.print(dt); Serial.print("\t");
    Serial.println(EXPR_VARS::total_integral);
  #endif
  
}
