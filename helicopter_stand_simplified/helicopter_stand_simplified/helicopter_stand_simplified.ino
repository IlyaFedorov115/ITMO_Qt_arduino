#include <Wire.h>
#include <Servo.h>
#include "filters.h"
/*** ================== FLAGS INCLUDE ================ ***/
#include "flags.h"


/* ================ BLDC SERVO SETTINGS ============== */
#define PWM_MIN (1200)
#define PWM_MAX (2000)
const byte ESC_PIN = 9;
Servo bldcEsc;


/* ============= WORKING VARIABLES =============== */
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_rawXf, Gyro_rawYf, Gyro_rawZf;
float Acc_rawX_filt, Acc_rawY_filt, Acc_rawZ_filt, Gyr_rawX_filt, Gyr_rawY_filt, Gyr_rawZ_filt;

float Acceleration_angle[2] = {0, 0};
float Gyro_angle[2] = {0, 0};
float Total_angle[2] = {0, 0};

float Acceleration_angle_filt[2];
float Gyro_angle_filt[2];
float Total_angle_filt[2];

float elapsedTime = 0.001;
float currTime, timePrev;
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


union FloatType {
  byte buf[4];
  float val;
};

FloatType sendData[BYTE_SEND_BUFFER_SIZE];
uint8_t sendBuffer[sizeof(sendData)];





/*
    ============================================================= SETUP FUNCTION =============================================================
*/
void setup() {

  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(serialFreq);

  bldcEsc.attach(ESC_PIN);                // set bldc
  bldcEsc.writeMicroseconds(PWM_MIN);

  // ******** get angle in loop  to prepare mpu6050 *****//
  elapsedTime = 0.001;
  Total_angle[0] = Total_angle_filt[0] = 0.0;
  for (int i = 0; i < 3000; i++) {
    getAngle();
    getAngleFilt();
    if (i % 100 == 0) { 
      Serial.print("angle:"); Serial.print(Total_angle[1]); Serial.print(", "); Serial.print("angle_filt:"); Serial.println(Total_angle_filt[1]); 
    }
  }

  delay(1000);
  currTime = millis();
  FLAGS_WORK::startWorking = false;
}






/*
    ============================================================ MAIN LOOP ==================================================================
*/
void loop() {
  timePrev = currTime;  
  currTime = millis();  
  elapsedTime = (currTime - timePrev) / 1000.0; 


  if (Serial.available() > 0){ switchWorking(Serial.read()); }    /* read command to stop/start  */

  if (!FLAGS_WORK::startWorking) return;                          /* set flag to do nothing */


  /* ============ READ ANGLE FROM MPU ============= */
#ifdef DEBUG_TIMERS
  Serial.print(F("elapsed: ")); Serial.println(time - timePrev); timeStartRead = millis();
#endif
  
  getAngle();
  getAngleFilt();  


  // ======= Если угол начинается с +40 и идет в 0, то false
  if (USE_FILT_ANGLE){
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle_filt[1]) : (Total_angle_filt[1] - desired_angle);
  } else {
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle[1]) : (Total_angle[1] - desired_angle);
  } 
  

  /* =========================== SAFETY SAFETY =========================== */
   //************** if (Total_angle[1] > 40) { stop(); return; } **********/
  if (error > ERROR_ANGLE_LIMIT || error < -ERROR_ANGLE_LIMIT || error != error) {  // error != error - check if nan and sensor failed
    smoothStop();
    return;
  }


  /* ============== calc signal ============== */
  PID_Step(error, elapsedTime);
  pwmLeft = throttle + EXPR_VARS::control_signal;


  if (pwmLeft < PWM_SEND_MIN) {
    pwmLeft = PWM_SEND_MIN;
  } else if (pwmLeft > PWM_SEND_MAX) {
    pwmLeft = PWM_SEND_MAX;
  }
 

  /* ==== send PWM ==== */
  bldcEsc.writeMicroseconds(pwmLeft);



  /* ========== ЛОГГИРОВАНИЕ ДАННЫХ ========== */

  #ifdef DEBUG_WORK_PRINT
  debugWorkPrint();
  #endif

  #ifdef DEBUG_WRITE_BYTE
  debugByte();
  #endif

}
/* ======================================== MAIN LOOP ======================================== */


void debugByte()
{
  if (EXPR_VARS::count_2_log != 0) EXPR_VARS::count_2_log--;
  else
  {
    sendData[0].val = Total_angle[1]; 
    sendData[1].val = Total_angle_filt[1]; 
    sendData[2].val = EXPR_VARS::control_signal; 
    sendData[3].val = EXPR_VARS::total_integral; 
    sendData[4].val = elapsedTime*1000;
    if (IS_DEBUG_PID) {
      sendData[PID_INDEX_START].val = EXPR_VARS::P_last_OUT; 
      sendData[PID_INDEX_START+1].val = EXPR_VARS::I_last_OUT; 
      sendData[PID_INDEX_START+2].val = EXPR_VARS::D_last_OUT; 
    }
    if (IS_DEBUG_RAW) {
      sendData[RAW_INDEX_START].val = Acc_rawX; sendData[RAW_INDEX_START+1].val = Acc_rawY; sendData[RAW_INDEX_START+2].val = Acc_rawZ;
      sendData[RAW_INDEX_START+3].val = Gyr_rawX; sendData[RAW_INDEX_START+4].val = Gyr_rawY; sendData[RAW_INDEX_START+5].val = Gyr_rawX;
    }

    memcpy(sendBuffer, (void*)sendData, sizeof(sendBuffer));
    Serial.write('#'); Serial.write('#');
    Serial.write(sendBuffer, sizeof(sendBuffer));
    EXPR_VARS::count_2_log = EXPR_VARS::LOG_EVERY_TIMES;
  }
}


void switchWorking(int ch) {
    switch(ch)
    {
      case START_BUTTON:
        if (FLAGS_WORK::startWorking || SMOOTH_SET) return;   // already start
        FLAGS_WORK::startWorking = true;
        /* !!!! CHECK MPU IS OK !!!! */
        start();
        break;
      case STOP_BUTTON:
        if (!FLAGS_WORK::startWorking || SMOOTH_SET) return;  // already stop
        FLAGS_WORK::startWorking = false;
        smoothStop();
        break;
      case PANIC_BUTTON1:                       // PANIC STOP BLDC !!!!
      case PANIC_BUTTON2:
        //if (!FLAGS_WORK::startWorking) return; // already stop
        stopPANIC();
        FLAGS_WORK::startWorking = SMOOTH_SET = false;
        break;
      case RESET_PID_BUTTON:
        EXPR_VARS::resetPid();
    }
}


void stopPANIC() {
  bldcEsc.writeMicroseconds(PWM_MIN);
  FLAGS_WORK::startWorking = false;
  reset();
}




void smoothStop() {     
  static long last_time_decrease;

  FLAGS_WORK::startWorking = false;
  SMOOTH_SET = true;

  int currPWM = (int)EXPR_VARS::control_signal + throttle;
  // start loop (maybe switchWorking make with return to break loop)
  last_time_decrease = millis();
  if (currPWM > SMOOTH_MAX_LIMIT) {
    currPWM = SMOOTH_MAX_LIMIT;
    bldcEsc.writeMicroseconds(currPWM);
  }

  while (currPWM > SMOOTH_STOP_PWM) {
    if (Serial.available() > 0){ 
      switchWorking(Serial.read()); 
      if (!SMOOTH_SET) break;    // set PANIC while smooth || currPWM > (EXPR_VARS::control_signal+throttle)
    } 
    
    if ((millis() - last_time_decrease) > SMOOTH_STEP_TIME_DECREASE)
    {
      currPWM -= SMOOTH_PWM_DECREASE; bldcEsc.writeMicroseconds(currPWM);
      last_time_decrease = millis();
    }
  }

  bldcEsc.writeMicroseconds(PWM_MIN);
  FLAGS_WORK::startWorking = false;
  reset();
  SMOOTH_SET = false;
}


void start() {
  /* ======================= CHECK MPU ADD ==================== */
  reset();
  FLAGS_WORK::startWorking = true;
}


void reset() {
  EXPR_VARS::resetPid();
  SMOOTH_SET = false;
  currTime = millis();
}




// =======================================================  MATHEMATICS ======================================================= //
void PID_Step(double error_, double dt) // dt - ms or sec
{
  if (dt < 0.0001) return;

  EXPR_VARS::total_integral += error_ * dt;   

  EXPR_VARS::P_last_OUT = pid_Kp * error_;                                /* limit every part of PID? */
  EXPR_VARS::I_last_OUT = pid_Ki * EXPR_VARS::total_integral;
  EXPR_VARS::D_last_OUT = pid_Kd * (error_ - EXPR_VARS::last_error)/dt;

  // init diff last error (prevent last = 0 and curr 40 -> too much)
  if (!EXPR_VARS::setLastError){
    EXPR_VARS::D_last_OUT = 0.0;
    EXPR_VARS::setLastError = true;
  }
  
  EXPR_VARS::control_signal = EXPR_VARS::P_last_OUT + EXPR_VARS::I_last_OUT + EXPR_VARS::D_last_OUT;

  EXPR_VARS::last_error = error_;

  // limit control
  if (EXPR_VARS::control_signal > max_PID_control) {
    EXPR_VARS::control_signal = max_PID_control;
  }
  if (EXPR_VARS::control_signal < min_PID_control) {
    EXPR_VARS::control_signal = min_PID_control;
  }
}



// https://github.com/rfetick/MPU6050_light/blob/master/src/MPU6050_light.cpp
// про минусы и прочее, если понадобится 

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





/* см. ФИЛЬТРАЦИЯ obsidian */

void getAngleFilt()
{
  static SimpleLowPass filter1(FILTER_COEF_ACCEL);
  static SimpleLowPass filter2(FILTER_COEF_ACCEL);
  static SimpleLowPass filter3(FILTER_COEF_ACCEL);
  static SimpleLowPass filterAngle(FILTER_ANGLE);

  Acc_rawX_filt = filter1.filter(Acc_rawX);
  Acc_rawY_filt = filter2.filter(Acc_rawY);
  Acc_rawZ_filt = filter3.filter(Acc_rawZ);

  Acceleration_angle_filt[0] = atan((Acc_rawY_filt/16384.0)/sqrt((Acc_rawX_filt/16384.0)*(Acc_rawX_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;
  Acceleration_angle_filt[1] = atan(-1*(Acc_rawX_filt/16384.0)/sqrt((Acc_rawY_filt/16384.0)*(Acc_rawY_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;

  Acceleration_angle_filt[0] -= AccErrorX_calcFilt;
  Acceleration_angle_filt[1] -= AccErrorY_calcFilt;

  Gyro_angle_filt[1] = Gyro_angle_filt[1] + Gyro_rawYf * elapsedTime;
  ////Gyro_angle_filt[1] = COEF_GYRO_COMP * Gyro_angle_filt[1] + (1-COEF_GYRO_COMP) * Acceleration_angle_filt[1];

  //Gyro_angle_filt[1] = COEF_GYRO_COMP * Gyro_angle_filt[1] + (1-COEF_GYRO_COMP) * Acceleration_angle[1];
  ///Gyro_angle_filt[1] = filterAngle.filter(Gyro_angle_filt[1]);
  Gyro_angle_filt[1] = filterAngle.filter(Gyro_angle[1]);
  Total_angle_filt[1] = Gyro_angle_filt[1];
  
  //Total_angle_filt[1] = filterAngle.filter(Total_angle_filt[1]);
}



void debugWorkPrint()  /* Serial.print angles and pid */
{

  Serial.print("angle:"); Serial.print(Total_angle[1]);           Serial.print(", "); 
  Serial.print("angle_filt:"); Serial.print(Total_angle_filt[1]); Serial.print(", "); 
  Serial.print("pid:"); Serial.print(EXPR_VARS::control_signal);  Serial.print(", ");
  Serial.print("dt:"); Serial.print(elapsedTime*1000);            Serial.print(", ");
  Serial.print("err:"); Serial.print(error);            Serial.print(", ");
  Serial.print("interSum:"); Serial.print(EXPR_VARS::total_integral);                Serial.print(", ");
  if (IS_DEBUG_PID){
    Serial.print("P:"); Serial.print(EXPR_VARS::P_last_OUT);      Serial.print(", ");
    Serial.print("D:"); Serial.print(EXPR_VARS::D_last_OUT);      Serial.print(", ");
    Serial.print("I:"); Serial.print(EXPR_VARS::I_last_OUT);      Serial.print(", ");
  }
  if (IS_DEBUG_RAW) {
    Serial.print("accX:"); Serial.print(Acc_rawX);                Serial.print(", ");
    Serial.print("accY:"); Serial.print(Acc_rawY);                Serial.print(", ");
    Serial.print("accZ:"); Serial.print(Acc_rawZ);                Serial.print(", ");
    Serial.print("gyrX:"); Serial.print(Gyr_rawX);                Serial.print(", ");
    Serial.print("gyrY:"); Serial.print(Gyr_rawY);                Serial.print(", ");
  }
  Serial.println();
}



float filt(float pot){
  static const float K = 0.25;
  static float val = 0;
  val = val*(1-K) + pot*K;
  return val;
}



/*
// ДРЕЙФ ЕСТЬ /
inline void getAngleAlter()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);                     //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 

  Acc_rawX=Wire.read()<<8|Wire.read(); //  needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();

 // Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;    //---X---//
 // Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg; //---Y---//

  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt((Acc_rawX/16384.0)*(Acc_rawX/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;  //---X---//
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt((Acc_rawY/16384.0)*(Acc_rawY/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg; //---Y---//


  Acceleration_angle[0] -= AccErrorX_calc;
  Acceleration_angle[1] -= AccErrorY_calc;

  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); //Just 4 registers  

  Gyr_rawX = Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read()<<8|Wire.read();
  Gyr_rawZ = Wire.read()<<8|Wire.read();

  Gyro_rawXf = (Gyr_rawX/131.0) - GyroErrorX_calc;   //---X---//
  Gyro_rawYf = (Gyr_rawY/131.0) - GyroErrorY_calc;   //---Y---//
  Gyro_angle[1] = Gyro_angle[1] + Gyro_rawYf * elapsedTime;   //---Y---//
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_rawYf * elapsedTime) + 0.02 * (Acceleration_angle[1]);
 // Total_angle[1] = COEF_ACCEL_COMP * (Total_angle[1] + Gyro_angle[1]*elapsedTime) + (1-COEF_ACCEL_COMP) * Acceleration_angle[1];  //---Y axis angle---//
  //Total_angle[1] = COEF_ACCEL_COMP * (Gyro_angle[1]) + (1-COEF_ACCEL_COMP) * Acceleration_angle[1];
}
*/