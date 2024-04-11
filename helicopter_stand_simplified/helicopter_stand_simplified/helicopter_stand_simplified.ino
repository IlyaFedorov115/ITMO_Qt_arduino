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


union FloatType {
  byte buf[4];
  float val;
};

FloatType sendData[5];
uint8_t sendBuffer[sizeof(sendData)];


/*
    ======================================== SETUP FUNCTION ========================================
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

  // set bldc
  bldcEsc.attach(ESC_PIN);
  bldcEsc.writeMicroseconds(PWM_MIN);

  // ******** get angle in loop ********//
  // ****** to prepare mpu6050 *****//
  // ****** check is ok ****** //
  Total_angle[0] = Total_angle_filt[0] = 0.0;
  for (int i = 0; i < 3000; i++) {
    getAngle();
    getAngleFilt();
    if (i % 50 == 0){
      Serial.print("angle:"); Serial.print(Total_angle[1]); Serial.print(", "); 
      Serial.print("angle_filt:"); Serial.println(Total_angle_filt[1]); 
    }
  }

  delay(1000);
  time = millis();
  // add checking if angle < 0 and add checking if WIRE mpu working or ignore
  // in infinity loop
  // for(;;)

  FLAGS_WORK::startWorking = false;
}




/*
    ======================================== MAIN LOOP ========================================
*/
void loop() {
  timePrev = time;  
  time = millis();  
  elapsedTime = (time - timePrev) / 1000.0; 

  // read command to stop/start
  if (Serial.available() > 0){ switchWorking(Serial.read()); } 

  // was set flag to do nothing STOP
  if (!FLAGS_WORK::startWorking) return;



  /* ============ READ ANGLE FROM MPU ============= */
#ifdef DEBUG_TIMERS
  Serial.print(F("elapsed: ")); Serial.println(time - timePrev);
  timeStartRead = millis();
#endif
  
  getAngle();
  getAngleFilt();

#ifdef DEBUG_TIMERS
  Serial.print(F("elapsedGetAngle: ")); Serial.println(millis() - timeStartRead);
#endif



  // ======= Если угол начинается с +40 и идет в 0, то false
  if (USE_FILT_ANGLE){
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle_filt[1]) : (Total_angle_filt[1] - desired_angle);
  } else {
    error = (CURR_ANGLE_INCREASE) ? (desired_angle - Total_angle[1]) : (Total_angle[1] - desired_angle);
  } // error = Total_angle[1] - desired_angle;
  
  //error += OFFSET_ANGLE;


  /* =========================== SAFETY SAFETY =========================== */
   //************** if (Total_angle[1] > 40) { stop(); return; } **********/
  if (error > ERROR_ANGLE_LIMIT || error < -ERROR_ANGLE_LIMIT || error != error) {  // error != error - check if nan and sensor failed
    smoothStop();
    return;
  }


  /* === calc signal === */
  PID_Step(error, elapsedTime);
  pwmLeft = throttle + EXPR_VARS::control_signal;

  if (pwmLeft < PWM_SEND_MIN) {
    pwmLeft = PWM_SEND_MIN;
  } else if (pwmLeft > PWM_SEND_MAX) {
    pwmLeft = PWM_SEND_MAX;
  }
 

  /* ==== send PWM ==== */
  bldcEsc.writeMicroseconds(pwmLeft);
  /* ==== send PWM ==== */



  /* ========== ЛОГГИРОВАНИЕ ДАННЫХ ========== */

  #ifdef DEBUG_WORK
  Serial.print("angle:"); Serial.print(Total_angle[1]); Serial.print(", "); 
  Serial.print("angle_filt:"); Serial.print(Total_angle_filt[1]); Serial.print(", "); 
  Serial.print("pid:"); Serial.print(EXPR_VARS::control_signal); Serial.print(", ");
  Serial.print("pwmLeft:"); Serial.println(pwmLeft);
  #endif

  #ifdef DEBUG_WRITE_BYTE
  /*
  Serial.write((char*)&Total_angle[1], sizeof(Total_angle[1])); Serial.write(',');
  Serial.write((char*)&Total_angle_filt[1], sizeof(Total_angle_filt[1])); Serial.write(',');
  Serial.write((char*)&EXPR_VARS::control_signal, sizeof(EXPR_VARS::control_signal)); Serial.write(',');
  Serial.write((char*)&pwmLeft, sizeof(pwmLeft)); Serial.write('\n');
  */
  // maybe 1 in 5 times, not every itteration
  sendData[0].val = Total_angle[1]; sendData[1].val = Total_angle_filt[1]; sendData[2].val = EXPR_VARS::control_signal; 
  sendData[3].val = EXPR_VARS::total_integral; sendData[4].val = elapsedTime*1000;
  memcpy(sendBuffer, (void*)sendData, sizeof(sendBuffer));
  Serial.write('#'); Serial.write('#');
  Serial.write(sendBuffer, sizeof(sendBuffer));
  #endif

}
/* ======================================== MAIN LOOP ======================================== */



void switchWorking(int ch) {
    //Serial.read(); Serial.read();
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
  //bldcEsc.stop();
  bldcEsc.writeMicroseconds(PWM_MIN);
  FLAGS_WORK::startWorking = false;
  reset();
}




void smoothStop() {     // "мягкий стоп"
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
  //saf_reset(); //timersStart();
  /* ======================= CHECK MPU ADD ==================== */
  // else DEBUG PRINT 
  reset();
  FLAGS_WORK::startWorking = true;
  //bldcEsc.speed(PWM_MIN);
 // FLAGS_WORK::startWorking = true;
}


void reset() {
  EXPR_VARS::resetPid();
  SMOOTH_SET = false;
  //saf_reset();
  //mpuObject.resetFIFO();
}




// =======================================================  MATHEMATICS ======================================================= //
// =======================================================  MATHEMATICS ======================================================= //
// dt - ms or sec
void PID_Step(double error_, double dt)
{
  if (dt < 0.0001) return;

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

  //Total_angle[0] = COEF_ACCEL_COMP *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + (1-COEF_ACCEL_COMP) * Acceleration_angle[0];  /*---X axis angle---*/
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




float filt(float pot){
  static const float K = 0.25;
  static float val = 0;
  val = val*(1-K) + pot*K;
  return val;
}


