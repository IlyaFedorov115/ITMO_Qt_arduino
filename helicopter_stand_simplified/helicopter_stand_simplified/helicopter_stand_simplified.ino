#include <Wire.h>
#include <Servo.h>
#include "filters.h"
#define PWM_MIN (1200)
#define PWM_MAX (2000)

//#define DEBUG_WORK
//#define DEBUG_TIMERS
#define DEBUG_WRITE_BYTE


const byte ESC_PIN = 9;
Servo bldcEsc;



//int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
const int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acc_rawX_filt, Acc_rawY_filt, Acc_rawZ_filt, Gyr_rawX_filt, Gyr_rawY_filt, Gyr_rawZ_filt;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float Acceleration_angle_filt[2];
float Gyro_angle_filt[2];
float Total_angle_filt[2];

float elapsedTime, time, timePrev;
int i;
const float rad_to_deg = 180/3.141592654;

float timeStartRead;


const double throttle=1550; //initial value of throttle to the motors
const float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
const double kp=1.8;//3.55
const double ki=0.1;//0.003
const double kd=0.7;//2.05

const double pid_Kp = kp;//0.6*1.25
const double pid_Ki = ki;//pid_Kp*2 / 1000 0.006;    // ms
const double pid_Kd = kd;//300;


namespace EXPR_VARS {
  double total_integral = 0;
  double last_error = 0;
  double control_signal;
  void resetPid(){
    total_integral = 0; last_error = 0; control_signal = 0;
  }
}
const double min_PID_control = -800;                       /* !! Attention !! */
const double max_PID_control = 800;  


namespace FLAGS_WORK {
  bool startWorking = true;  
  bool HARDWARE_STATUS = false;
}
const int wireTimeout = 3000;


/*
* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* NEED TO SET OFFSETS
* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/


void setup() {



  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115500);

  bldcEsc.attach(ESC_PIN);
  bldcEsc.writeMicroseconds(PWM_MIN);


  //********** 100 times ********* //
  // ******** get angle in loop ********//
  // ****** to prepare mpu6050 *****//
  for (int i = 0; i < 3000; i++) {
    getAngle();
    getAngleFilt();
  }


  delay(1000);

  time = millis();


  // add checking if angle < 0 and add checking if WIRE mpu working or ignore
  // in infinity loop
  // for(;;)

}

void loop() {
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000.0; 

if (Serial.available() > 0){ switchWorking(Serial.read()); } // read command to stop/start
  // set flag to do nothing
if (!FLAGS_WORK::startWorking) return;

#ifdef DEBUG_TIMERS
  Serial.print(F("elapsed: ")); Serial.println(time - timePrev);
  timeStartRead = millis();
#endif

  /* ========= read angle ============= */

  getAngle();
  getAngleFilt();



#ifdef DEBUG_TIMERS
  Serial.print(F("elapsedGetAngle: ")); Serial.println(millis() - timeStartRead);
#endif


 //************** if (Total_angle[1] > 40) { stop(); return; } **********/

  /****** START -70, hor = 0 ****/
  //error = desired_angle - Total_angle[1];//Total_angle[1] - desired_angle;
 // error = Total_angle[1] - desired_angle;
  error = Total_angle_filt[1] - desired_angle;
  /*
pid_p = kp*error;
if(-3 <error <3)
{
  pid_i = pid_i+(ki*error);  
}
pid_d = kd*((error - previous_error)/elapsedTime);
PID = pid_p + pid_i + pid_d;
if(PID < -800){
  PID=-800;
}
if(PID > 800){
  PID=800;
}
pwmLeft = throttle + PID;

  */
  PID_Step(error, elapsedTime);
  pwmLeft = throttle + EXPR_VARS::control_signal;

  if (pwmLeft < 1200) {
    pwmLeft = 1200;
  } else if (pwmLeft > 1700) {
    pwmLeft = 1700;
  }
 
 // pwmLeft = 1614;
  bldcEsc.writeMicroseconds(pwmLeft);
  previous_error = error;

 // #ifdef DEBUG_WORK
 // Serial.print("pwm_pid_err: "); Serial.print(pwmLeft); Serial.print(' '); Serial.print(EXPR_VARS::control_signal); Serial.print(' '); Serial.println(error);
 // #endif

  #ifdef DEBUG_WORK
  Serial.print("angle:"); Serial.print(Total_angle[1]); Serial.print(", "); 
  Serial.print("angle_filt:"); Serial.print(Total_angle_filt[1]); Serial.print(", "); 
  Serial.print("pid:"); Serial.print(EXPR_VARS::control_signal); Serial.print(", ");
  Serial.print("pwmLeft:"); Serial.println(pwmLeft);
  #endif

  #ifdef DEBUG_WRITE_BYTE
  Serial.write((char*)&Total_angle[1], sizeof(Total_angle[1])); Serial.write(',');
  Serial.write((char*)&Total_angle_filt[1], sizeof(Total_angle_filt[1])); Serial.write(',');
  Serial.write((char*)&EXPR_VARS::control_signal, sizeof(EXPR_VARS::control_signal)); Serial.write(',');
  Serial.write((char*)&pwmLeft, sizeof(pwmLeft)); Serial.write('\n');
  #endif

}



// dt - ms or sec
void PID_Step(double error_, double dt)
{
  if (dt < 0.0001) return;
  EXPR_VARS::total_integral += error_ * dt;             
  EXPR_VARS::control_signal = pid_Kp * error_ + 
        pid_Ki * EXPR_VARS::total_integral +
        pid_Kd * (error_ - EXPR_VARS::last_error)/dt;
  //EXPR_VARS::total_integral += error_ * dt;             // before calc
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



void switchWorking(int ch) {
    //Serial.read(); Serial.read();
    //FLAGS_WORK::startWorking = !FLAGS_WORK::startWorking;
    switch(ch)
    {
      case '0':
      case '1':
        FLAGS_WORK::startWorking = !FLAGS_WORK::startWorking;
        if (FLAGS_WORK::startWorking == false) stop();
        else start();
        break;
      case '2':
        EXPR_VARS::resetPid();
    }
}



void stop() {
  //bldcEsc.stop();
  bldcEsc.writeMicroseconds(PWM_MIN);
  FLAGS_WORK::startWorking = false;
  reset();
  //FLAGS_WORK::startWorking = false;
}

void start() {
  //saf_reset();
  //timersStart();
  reset();
  //bldcEsc.speed(PWM_MIN);
 // FLAGS_WORK::startWorking = true;
}


void reset() {
  EXPR_VARS::resetPid();
  //saf_reset();
  //mpuObject.resetFIFO();
}



inline void getAngle()
{
   /* ========= read angle ============= */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 

  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();


  // offset
//  Acc_rawX -= offsetMPU[0];
//  Acc_rawY -= offsetMPU[1];
//  Acc_rawZ -= offsetMPU[2];


 // Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

 // Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

 /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt((Acc_rawX/16384.0)*(Acc_rawX/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt((Acc_rawY/16384.0)*(Acc_rawY/16384.0) + (Acc_rawZ/16384.0)*(Acc_rawZ/16384.0)))*rad_to_deg;

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





float filt(float pot){
  static const float K = 0.25;
  static float val = 0;
  val = val*(1-K) + pot*K;
  return val;
}


void getAngleFilt()
{
  static SimpleLowPass filter1(0.26);
  static SimpleLowPass filter2(0.26);
  static SimpleLowPass filter3(0.26);
   /* ========= read angle ============= */

  Acc_rawX_filt = filter1.filter(Acc_rawX);
  Acc_rawY_filt = filter2.filter(Acc_rawY);
  Acc_rawZ_filt = filter3.filter(Acc_rawZ);

  //Acceleration_angle_filt[0] = atan((Acc_rawY_filt/16384.0)/sqrt(pow((Acc_rawX_filt/16384.0),2) + pow((Acc_rawZ_filt/16384.0),2)))*rad_to_deg;
  //Acceleration_angle_filt[1] = atan(-1*(Acc_rawX_filt/16384.0)/sqrt(pow((Acc_rawY_filt/16384.0),2) + pow((Acc_rawZ_filt/16384.0),2)))*rad_to_deg;

  Acceleration_angle_filt[0] = atan((Acc_rawY_filt/16384.0)/sqrt((Acc_rawX_filt/16384.0)*(Acc_rawX_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;
  Acceleration_angle_filt[1] = atan(-1*(Acc_rawX_filt/16384.0)/sqrt((Acc_rawY_filt/16384.0)*(Acc_rawY_filt/16384.0) + (Acc_rawZ_filt/16384.0)*(Acc_rawZ_filt/16384.0)))*rad_to_deg;

 /*---X---*/
//  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
//  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

 // Acceleration_angle_filt[0] = filter1.filter(Acceleration_angle[0]);
 // Acceleration_angle_filt[1] = filter2.filter(Acceleration_angle[1]);

/*---X axis angle---*/
//   Total_angle_filt[0] = 0.98 *(Total_angle_filt[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle_filt[0];
   /*---Y axis angle---*/
 //  Total_angle_filt[1] = 0.98 *(Total_angle_filt[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle_filt[1];

   //Total_angle_filt[0] = filterObjAccelAngle1.filter(Total_angle[0]);
  // Total_angle_filt[1] = filterObjAccelAngle1.filter(Total_angle[1]);
 // Total_angle_filt[1] = filt(Total_angle[1]);
 Total_angle_filt[1] = 0.98 *(Total_angle_filt[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle_filt[1];
}


