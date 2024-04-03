#include "ESC.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "supp.h"



/* --------------- SETTINGS CONSTANTS ------------ */
const int MPU_addr = 0x68; // адрес датчика
const long serialFreq = 115200;
const int wireTimeout = 3000;
#define SETUP_WIRE
#define WIRE_HAS_TIMEOUT 1
#define DEBUG_HW               // Debug hardware status (MPU6050) 
//#define DEBUG_PID
//#define DEBUG_ANGLE
#define DEBUG_SAFETY_BREACHED  // Debug when danger zone 
#define DEBUG_PLOTTING_VIEW    // Debug for plotting in Arduino ide
//#define DEBUG_WORK_PROCESS   // debug timers 
#define USE_DMP_MPU         // using dmp get angles (10 ms + 3 to get angle)
#define PIN_LOG_TRUE        // pin switch every secind while loop()



// for mpu2 Start = +72, horisontal = 0 -> decrease

//#define CHECK_BLDC_BEFORE_START  // run bldc from to for test

/* -------------- ESC settings ---------------- */
#define PWM_MIN (1200)
#define PWM_MAX (2000)
const byte ESC_PIN = 9;
ESC bldcEsc(ESC_PIN, PWM_MIN, PWM_MAX);
int Speed2Send = 0;

/* -------------- MPU6050 SETTINGS ------------- */
#ifdef USE_DMP_MPU
  #include "MPU6050_6Axis_MotionApps20.h"
  MPU6050 mpuObject(MPU_addr);
#else
  #include "MPU6050_raw.h"
  MPU6050_raw mpuObject(Wire);
  mpuObject.setAddress(MPU_addr);
#endif



int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
//int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main
#ifdef USE_DMP_MPU
  namespace DMP_DATA {
    uint8_t fifoBuffer[64];         // буфер 
    Quaternion quart;         // [w, x, y, z] 
    VectorFloat gravity;      // [x, y, z]            gravity vector
    float ypr[3];
    float euler[3];         // [psi, theta, phi]    Euler angle container
  }
#else
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  double accAngleX;
  double accAngleY;
  double gyroAngleX;
  double gyroAngleY;
  double Kcmpl = 0.96;
#endif



/* -------------- TIMER VARS ------------------ */
namespace TIMER_INTER {
  unsigned long last_time = 0;
  unsigned long time_step = 15;
  unsigned long time_since_start_full = 0;
  unsigned long time_since_start_experimental = 0;
  unsigned long time_last_dmp = 0;  

  // safety timers
  //unsigned long
}




/* --------------- PID AND SAT VARS ------------- */
double pid_Kp = 0.6*1.25;//0.8;
double pid_Ki = pid_Kp*2 / 1000;//0.006;    // ms
double pid_Kd = 0.125*pid_Kp*1000;//300;
const bool USE_START_PWM = false;                     /* pwm_hor + u_pid */ 
double startPwm = 1400;                               /* Set another max/min pid */
const double min_PID_control = (USE_START_PWM) ? (PWM_MIN-startPwm) : 0;                       /* !! Attention !! */
const double max_PID_control = (USE_START_PWM) ? (PWM_MAX-startPwm) : (PWM_MAX-PWM_MIN);       /* if use pwm_hor */

namespace SATURATION_NS {
  double minOut = 1200;                              
  double maxOut = 1500;       
  bool USE_START_PWM = false; 
  inline double satSignal(double signal){
    if (signal < minOut) signal = minOut;
    if (signal > maxOut) signal = maxOut;
    return signal;
  }

  inline double sigFromPid2Pwm(double sig) {
    if (USE_START_PWM) return sig + startPwm;
    else return  map(sig, min_PID_control, max_PID_control, PWM_MIN, PWM_MAX);;
  }
}


/* ------------ HELICOPTER EXPERIMENT VARS ------------- */
namespace EXPR_VARS {
  double setpoint_angle = 20;
  long total_integral = 0;
  double last_error = 0;
  double control_signal;
  void resetPid(){
    total_integral = 0; last_error = 0; control_signal = 0;
  }
}
const bool CURR_ANGLE_INCREASE = false; // Which first for error

/* SAFETY */
const double SAFE_LIMIT_ANGLE = -75;
const double saf_MaxAngleDiff = 30.0;
double saf_LastAngle = 0;
bool saf_SetLastAngle = false;
bool checkSafety(double curr_angle);   /* STOP work if angle in danger Zone */
#define YPR_COMPONENT (1)


/* --------------- WORKING FLAGS ------------------- */
namespace FLAGS_WORK {
  bool recorded_settling = false;
  bool recorded_rise = false;
  bool startWorking = false;  
}


/* ------------- SETUP PROJECT ------------------ */

void setup() {
  Serial.begin(serialFreq);
  bldcEsc.arm();  // send arm value to get ready
  bldcEsc.speed(PWM_MIN);
  delay(1000);    // wait to prepare

  #ifdef CHECK_BLDC_BEFORE_START
  bldcCheckRampUpDown();
  #endif

  #ifdef SETUP_WIRE
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
    #endif
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin(); 
      Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true); 
    #endif
  #endif

  mpu_initialization(); 
  set_calibration(offsetMPU);

  #ifdef PIN_LOG_TRUE
  pinMode(led_logger.LED_PIN, OUTPUT); // enable pin flashing for logging
  #endif


  FLAGS_WORK::startWorking = false;
  TIMER_INTER::time_since_start_full = millis();
}




/* ------------------ MAIN LOOP -------------------- */
void loop() {
  
  if (Serial.available() > 0){ switchWorking(Serial.read()); } // read command to stop/start

  // set flag do nothing
  if (!FLAGS_WORK::startWorking) return;

  // dt step working
  if (millis() - TIMER_INTER::last_time > TIMER_INTER::time_step) 
  {
    TIMER_INTER::last_time = millis();
  
    if (mpuObject.dmpGetCurrentFIFOPacket(DMP_DATA::fifoBuffer))  // about 3-5 ms for dmpGetFifo
    { 
      #ifdef DEBUG_WORK_PROCESS
      Serial.print(F("dmp ")); Serial.print(millis() - TIMER_INTER::time_last_dmp); Serial.print(F(" dt ")); Serial.println(millis()-TIMER_INTER::last_time);
      TIMER_INTER::time_last_dmp = millis();
      #endif

      // get angle and safety
      dmpGetYawPitchRoll();
      checkSafety(rad2Degree(DMP_DATA::ypr[YPR_COMPONENT])); // danger zone
      
      // Pid working
      double error = calcError(EXPR_VARS::setpoint_angle, rad2Degree(DMP_DATA::ypr[YPR_COMPONENT]));
      PID_Step(error, TIMER_INTER::time_step);


      // Get signal and saturation
      long signal = SATURATION_NS::sigFromPid2Pwm(EXPR_VARS::control_signal);
      signal = SATURATION_NS::satSignal(signal);
      if (signal > 1650) signal = 1650;
      bldcEsc.speed(signal);


      #ifdef DEBUG_PLOTTING_VIEW
      Serial.print(F("pitch:")); Serial.print(rad2Degree(DMP_DATA::ypr[YPR_COMPONENT])); Serial.print(", ");
      Serial.print(F("err:")); Serial.print(error); Serial.print(", "); 
      Serial.print(F("pwm:")); Serial.print(signal); Serial.print(", "); Serial.print("int:");
      Serial.println(EXPR_VARS::total_integral);
      #endif
      //TIMER_INTER::last_time = millis();
    }

    #ifdef DEBUG_WORK_PROCESS
    Serial.print(F("f exp "));
    Serial.print((millis()-TIMER_INTER::time_since_start_full) / 1000.0); Serial.print("\t");
    Serial.print((millis()-TIMER_INTER::time_since_start_experimental) / 1000.0); Serial.print("\t");
    #endif

  }



  #ifdef PIN_LOG_TRUE
  led_log_fun();
  #endif 

}


void controlBldc()
{
  //EXPR_VARS
  //bldcEsc.speed(EXPR_VARS::control_signal);

  // if elapsed is greater, than calc
  // TIMER_INTER::t_now = millis(); // since start
  //if (TIMER_INTER::t_now - TIMER_INTER::t_last_PID >= TIMER_INTER::T_sample){
  //  TIMER_INTER::t_last_PID = TIMER_INTER::t_now;
    //calcpid
    // to much maybe bad
  //}

}

// dt - ms or sec
void PID_Step(double error, double dt)
{
  if (dt < 0.0001) return;
  EXPR_VARS::total_integral += error * dt;             
  EXPR_VARS::control_signal = pid_Kp * error + 
        pid_Ki * EXPR_VARS::total_integral +
        pid_Kd * (error - EXPR_VARS::last_error)/dt;
  //EXPR_VARS::total_integral += error * dt;             // before calc
  EXPR_VARS::last_error = error;

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

inline bool checkSafety(double curr_angle) {
  // check Danger zone limit
  bool checkDanger = (CURR_ANGLE_INCREASE) ? (curr_angle > SAFE_LIMIT_ANGLE) : (curr_angle < SAFE_LIMIT_ANGLE);

  // check if Sudden change angle too fast
  if (!checkDanger && saf_SetLastAngle) {
    double diff = curr_angle - saf_LastAngle;
    if (diff < 0.0) diff = -diff;
    if (diff >= saf_MaxAngleDiff) checkDanger = true;
  }

  if (checkDanger) {
    stop();
    #ifdef DEBUG_SAFETY_BREACHED
      Serial.println(F("\n===DANGER===\n"));
    #endif
  }
  if (!saf_SetLastAngle) {  saf_SetLastAngle = true; }
  saf_LastAngle = curr_angle;
}

void stop() {
  bldcEsc.stop();
  reset();
  FLAGS_WORK::startWorking = false;
}

void start() {
  timersStart();
  reset();
  FLAGS_WORK::startWorking = true;
}

void timersStart() {
  TIMER_INTER::time_since_start_experimental = millis();
}

//void resetPid();
void reset() {
  EXPR_VARS::resetPid();
  TIMER_INTER::last_time = 0;
  TIMER_INTER::time_since_start_full = 0;
  TIMER_INTER::time_since_start_experimental = 0;
}

double calcError(double setAngle, double currAngle)
{
  return (CURR_ANGLE_INCREASE) ? (setAngle - currAngle) : (currAngle - setAngle);
}


/* --------- ANGLES FUNCTIONS ------------ */

#ifdef USE_DMP_MPU
void dmpGetQuart()
{
    mpuObject.dmpGetQuaternion(&DMP_DATA::quart, DMP_DATA::fifoBuffer);
    #ifdef DEBUG_ANGLE
      Serial.print(F("quat\t"));
      Serial.print(DMP_DATA::quart.w); Serial.print("\t");
      Serial.print(DMP_DATA::quart.x); Serial.print("\t");
      Serial.print(DMP_DATA::quart.y); Serial.print("\t");
      Serial.println(DMP_DATA::quart.z);
    #endif
}

void dmpGetYawPitchRoll()
{
  mpuObject.dmpGetQuaternion(&DMP_DATA::quart, DMP_DATA::fifoBuffer);
  mpuObject.dmpGetGravity(&DMP_DATA::gravity, &DMP_DATA::quart);
  mpuObject.dmpGetYawPitchRoll(DMP_DATA::ypr, &DMP_DATA::quart, &DMP_DATA::gravity);

  #ifdef DEBUG_ANGLE
    Serial.print(F("ypr\t"));
    Serial.print(DMP_DATA::ypr[0] * 180/M_PI); Serial.print("\t");
    Serial.print(DMP_DATA::ypr[1] * 180/M_PI); Serial.print("\t");
    Serial.println(DMP_DATA::ypr[2] * 180/M_PI);
  #endif
}
#endif



#ifndef USE_DMP_MPU

void getRawAccelGyro()
{
  mpuObject.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  #ifdef DEBUG_ANGLE
    Serial.print(F("a/g:\t"));
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  #endif
}

void getAnglesFromRaw(long elapsed_time, double* roll, double* pitch, double* yaw)
{
  /* view on error of imu  */
  // https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
  elapsed_time /= 1000; // get in seconds
  double AccX = ax / 16384.0;
  double AccY = ay / 16384.0;
  double AccZ = ax / 16384.0;
  
  accAngleX = atan2(AccY / (sqrt(AccX*AccX + AccZ*AccZ))) * 180/PI;
  accAngleY = atan2(-AccX/sqrt(AccY*AccY + AccZ*AccZ)) * 180/PI;
  //accAngleY = (atan2(ax, az) + PI) * RAD_TO_DEG;
  //accAngleX = (atan2(ay, az) + PI) * RAD_TO_DEG;
  double GyroX = gx / 131.0;
  double GyroY = gy / 131.0;
  double GyroZ = gz / 131.0;

  gyroAngleX += GyroX * elapsed_time;
  gyroAngleY += GyroY * elapsed_time;
  *yaw = *yaw + GyroZ * elapsed_time;

  *roll = Kcmpl * gyroAngleX + (1-Kcmpl)*accAngleX;
  *pitch = Kcmpl * gyroAngleY + (1-Kcmpl)*accAngleY;

  #ifdef DEBUG_ANGLE
  Serial.print(F("raw ypr\t"));
  Serial.print(*yaw); Serial.print("\t");
  Serial.print(*pitch); Serial.print("\t");
  Serial.println(*roll); 
  #endif

}

#endif


/* --------- SUPPS FUNCTIONS ----------- */


double degree2Rad(double deg)
{
  return deg * PI / 180.0;
}

double rad2Degree(double rad)
{
  return rad * (180.0 / PI);
}

void set_calibration(int16_t data[6])
{
  #ifdef USE_DMP_MPU
        mpuObject.setXAccelOffset(data[0]);
        mpuObject.setYAccelOffset(data[1]);
        mpuObject.setZAccelOffset(data[2]);
        mpuObject.setXGyroOffset(data[3]);
        mpuObject.setYGyroOffset(data[4]);
        mpuObject.setZGyroOffset(data[5]);
  #else
    mpuObject.setGyroOffsets(data[3], data[4], data[5]);
    mpuObject.setAccOffsets(data[0], data[1], data[2]);
  #endif

}

void bldcCheckRampUpDown() {
  for (Speed2Send = PWM_MIN; Speed2Send <= PWM_MIN+100; Speed2Send += 10) {       
    bldcEsc.speed(Speed2Send);                                    
    delay(50);                                            
  }
  delay(2000);                                            
  
  for (Speed2Send = PWM_MIN+100; Speed2Send >= PWM_MIN; Speed2Send -= 10) {        
    bldcEsc.speed(Speed2Send);                                     
    delay(50);                                             
   }
  delay(2000);                                            
}

void mpu_initialization() { 
  mpuObject.initialize();
#ifdef DEBUG_HW
  Serial.println(mpuObject.testConnection() ? F("MPU OK") : F("MPU FAIL")); // состояние соединения
  Serial.println("Init DMP");
#endif

#ifdef USE_DMP_MPU
  int8_t devStatus = mpuObject.dmpInitialize();
  if (devStatus == 0) {
    mpuObject.setDMPEnabled(true);
  } else {
    Serial.print(F("DMP err: "));
    Serial.println(devStatus);
  }
#endif
}



