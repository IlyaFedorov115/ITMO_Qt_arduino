


// ======================== Флаги вывода ======================== //

//#define DEBUG_WORK_PRINT            // лог углов, пид через Serial.print (для отладки, 2.5 мс) 
//#define DEBUG_TIMERS                // лог шага времени затрат на вычисление угла
#define DEBUG_WRITE_BYTE              // вывод байтами 
//#define DEBUG_PID                   // вывод ПИД инфы 

const bool IS_DEBUG_PID = 1;                        // debug pid
const bool IS_DEBUG_RAW = 0;                        // debug raw accel gyro
//const bool IS_DEBUG_MAIN = 1;                     // angle, filt_angle, pwm, dt, integral_sum


const unsigned int BYTE_SEND_BUFFER_SIZE = 5 + IS_DEBUG_PID * 3 + IS_DEBUG_RAW * 6;

const unsigned int PID_INDEX_START = 5;                                 // if use pid debug, write main [0-4], then [5-7] for pid
const unsigned int RAW_INDEX_START = (IS_DEBUG_PID) ? (5+3) : 5;

/* [0 1 2 3 4] [5 6 7] [8 9 10| 11 12 13] */



// ======================================================= КОНСТАНТЫ И СМЕЩЕНИЯ ДЛЯ MPU6050 ======================================================== //
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

const float COEF_GYRO_COMP = 0.995;   // часть гиро в комплементарном фильтре





// ================================================== КОНСТАНТЫ ДЛЯ РАБОТЫ ПИД ========================================= //
const double throttle = 1505;// 1595 1550; -long     //initial value of throttle
const float desired_angle = 10; // target angle 

const double pid_Kp = 0.32;//0.72;//3.55
const double pid_Ki = 0.077;//0.04;//0.003
const double pid_Kd = 0.025;//0.02;//2.05


namespace EXPR_VARS {
  const double RESET_CONTROL_VAR = 1200-throttle;
  double total_integral = 0;
  double last_error = 0;
  double control_signal = RESET_CONTROL_VAR;

  const int LOG_EVERY_TIMES = 0;
  unsigned count_2_log = LOG_EVERY_TIMES;
  const float stepDtMilSec = 5.0;   //ms
  const float stepDtMicroSec = stepDtMilSec * 1000.0;

  float P_last_OUT = 0.0;
  float I_last_OUT = 0.0;
  float D_last_OUT = 0.0;

  bool setLastError = false;        // prevent D component start

  void resetPid(){
    total_integral = 0; last_error = 0.0; control_signal = RESET_CONTROL_VAR;
    P_last_OUT = I_last_OUT = D_last_OUT = 0.0;
    setLastError = false;
  }
}

const double min_PID_control = -800;      // min and max PID result
const double max_PID_control = 800;       // 1200 + 800 and 2000 - 800. 


const float FILTER_COEF_ACCEL = 0.1;      // фильтрация сырых акселерометра
const float FILTER_ANGLE = 0.18;           // фильтрация угла 





// !!!!!!! ====================================== Безопасность и работа ====================================== !!!!!!! //

const bool CURR_ANGLE_INCREASE = false;   // Угол mpu увеличивается или уменьшается (+40 до 0 -> false)
const bool USE_FILT_ANGLE = true; 


const float PWM_SEND_MIN = 1230.0;        // лимиты конечной отправки
const float PWM_SEND_MAX = 1580;          // подаваемые на двигатель

const float ERROR_ANGLE_LIMIT = 64.0; // лимит ошибки угла. Если превышен, то стоп
                                      // из-за некорректных показаний датчика, либо из-за 
                                      // отключения датчика и выдачи мусора
                                      // либо переваливание


const char START_BUTTON = '2';      //  кнопки для  старта
const char STOP_BUTTON = '1';       //  плавная остановка
const char PANIC_BUTTON1 = '9';     // аварийная кнопка, просто отключает все в ноль
const char PANIC_BUTTON2 = '0';     // аварийная кнопка, просто отключает все в ноль
const char RESET_PID_BUTTON = '3';  // обнулить пид 



/* =========== smooth stop constants =============== */
const long SMOOTH_STEP_TIME_DECREASE = 200;    // ms для плавной остановки
const int SMOOTH_STOP_PWM = 1300;             // pwm до которого уменьшение, потом стоп     (was 1350)
const int SMOOTH_PWM_DECREASE = 5;            // насколько уменьшается каждый цикл
const int SMOOTH_MAX_LIMIT = 1500;            // если pwm текущий больше этого, то вернуться к этому значению (was 1600)
bool SMOOTH_SET = false;                      // smooth on

