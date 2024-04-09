
/*
  ========= ПОРЯДОК РАБОТЫ ===========
  1) проверить угол датчика вначале (увеличивается или уменьшается) + флаг на это
  2) проверить двигатель
  3) проверить настройки безопасности и ПИД
  4) запуск идет с задержкой
*/



// ======================== Флаги вывода ======================== //

//#define DEBUG_WORK        // лог углов, пид через Serial.print (для отладки, 2.5 мс) 
//#define DEBUG_TIMERS      // лог шага времени затрат на вычисление угла
//#define DEBUG_WRITE_BYTE  // вывод байтами (докинуть нужное)
//#define DEBUG_PID         // вывод ПИД инфы (перенесется в предыдущий)


// ====================== КОНСТАНТЫ И СМЕЩЕНИЯ ДЛЯ MPU6050 ====================== //
//int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
const int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main

const float COEF_ACCEL_COMP = 0.98;   // часть акселерометра в комплементарном фильтре



// ================== КОНСТАНТЫ ДЛЯ РАБОТЫ ПИД ================== //
const double throttle= 1470;// 1550;     //initial value of throttle
const float desired_angle = 0; // target angle

const double pid_Kp = 1.8;//3.55
const double pid_Ki = 0.035;//0.003
const double pid_Kd = 0.55;//2.05

namespace EXPR_VARS {
  double total_integral = 0;
  double last_error = 0;
  double control_signal = 1200;
  void resetPid(){
    total_integral = 0; last_error = 0; control_signal = 1200;
  }
}

const double min_PID_control = -800;      // min and max PID result
const double max_PID_control = 800;       // 1200 + 800 and 2000 - 800. 



// !!!!!!! ====================================== Безопасность и работа ====================================== !!!!!!! //

const bool CURR_ANGLE_INCREASE = false; // Угол mpu увеличивается или уменьшается (+40 до 0 -> false)
const bool USE_FILT_ANGLE = true; 


const float PWM_SEND_MIN = 1200.0;   // лимиты конечной отправки
const float PWM_SEND_MAX = 1650.0;   // подаваемые на двигатель

const float ERROR_ANGLE_LIMIT = 50.0; // лимит ошибки угла. Если превышен, то стоп
                                      // может быть либо из-за некорректных показаний датчика, либо из-за 
                                      // отключения датчика и выдачи мусора
                                      // !! Пока действует и как защита при превышении угла и отключении


const char START_BUTTON = '2';  //  кнопки для  старта
const char STOP_BUTTON = '1';   //  плавная остановка
const char PANIC_BUTTON1 = '9';  // аварийная кнопка, просто отключает все в ноль
const char PANIC_BUTTON2 = '0';  // аварийная кнопка, просто отключает все в ноль
const char RESET_PID_BUTTON = '3'; // обнулить пид (интегратор в основном)


const long SMOOTH_STEP_TIME_DECREASE = 200;    // ms для плавной остановки
const int SMOOTH_STOP_PWM = 1350;             // pwm до которого уменьшение, потом стоп
const int SMOOTH_PWM_DECREASE = 5;            // насколько уменьшается каждый цикл
const int SMOOTH_MAX_LIMIT = 1600;            // если pwm текущий больше этого, то вернуться к этому значению
bool SMOOTH_SET = false;                      // smooth on

