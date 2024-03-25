#include <Wire.h>
#include <MPU6050.h>


namespace SUPP_FUNCS{
/* мигание светодиода от зависания */
  struct {
    byte LED_PIN = 13;
    bool blinkState = false;
    unsigned long last_time = 0; //milliseconds
    unsigned long time_step = 1000;
  } led_logger;

  void led_log_fun()
  {
    if (millis() - led_logger.last_time > led_logger.time_step){
      led_logger.blinkState = !led_logger.blinkState;
      digitalWrite(led_logger.LED_PIN, led_logger.blinkState);
      led_logger.last_time = millis();
    }
  }

  void waitBeforeStartProgramm()
  {
    Serial.println(F("\nSend any character to begin programm: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer 
  }

  template <class X, class M, class N, class O, class Q>
  int map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
    if (x > in_max) x = in_max;
    if (x < in_min) x = in_min;
    int res =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return res;
 }

}

namespace SUPP_FUNCS {

    void calibrate(MPU6050* mpu, uint8_t loops_gyro = 8, uint8_t loops_acc = 8)
    {
        mpu->CalibrateGyro(loops_gyro);
        mpu->CalibrateAccel(loops_acc);
    }


void set_calibration(MPU6050* mpu, int16_t data[6])
    {
        mpu->setXAccelOffset(data[0]);
        mpu->setYAccelOffset(data[1]);
        mpu->setZAccelOffset(data[2]);
        mpu->setXGyroOffset(data[3]);
        mpu->setYGyroOffset(data[4]);
        mpu->setZGyroOffset(data[5]);
    }

    void set_calibration(MPU6050* mpu,int16_t offset_XAcc, int16_t offset_YAcc, int16_t offset_ZAcc,
                         int16_t offset_Xgyr, int16_t offset_Ygyr, int16_t offset_Zgyr){
        int16_t data[6] = {offset_XAcc,offset_YAcc,offset_ZAcc,
            offset_Xgyr, offset_Ygyr, offset_Zgyr
        };
        set_calibration(mpu, data);
    }

    void data_2_SI(int16_t data[7], float data_si[7], 
                    int boost = 2, int speed = 250)
    {
        for (int i = 0; i < 3; i++)
            data_si[i] = (float)data[i] / 32768 * boost;
        for (int i = 4; i < 7; i++)
            data_si[i] = (float)data[i] / 32768 * speed;
    }

}
