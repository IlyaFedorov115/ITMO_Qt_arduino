#include <Servo.h>

class BLDC_API {
  public:
    Servo escObj;
    const int ESC_PIN;
    const int ESC_MIL_MIN;
    const int ESC_MIL_MAX;
    int TEST_MAX = 1450;
    BLDC_API(int escPin = 9, int milMin = 1000, int milMax = 2000, int test_max = 1450) 
    : ESC_PIN(escPin), ESC_MIL_MAX(milMax), ESC_MIL_MIN(milMin), TEST_MAX(test_max)
    {
      escObj.attach(ESC_PIN, ESC_MIL_MIN, ESC_MIL_MAX);
      //this->_display_instruction_manual();
    }

    void writeMicroseconds(int val)
    {
      escObj.writeMicroseconds(val);
    }

    /* 
    Загрузка эскиза в Arduino. Затем и БЕЗ питания ESC:
    1) Подключите Arduino к компьютеру, терминал - введите 1.
    ESC перейдут в режим программирования.
    2) Включить ESC. Вы должны услышать 3 звуковых сигнала «бип1 бип2 бип3», - с источником питания все в порядке.
    Через 2 секунды вы должны услышать «звуковой сигнал», полный газ зафиксирован ESC.
    3) Теперь введите 0 - «остановить двигатель»  на ESC.
    Раздастся несколько звуковых сигналов, соответствующих количеству ячеек в батарее 3S
    Раздается длинный звуковой сигнал, означающий, что ESC зафиксировали положение «остановка двигателя».
    4) Теперь ESC откалиброваны. Введите 2 - тестирование.
    */

    void autoCalibration()
    {
      escObj.writeMicroseconds(ESC_MIL_MAX);
      delay(10000);
      escObj.writeMicroseconds(ESC_MIL_MIN);
      delay(5000);
      /*
      for (int i = ESC_MIL_MIN*1.5; i <= ESC_MIL_MAX; i += 10){
        escObj.writeMicroseconds(i);
        delay(200);
      }
      escObj.writeMicroseconds(ESC_MIL_MIN);
      */
    }

    void manualCalibration()
    {
      bool exit_flag = false;
      while (!exit_flag)
      {
        if (Serial.available()){
          char data = Serial.read();

          switch (data) {
            // 0
            case 48:
              Serial.println("Sending minimum throttle");
              escObj.writeMicroseconds(ESC_MIL_MIN);
              break;

            // 1
            case 49: 
              Serial.println("Sending maximum throttle");
              escObj.writeMicroseconds(ESC_MIL_MAX);
              break;

            // 2
            case 50: 
              this->_working_test();
              break;

            // 3
            case 51:
              this->_display_instruction_manual();
              break;
            // 4
            case 52:
              Serial.println("Stop manual calibration");
              exit_flag = true;
              break;  
          }
        }
      }
    }

    void _working_test()
    {
      Serial.print("Running test in 3 ");
      delay(1000);
      Serial.print("2 ");
      delay(1000);
      Serial.print("1...");
      delay(1000);
      if (TEST_MAX > 1700) TEST_MAX = 1700;
      for (int i = ESC_MIL_MIN; i <= TEST_MAX; i += 10){
        Serial.print("Pulse: "); Serial.println(i);
        escObj.writeMicroseconds(i);
        delay(200);
      }

      Serial.println("Stop testing...");
      escObj.writeMicroseconds(ESC_MIL_MIN);

    }

    void _display_instruction_manual()
    {
      Serial.println("Instruction for manual calibration:");
      Serial.println("\t0 : Send min throttle");
      Serial.println("\t1 : Send max throttle");
      Serial.println("\t2 : Run test function");
      Serial.println("\t3 : Display instruction");
      Serial.println("\t4 : Stop calibration");
    }


};

