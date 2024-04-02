

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