#include <Wire.h>


const int wireTimeout = 3000;
#define SERIAL_SPEED (250000)//(115500)

unsigned long last_time_write_millis = 0;
unsigned long time_millis_diff = 0;

unsigned long last_time_write_micros = 0;
unsigned long time_micros_diff = 0;

float a1 = 0;
float a2;
float a3;
float a4;
float a5;

union FLOAT_TYPE {
  byte buffer[sizeof(float)];
  float num;
};

FLOAT_TYPE a_1;
FLOAT_TYPE a_2;
FLOAT_TYPE a_3;
FLOAT_TYPE a_4;
FLOAT_TYPE a_5;


void setup() {
  Wire.begin(); //begin the wire comunication
      Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
      Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(SERIAL_SPEED);

  last_time_write_millis = millis();
  last_time_write_micros = micros();

}

void loop() {
  // put your main code here, to run repeatedly:
  a1 = random(50) * PI / (float)(random(1,2));
  a2 = random(60) * PI / (float)(random(1,2));
  a3 = random(55) * PI / (float)(random(1,2));
  a4 = random(40) * PI / (float)(random(1,2));
  a5 = random(20) * PI / (float)(random(1,2));

  a_1.num = a1;
  a_2.num = a1;
  a_3.num = a1;
  a_4.num = a1;
  a_5.num = a1;

  last_time_write_millis = millis();
  last_time_write_micros = micros();

  /*
  // ----------------- SERIAL PRINT ---------------------- //
  Serial.print(a1); Serial.print(','); Serial.print(a2);
  Serial.print(a3); Serial.print(','); Serial.print(a4);
  Serial.println(a5);
  //*/

  /*
  Serial.write((const char*)&a1, sizeof(a1));
  Serial.write(',');
  Serial.write((const char*)&a1, sizeof(a2));
  Serial.write(',');
  Serial.write((const char*)&a1, sizeof(a3));
  Serial.write(',');
  Serial.write((const char*)&a1, sizeof(a4));
  Serial.write(',');
  Serial.write((const char*)&a1, sizeof(a5));
  Serial.write('\n');
  */

///*
  Serial.write(a_1.buffer, 4);
  Serial.write(',');
  Serial.write(a_2.buffer, 4);
  Serial.write(',');
  Serial.write(a_3.buffer, 4);
  Serial.write(',');
  Serial.write(a_4.buffer, 4);
  Serial.write(',');
  Serial.write(a_5.buffer, 4);
  Serial.write('\n');
//*/

  time_micros_diff = micros() - last_time_write_micros;
  time_millis_diff = millis() - last_time_write_millis;
  Serial.println();
  Serial.print(time_micros_diff); Serial.print(' '); Serial.println(time_millis_diff);

}
