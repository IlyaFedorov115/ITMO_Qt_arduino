//#include "..\..\..\..\..\..\..\ITMO_2024_WORK\ITMO_Qt_arduino\Test_Protocol\serialmanager.h"
//#include "../../../../../../../ITMO_2024_WORK/ITMO_Qt_arduino/Test_Protocol/vtolprotocol.h"
#include <Wire.h>
//#include <MPU6050.h>
#include "vtolprotocol.h"
#include "serialmanager.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "bldc_api.h"
#include "supp_functions.h"


/* ----- SUPP FLAGS ---- */
#define PIN_LOG_TRUE
#define USE_SERVO_FLAG 1
#define USE_AUTO_MPU_CALIBRATION 0

#define WIRE_HAS_TIMEOUT 1
#define LOGGING_MSG_FLAG 0 // loging for Mpu init and other hardware

/* ------------- CONSTANTS ------------ */
const int MPU_addr = 0x68; // адрес датчика
const long serialFreq = 115200;
const int wireTimeout = 3000;
bool IS_SET_START = true;
const int timeoutRead = 1;  // ms

MPU6050 mpuObject(MPU_addr);
int16_t offsetMPU[6] = {-1372,	857,	887,	3,	-40,	-52};//{-3761, 1339, -3009, 45, -54, 44}; // my 2
//int16_t offsetMPU[6] = {-600,	-4997,	1185,	127,	27,	-99}; // 3 - main

namespace TIMER_INTER {
  unsigned long last_time = 0;
  unsigned long time_step = 500;
}
bool USE_IMU_INTERRUPT = false;

namespace DMP_DATA {
  uint8_t fifoBuffer[64];         // буфер 
  Quaternion quart;         // [w, x, y, z] 
  VectorFloat gravity;      // [x, y, z]            gravity vector
  float ypr[3];
  float euler[3];         // [psi, theta, phi]    Euler angle container
  int PACKET_SIZE = 0;
  bool DMP_READY = false;
  int16_t rawDataBuffer[7];
}
volatile bool mpuFlag = false;  // флаг прерывания готовности
void dmpReady() {
  mpuFlag = true;
}


void log_2_serial(char* str){
  if (LOGGING_MSG_FLAG) Serial.println(str);
}

enum FormatAngle {
  QUARTERNION_ANGLE,
  RAW_ACCEL_GYRO_ANGLE,
  YAW_PITCH_ROLL_ANGLE,
};

namespace SERVO_DATA {
  const int ESC_PIN = 9;  //D9
  const int ESC_MIL_MIN = 1200;
  const int ESC_MIL_MAX = 2000;
  BLDC_API* servoAPI;
}

FormatAngle CURR_FORMAT_ANGLE = FormatAngle::QUARTERNION_ANGLE;

/* ====== Packets to get ====== */

vtol_protocol::ProtocolMsg* msgReceive = new vtol_protocol::ProtocolMsg();
vtol_protocol::ProtocolMsg* msgSend = new vtol_protocol::ProtocolMsg();
SerialPacket* packetReceive = new SerialPacket();
SerialPacket* packetSend = new SerialPacket();

SerialPacketManager* serialManager = new SerialPacketManager();

uint8_t* packetBuffer = new uint8_t[SerialPacket::MAX_PACKET_SIZE];





void setup() {
  /* ========= Init variables ======== */

  Serial.begin(serialFreq); //while (!Serial);
  Serial.setTimeout(timeoutRead);
  /* =======     Init BLDC     ======= */
  #if(USE_SERVO_FLAG == 1)
  SERVO_DATA::servoAPI = new BLDC_API(SERVO_DATA::ESC_PIN, 
                                      SERVO_DATA::ESC_MIL_MIN, 
                                      SERVO_DATA::ESC_MIL_MAX);
  #endif
  /* SERVO_DATA::servoAPI->autoCalibration(); */
  /* SERVO_DATA::servoAPI->writeMicroseconds(SERVO_DATA::ESC_MIL_MIN); */



  /* =======     Init WIRE     ======= */
  #if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(wireTimeout /* us */, true /* reset_on_timeout */);
  #endif
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true); //Serial.println("Use FastWire::setup\n");
  #endif



  /* =======     Init MPU     ======= */
  mpu_initialization1();

  DMP_DATA::PACKET_SIZE = mpuObject.dmpGetFIFOPacketSize();

  #if(USE_AUTO_MPU_CALIBRATION == 1)
    mpuObject.CalibrateAccel(6);
    mpuObject.CalibrateGyro(6);
  #else
    SUPP_FUNCS::set_calibration(&mpuObject, offsetMPU);
  #endif

  /* =======     Init LED Logger     ======= */
  #ifdef PIN_LOG_TRUE
  pinMode(SUPP_FUNCS::led_logger.LED_PIN, OUTPUT); // enable pin flashing for logging
  #endif


  /* ======= Init packets and messages =======*/
  msgSend->type = vtol_protocol::MsgProps::MSG_TYPE::QUART_ANGLE;
  msgSend->lenData = vtol_protocol::MsgProps::getDataLen(msgSend->type);
  //serialManager->setCheckSumMethod(SerialPacketManager::crc8);
}

void loop() {
  /* Failed connect to IMU */
  if (!DMP_DATA::DMP_READY) return;



  /* Not set to start running */
  if (!IS_SET_START) return;

  if(!USE_IMU_INTERRUPT)
  {
    if (millis() - TIMER_INTER::last_time > TIMER_INTER::time_step)
    {
        /* Get and parsing */
  if (Serial.available() > 0)
  {
    Serial.print("Get ");
    Serial.println(Serial.available());
    //_parsePacket(packetReceive, msgReceive, packetBuffer);
    Serial.readBytes(packetBuffer, Serial.available());
      // soon - read and change current state. Asyns wait for all packet
      // now - read all in one time, else - discard packet
  }




      if (mpuObject.dmpGetCurrentFIFOPacket(DMP_DATA::fifoBuffer)) {
        prepareAngle();
        vtol_protocol::Parser::parse2Serial(packetSend, msgSend);
        packetSend->_checkSum = SerialPacketManager::crc8((uint8_t*)packetSend+1, packetSend->getFullPacketSize());//serialManager->calcCheckSum(packetSend);
        //serialManager->calcCheckSum(packetSend);
        memcpy(packetBuffer, (void*)packetSend, packetSend->getFullPacketSize());
 //       Serial.write(packetBuffer, packetSend->getFullPacketSize());
        //SerialPacket packet =  vtol_protocol::Parser::parse2Serial(msgSend);
        //packet._checkSum = serialManager.calcCheckSum(packet);
        //memcpy(&packetSendBuffer[0], (void*)&packet, packet.getFullPacketSize());
        //memcpy(&packetSendBuffer[0], (void*)&packet, 6*4+3);
        //memcpy(packetSendBuffer_, packetSendBuffer__, 50);
        //Mymemcpy(packetSendBuffer, (void*)packetSend, 10);
        
      }

      TIMER_INTER::last_time = millis();

    }

  }
}

// Make read by state: start and end symbol
// use struct without buffer
void _parsePacket(SerialPacket* packetReceive, vtol_protocol::ProtocolMsg* msgReceive, uint8_t* packetBuffer)
{
  // too small packet, count that bad
  Serial.println(Serial.available());
  if (Serial.available() < 3) {
    Serial.readBytes(packetBuffer, Serial.available());
    return;
    // ++
  }
  packetReceive->_checkSum = Serial.read();
  packetReceive->_msgType = Serial.read();
  packetReceive->_dataLength = Serial.read();

  // !! Maybe situation, when Qt send packet with pwm and 
  // user click "stop" button -> maybe 2 packets in one time
  if (packetReceive->_dataLength != Serial.available()) {
    // bad packet
    Serial.readBytes(packetBuffer, Serial.available());
    return;
  }

  if (packetReceive->_dataLength != 0) {
      int read = Serial.readBytes(packetBuffer, packetReceive->_dataLength);
      if (read < packetReceive->_dataLength) {
        // timeout full packet
        return;
      }
      memcpy(packetReceive->buffer, packetBuffer, packetReceive->_dataLength);
  }

  if (!serialManager->isValidPacket(*packetReceive)) {
    // bad packet
    return;
  }
  /*
  if (packetReceive->_dataLength < Serial.available()) {
    Serial.readBytes(packetBuffer, Serial.available());
    return;
  }
  Serial.readBytes(packetReceive->buffer, Serial.available());
  */
  _parseMsg(packetReceive, msgReceive);  
}

void _parseMsg(SerialPacket* packetReceive, vtol_protocol::ProtocolMsg* msgReceive)
{
  vtol_protocol::Parser::PARSE_CODE code = vtol_protocol::Parser::parse(packetReceive, msgReceive);

  if (code == vtol_protocol::Parser::PARSE_CODE::WRONG_TYPE){
    // bad packet
    return;
  }
  if (code == vtol_protocol::Parser::PARSE_CODE::WRONG_DATA_LEN) {
    // bad packet
    return;
  }

  if (msgReceive->type == vtol_protocol::MsgProps::MSG_TYPE::PWM_SIGNAL) {
    // Shield maybe
    if (msgReceive->data[0].number > 1470) msgReceive->data[0].number = 1470;
    SERVO_DATA::servoAPI->writeMicroseconds((int)msgReceive->data[0].number);
  } else if (msgReceive->type == vtol_protocol::MsgProps::MSG_TYPE::SET_BY_TIMER) {
    TIMER_INTER::time_step = (long)msgReceive->data[0].number;
  } else if (msgReceive->type == vtol_protocol::MsgProps::MSG_TYPE::START_SIM) {
    IS_SET_START = true;
  } else if (msgReceive->type == vtol_protocol::MsgProps::MSG_TYPE::STOP_SIM) {
    IS_SET_START = false;
  } else {
    return;
  }

}




void prepareAngle() {
  switch (CURR_FORMAT_ANGLE) {
    case FormatAngle::QUARTERNION_ANGLE:
      _prepareQuart();
      break;
    case FormatAngle::RAW_ACCEL_GYRO_ANGLE:
      _prepareRawAngle();
      break;
    case FormatAngle::YAW_PITCH_ROLL_ANGLE:
      _prepareYawPitchRoll();
      break;
    default:
      return;
  }
}

void _prepareQuart() 
{
  mpuObject.dmpGetQuaternion(&DMP_DATA::quart, DMP_DATA::fifoBuffer);
  msgSend->type = vtol_protocol::MsgProps::MSG_TYPE::QUART_ANGLE;
  msgSend->lenData = 4;
  msgSend->data[0].number = DMP_DATA::quart.w;
  msgSend->data[1].number = DMP_DATA::quart.x;
  msgSend->data[2].number = DMP_DATA::quart.y;
  msgSend->data[3].number = DMP_DATA::quart.z;
}

void _prepareYawPitchRoll()
{
  mpuObject.dmpGetQuaternion(&DMP_DATA::quart, DMP_DATA::fifoBuffer);
  mpuObject.dmpGetGravity(&DMP_DATA::gravity, &DMP_DATA::quart);
  mpuObject.dmpGetYawPitchRoll(DMP_DATA::ypr, &DMP_DATA::quart, &DMP_DATA::gravity);
  msgSend->type = vtol_protocol::MsgProps::MSG_TYPE::YAW_PITCH_ROLL;
  msgSend->lenData = 3;
  msgSend->data[0].number = DMP_DATA::ypr[0];
  msgSend->data[1].number = DMP_DATA::ypr[1];
  msgSend->data[2].number = DMP_DATA::ypr[2];
}

void _prepareRawAngle()
{
  mpuObject.dmpGetQuaternion(&DMP_DATA::quart, DMP_DATA::fifoBuffer);
  mpuObject.dmpGetGyro(&DMP_DATA::rawDataBuffer[3], DMP_DATA::fifoBuffer);
  mpuObject.dmpGetAccel(&DMP_DATA::rawDataBuffer[0], DMP_DATA::fifoBuffer);
  msgSend->type = vtol_protocol::MsgProps::MSG_TYPE::RAW_GYRO_ACCEL;
  msgSend->lenData = 6;
  for (int i = 0; i < 6; i++) {
    msgSend->data[i].number = (float) DMP_DATA::rawDataBuffer[i];
  }

}



void mpu_initialization1() { 
  
  mpuObject.initialize();
  bool res = mpuObject.testConnection();

  DMP_DATA::DMP_READY = (res) ? true : false;
  #if(LOGGING_MSG_FLAG == 1)
  (res) ? log_2_serial("MPU6050 connect OK") : log_2_serial("MPU6050 connect FAIL");
  #endif

  int8_t devStatus = mpuObject.dmpInitialize();

  if (devStatus != 0) {
    #if(LOGGING_MSG_FLAG == 1)
    log_2_serial("DMP failed code: ");
    log_2_serial(devStatus);
    #endif
    return;
  }
  
  mpuObject.setDMPEnabled(true);

}

