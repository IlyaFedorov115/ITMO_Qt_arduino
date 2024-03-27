#ifndef VTOLPROTOCOL_H
#define VTOLPROTOCOL_H
//#include <cstdint>
#include <string.h>
#include <stdint.h>
//#include <exception>

/*
try {
    compare( -1, 3 );
}
catch( const std::invalid_argument& e ) {
    // do stuff with exception...
}
*/

class SerialPacket;

namespace vtol_protocol {
static const uint8_t MSG_MAX_LEN = 10;

/* Убрать повтор кода со switch case. */

class MsgProps {
public:
    enum class ANGLE_TYPE {
      QUART = 100,
      RAW = 101,
      YPR = 102,
      EULER = 103
    };

    enum class MSG_TYPE : uint8_t {
        PWM_SIGNAL,
        QUART_ANGLE,
        RAW_GYRO_ACCEL,
        YAW_PITCH_ROLL,
        EULER_ANGLE,
        SET_BY_INTERRUPT,
        SET_BY_TIMER,
        START_SIM,
        STOP_SIM,
        ANSW_HW_OK,
        ANSW_HW_WRONG,  // in float bad value, and pwm to match or flag dont get
        SET_ANGLE_TYPE, // in float what type
        GET_WRONG_CH_SUMM, // From qt to get how match wrong
        SEND_WRONG_CH_SUMM, // in float value
        GET_WRONG_TIMEOUT,
        SEND_WRONG_TIMEOUT,  // if send get and by timeout don`t get answer -> error on GUI. Asyns + error
    };

    static uint8_t getPwmDataLen();
    static uint8_t getQuartDataLen();
    static uint8_t getRawGyroAccelDataLen();
    static uint8_t getYawPitchRollDataLen();
    static uint8_t getSetInterDataLen();
    static uint8_t getSetTimerDataLen();
    static uint8_t getStartSimDataLen();
    static uint8_t getStopSimDataLen();
    static uint8_t getAnswDataLen();
    static uint8_t getSetAngleTypeDataLen();
    static uint8_t getSendWrongDataLen();
    static uint8_t getGetWrongDataLen();
    static int8_t getDataLen(MSG_TYPE type); // switch case return
};


struct ProtocolMsg {

    typedef union{
      float number;
      uint8_t bytes[sizeof(float)];
    } FloatType;

    MsgProps::MSG_TYPE type;
    uint8_t lenData;            // elements of data
    FloatType data[MSG_MAX_LEN];

    ProtocolMsg(MsgProps::MSG_TYPE type);
    ProtocolMsg();
};

// classError parse
/*
class ParserError: public std::exception {

};

class WrongTypeError : public ParserError {
    // field: Type, get in constructor and return char* with this
public:
    virtual const char* what() const noexcept {
       return "Unknown type packet!";
    }
};

class WrongDataLenError : public ParserError {
    // field: Len, and must be
public:
    virtual const char* what() const noexcept {
       return "Wrong Data len!";
    }
};
*/

class Parser {
public:

    enum class PARSE_CODE : uint8_t {
        WRONG_TYPE = 100,
        WRONG_DATA_LEN = 101,
        SUCCESS = 0
    };

    /*
    static ProtocolMsg parseRawMpu(const SerialPacket& packet);
    static ProtocolMsg parseYawPitchRoll(const SerialPacket& packet);
    static ProtocolMsg parseQuart(const SerialPacket& packet);
    static ProtocolMsg parsePwm(const SerialPacket& packet);
    static ProtocolMsg parseStartSim(const SerialPacket& packet);
    static ProtocolMsg parseStopSim(const SerialPacket& packet);
    static ProtocolMsg parseAnsw(const SerialPacket& packet);
    static ProtocolMsg parseSetAngleType(const SerialPacket& packet);
    static ProtocolMsg parseGet(const SerialPacket& packet);
    static ProtocolMsg parseSend(const SerialPacket& packet);
    */
    static PARSE_CODE parse(const SerialPacket& packet, ProtocolMsg& msg);
    static PARSE_CODE parse(const SerialPacket* packet, ProtocolMsg* msg);
    static SerialPacket parse2Serial(ProtocolMsg& msg);
    static void parse2Serial(SerialPacket* packet, ProtocolMsg* msg);
};

}


#endif // VTOLPROTOCOL_H
