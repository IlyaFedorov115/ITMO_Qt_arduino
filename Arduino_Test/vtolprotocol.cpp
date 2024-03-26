
#include "vtolprotocol.h"
#include "serialmanager.h"
//#include <stdexcept>

using namespace vtol_protocol;


uint8_t MsgProps::getPwmDataLen() {
    return 1;
}

uint8_t MsgProps::getQuartDataLen() {
    return 4;
}

uint8_t MsgProps::getRawGyroAccelDataLen() {
    return 6;
}

uint8_t MsgProps::getYawPitchRollDataLen() {
    return 3;
}

uint8_t MsgProps::getSetInterDataLen() {
    return 0;
}

uint8_t MsgProps::getSetTimerDataLen() {
    return 1;
}

uint8_t MsgProps::getStartSimDataLen() {
    return 0;
}

uint8_t MsgProps::getStopSimDataLen() {
    return 0;
}

uint8_t MsgProps::getAnswDataLen() {
    return 0;
}

uint8_t MsgProps::getSetAngleTypeDataLen() {
    return 1;
}

uint8_t MsgProps::getSendWrongDataLen() {
    return 1;
}

uint8_t MsgProps::getGetWrongDataLen()
{
    return 0;
}


int8_t MsgProps::getDataLen(MSG_TYPE type) {
    switch (type) {
        case MSG_TYPE::PWM_SIGNAL:
            return getPwmDataLen();
            break;
        case MSG_TYPE::QUART_ANGLE:
            return getQuartDataLen();
            break;
        case MSG_TYPE::RAW_GYRO_ACCEL:
            return getRawGyroAccelDataLen();
            break;
        case MSG_TYPE::YAW_PITCH_ROLL:
            return getYawPitchRollDataLen();
            break;
        case MSG_TYPE::SET_BY_INTERRUPT:
            return getSetInterDataLen();
            break;
        case MSG_TYPE::SET_BY_TIMER:
            return getSetTimerDataLen();
            break;
        case MSG_TYPE::START_SIM:
            return getStartSimDataLen();
            break;
        case MSG_TYPE::STOP_SIM:
            return getStopSimDataLen();
            break;
        case MSG_TYPE::ANSW_HW_OK:
        case MSG_TYPE::ANSW_HW_WRONG:
            return getAnswDataLen();
            break;
        case MSG_TYPE::SET_ANGLE_TYPE:
            return getSetAngleTypeDataLen();
            break;
        case MSG_TYPE::GET_WRONG_TIMEOUT:
        case MSG_TYPE::GET_WRONG_CH_SUMM:
            return getGetWrongDataLen();
            break;
        case MSG_TYPE::SEND_WRONG_TIMEOUT:
        case MSG_TYPE::SEND_WRONG_CH_SUMM:
            return getSendWrongDataLen();
            break;
        default:
            // Handle invalid message type
            //throw std::invalid_argument("Invalid MSG_Type!");
            return -1;
            break;
    }
}

Parser::PARSE_CODE Parser::parse(const SerialPacket& packet, ProtocolMsg& msg)
{
    MsgProps::MSG_TYPE type_ = (MsgProps::MSG_TYPE)packet._msgType;
    int8_t len_ = MsgProps::getDataLen(type_);

    if (len_ < 0) {
        return PARSE_CODE::WRONG_TYPE;
    }
    if (len_*sizeof(ProtocolMsg::FloatType) != packet._dataLength) {
        return PARSE_CODE::WRONG_DATA_LEN;
    }

    msg.lenData = len_;
    msg.type = type_;
    memcpy(msg.data, packet.buffer, packet._dataLength);

    return PARSE_CODE::SUCCESS;
}

Parser::PARSE_CODE Parser::parse(const SerialPacket *packet, ProtocolMsg *msg)
{
    //return parse(*packet, *msg);
    MsgProps::MSG_TYPE type_ = (MsgProps::MSG_TYPE)packet->_msgType;
    int8_t len_ = MsgProps::getDataLen(type_);

    if (len_ < 0) {
        return PARSE_CODE::WRONG_TYPE;
    }
    if (len_*sizeof(ProtocolMsg::FloatType) != packet->_dataLength) {
        return PARSE_CODE::WRONG_DATA_LEN;
    }

    msg->lenData = len_;
    msg->type = type_;
    memcpy(msg->data, packet->buffer, packet->_dataLength);

    return PARSE_CODE::SUCCESS;
}

SerialPacket Parser::parse2Serial(ProtocolMsg &msg)
{
    SerialPacket res;
    res._msgType = (uint8_t)msg.type;
    res._dataLength = msg.lenData * sizeof(ProtocolMsg::FloatType);
    memcpy(res.buffer, msg.data, res._dataLength);
    return res;
}

void Parser::parse2Serial(SerialPacket *packet, ProtocolMsg *msg)
{
    packet->_msgType = (uint8_t)msg->type;
    packet->_dataLength = msg->lenData * sizeof(ProtocolMsg::FloatType);
    memcpy(packet->buffer, msg->data, packet->_dataLength);
}
