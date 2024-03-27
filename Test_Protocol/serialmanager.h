#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H


//#include <cstdint>
#include <string.h>
#include <stdint.h>

class SerialPacketManager;


class SerialPacketDelegate {
public:
    virtual void receiveCorrectPacket(SerialPacketManager* packet) = 0;
    virtual void receiveBadPacket(SerialPacketManager* packet) = 0;

    virtual void receiveCorrectPacket(uint8_t err) = 0; // or enum maybe
    virtual void receiveBadPacket(uint8_t err) = 0;
    // error Get
};

#define MAX_DATA_SIZE (50)


struct SerialPacket {
    uint8_t _checkSum;
    uint8_t _msgType;
    uint8_t _dataLength = 0;
    uint8_t buffer[MAX_DATA_SIZE];

    SerialPacket() = default;

    size_t getFullPacketSize() {
        return sizeof(_msgType) + sizeof(_checkSum) + sizeof(_dataLength) + _dataLength;
    }

    static const uint8_t MAX_PACKET_SIZE = MAX_DATA_SIZE + 3;

};


class SerialPacketManager {
private:
    // and method to send copy of struct


    uint8_t (*_ckeckSumMethod)(uint8_t*, size_t);
    SerialPacketDelegate** _delegates;
    size_t _countDelegates;

    uint8_t* _packetByteView(SerialPacket& packet);

    // maybe error Send and Error receive. And two Interfaces for this and errors and get Packet or send field with packet
    /* void _callDelegateError(uint8_t err); */
    /* void _callDelegateSuccess(uint8_t err); */

public:
    SerialPacketManager();
    ~SerialPacketManager();
    SerialPacketManager(const SerialPacketManager&) = delete;

    void addDelegate(SerialPacketDelegate* delegate);
    void setCheckSumMethod(uint8_t(*method)(uint8_t*, size_t));
    uint8_t calcCheckSum(SerialPacket& packet);
    uint8_t calcCheckSum(SerialPacket* packet);

    bool isValidPacket(SerialPacket& packet);

    // memcpy
    //uint8_t getBytePacket() const;

    static uint8_t crc8(uint8_t* data, size_t len);
    static uint8_t crc8Fast(uint8_t* data, size_t len);


    // method read from uint8_t/char array to '\0'
    // method read from uint8_t* array for num byte

    // or enum
    static const uint8_t CODE_NONE = 0;
    static const uint8_t STATE_START_WAIT = 1;
    static const uint8_t STATE_LENGTH = 2;
    static const uint8_t STATE_CRC = 3;

};

#endif // SERIALMANAGER_H
