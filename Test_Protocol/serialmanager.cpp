#include "serialmanager.h"


uint8_t *SerialPacketManager::_packetByteView(SerialPacket& packet)
{
    uint8_t* ptr = (uint8_t*)&packet;
    return ++ptr;
}

SerialPacketManager::SerialPacketManager()
    : _delegates(nullptr), _countDelegates(0)
{
    this->_ckeckSumMethod = SerialPacketManager::crc8;
}

SerialPacketManager::~SerialPacketManager()
{
    delete[] _delegates;
}

void SerialPacketManager::addDelegate(SerialPacketDelegate *delegate)
{
    if (!delegate) return;
    SerialPacketDelegate** temp = new SerialPacketDelegate*[_countDelegates + 1];
    for (size_t i = 0; i < _countDelegates; ++i) {
        temp[i] = _delegates[i];
    }
    temp[_countDelegates++] = delegate;
    delete[] _delegates;
    _delegates = temp;
}

uint8_t SerialPacketManager::calcCheckSum(SerialPacket &packet)
{
    return _ckeckSumMethod(_packetByteView(packet), packet.getFullPacketSize()-1);
}

uint8_t SerialPacketManager::calcCheckSum(SerialPacket *packet)
{
    return calcCheckSum(*packet);
}

bool SerialPacketManager::isValidPacket(SerialPacket &packet)
{
    uint8_t crc = calcCheckSum(packet);
    if (packet._checkSum != crc)
        return false;
    return true;
}

uint8_t SerialPacketManager::crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    unsigned int i;
    while (len--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }
    return crc;
}


void SerialPacketManager::setCheckSumMethod(uint8_t (*method)(uint8_t *, size_t))
{
    if (!method) return;
    this->_ckeckSumMethod = method;
}



