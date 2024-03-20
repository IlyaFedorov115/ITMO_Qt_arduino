#include <iostream>
#include <cstring>
#include "vtolprotocol.h"
#include "serialmanager.h"


using namespace std;



int main()
{
    SerialPacketManager manager;
    SerialPacket packet;
    bool testCrc = true;

    if (testCrc)
    {
        std::cout << "Is valid: " << manager.isValidPacket(packet) << std::endl;
        packet._dataLength = 3*sizeof(float);
        packet._msgType = 1;
        float a1 = 5;
        float a2 = 16.5345;
        float a3 = 7;
        std::memcpy(packet.buffer, &a1, sizeof(a1));
        std::memcpy(packet.buffer+sizeof(a1), &a2, sizeof(a2));
        std::memcpy(packet.buffer+sizeof(a1)*2, &a3, sizeof(a3));
        uint8_t crc = manager.calcCheckSum(packet);
        packet._checkSum = crc;
        std::cout << "Crc = " << (int)crc << std::endl;
        std::cout << "Is valid: " << manager.isValidPacket(packet) << std::endl;
        packet.buffer[3] += 2;
        crc = manager.calcCheckSum(packet);
        std::cout << "Crc change = " << (int)crc << std::endl;
        std::cout << "Is valid: " << manager.isValidPacket(packet) << std::endl;
    }


    return 0;
}
