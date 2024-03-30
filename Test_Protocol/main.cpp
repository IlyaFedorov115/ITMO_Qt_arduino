#include <iostream>
#include <cstring>
#include "qdebug.h"
#include "vtolprotocol.h"
#include "serialmanager.h"

#include <QSerialPort>
#include <QTimer>
#include <QTextStream>
#include <QCoreApplication>
#include <QThread>
#include "serialportmanager.h"
#include "mainprogram.h"

using namespace std;



int main(int argc, char *argv[])
{
    SerialPacketManager manager;
    SerialPacket packet;
    bool testCrc = false;
    bool testParse = false;
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

    if (testParse)
    {
        SerialPacket packet1;
        vtol_protocol::ProtocolMsg msg;
        msg.type = vtol_protocol::MsgProps::MSG_TYPE::PWM_SIGNAL;
        msg.lenData = vtol_protocol::MsgProps::getDataLen(msg.type);
        std::cout << "Len of msg = " << (int)msg.lenData << std::endl;
        for (int i = 0; i < msg.lenData; i++)
            msg.data[i].number = 15.034 + i;

        packet1 = vtol_protocol::Parser::parse2Serial(msg);

        std::cout << "Type of msg = " << (int)msg.type << " Type of packet = " << (int)packet1._msgType << std::endl;
        std::cout << "Len of msg = " << (int)msg.lenData << " Packet len = " << (int)packet1._dataLength << std::endl;
        float b = 0;
        memcpy(&b, packet1.buffer, packet1._dataLength);
        std::cout << "B after = " << b << std::endl;

        packet1._checkSum = manager.calcCheckSum(packet1);
        std::cout << "Check summ = " << (int)packet1._checkSum << std::endl;
        std::cout << "Is valid: " << manager.isValidPacket(packet1) << std::endl;

        vtol_protocol::ProtocolMsg msg_new;
        vtol_protocol::Parser::PARSE_CODE code = vtol_protocol::Parser::parse(packet1, msg_new);
        std::cout << "Parse code = " << (int)code << std::endl;
        std::cout << "Len of new msg = " << (int)msg_new.lenData << "Values of msg" << std::endl;
        for (int i = 0; i < msg_new.lenData; i++){
            std::cout << "i: " << msg_new.data[i].number << std::endl;
        }


    }
    /*
        QSerialPort serialPort;
        serialPort.setPortName("COM3");
        serialPort.setBaudRate(QSerialPort::Baud9600);
        QTextStream standardOutput(stdout);
*/


        QCoreApplication a(argc, argv);
        QSerialPort serialPort;
        serialPort.setPortName("COM3");
        serialPort.setBaudRate(QSerialPort::Baud115200);
        QTextStream standardOutput(stdout);
        if (!serialPort.open(QIODevice::ReadWrite)) {
                standardOutput << QObject::tr("Failed to open port %1, error: %2")
                               << "\n";
                return 1;
            }
        SerialPortManager serialPortReader(&serialPort);

        /*
        serialPortReader.sendTimerStepHW(400);
        QThread::msleep(2000);
        //return 1;
        std::cout << "Start" << std::endl;
        serialPortReader.sendStartSim();

        QTimer::singleShot(3000, nullptr, [&serialPortReader](){serialPortReader.sendStopSim();});

        //QThread::msleep(5000);
       // serialPortReader.sendStartSim();
        QTimer::singleShot(6000, nullptr, [&serialPortReader](){serialPortReader.sendStartSim();});
        */
        MainProgram prog;
        prog.setPortManager(&serialPortReader);
        prog.run();

        QTimer::singleShot(10000, nullptr, [&prog](){prog.stop();});

        return a.exec();
}
