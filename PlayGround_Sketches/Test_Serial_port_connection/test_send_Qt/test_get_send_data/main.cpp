#include <QCoreApplication>
#include <QSerialPort>
#include <iostream>
#include <QTextStream>
#include <QObject>
#include <QByteArray>
#include <QDebug>
union FloatType {
  char buf[4];
  float val;
};
FloatType receiveData[5];
char buffer[20];

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QByteArray arr;

    QSerialPort serialPort;
    serialPort.setPortName("COM3");
    serialPort.setBaudRate(250000); // QSerialPort::Baud115200
    QTextStream standardOutput(stdout);
    if (!serialPort.open(QIODevice::ReadWrite)) {
            standardOutput << QObject::tr("Failed to open port %1, error: %2")
                           << "\n";
            return 1;
        }

    QObject::connect(&serialPort, &QSerialPort::readyRead, [&standardOutput, &serialPort, &arr]{
        qint64 b = serialPort.bytesAvailable();
        if (b < 20) {
            return;
        }
        arr = serialPort.read(20);
        if (arr.size() < 20) {
             qDebug() << "=====BAD PACKET=====\n";
            return;
        }
        memcpy((void*)receiveData, arr.data(), sizeof(receiveData));
        std::cout << receiveData[0].val << ','
                  << receiveData[1].val << ','
                  << receiveData[2].val << ','
                  << receiveData[3].val << ','
                  << receiveData[4].val << '\n';

        //for (int i = 0; i < 5; i++) {
          //  qDebug() << receiveData[i].val << " ";
       // }
       //  qDebug() << "\n";
    });

    return a.exec();
}
