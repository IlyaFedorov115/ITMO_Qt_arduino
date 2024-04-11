#ifndef FRAMEREADER_H
#define FRAMEREADER_H

#include <QObject>
#include "abstractreader.h"
#include "numberformat.h"

#define MAX_BYTE_SIZE_FRAME 200

class FrameReader : public AbstractReader
{
    Q_OBJECT
public:
    explicit FrameReader(QIODevice* device, QObject *parent = 0);
    // QWidget* settingsWidget();
    // settings
    unsigned numChannels() const;

    enum CHECK_PACKET_METHOD {
       CHECK_SUMM,
       CRC8,
    };

private:
    CHECK_PACKET_METHOD _checkMethod = CHECK_SUMM;
    unsigned _numChannels;
    unsigned sampleSize;
    bool settingsValid;
    QByteArray syncWord;
    bool checksumEnabled;
    bool hasSizeByte;
    bool isSizeField2B;
    unsigned frameSize;         // while use like 5*float = 20. Also can 5*floatx2
    bool debugModeEnabled;

    bool checkSettings();

    // read state related members
    unsigned sync_i; /// sync byte index to be read next
    bool gotSync;    /// indicates if sync word is captured
    bool gotSize;    /// indicates if size is captured, ignored if size byte is disabled (fixed size)

    // checking packet
    unsigned char _checkSumFrame;                      // for check sum method
    unsigned char charCurrBuffer[MAX_BYTE_SIZE_FRAME]; // for crc8 method
    unsigned charCurrBufferInd = 0;

    void reset(); /// Resets the reading state

    // pointer to current method
    double (FrameReader::*readSample)();
    template<typename T> double readSampleAs();

    /// read frame and check
    /// @note should be called if enough bytes on device
    void readFrameDataAndCheck();

    unsigned readData() override;

public:
    // settings
    void setNumChannels(unsigned);
    void setHasSizeByte(bool);
    void setFrameSize(unsigned);
    void setSyncWord(QString);
    void setCheckSumEnable(bool);
    void setDebugModeEnable(bool);

private:
    QByteArray _setSync(QString&);
    unsigned char _calcCRC();
private slots:
    void onNumberFormatChanged(NumberFormat numberFormat);
    void onNumOfChannelsChanged(unsigned value);
    void onSyncWordChanged(QByteArray);
    //void onSizeFieldChanged(unsigned);
};

#endif // FRAMEREADER_H
