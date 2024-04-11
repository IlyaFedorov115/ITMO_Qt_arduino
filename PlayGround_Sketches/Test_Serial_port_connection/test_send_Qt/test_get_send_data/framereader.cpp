#include "framereader.h"
#include <QDebug>

FrameReader::FrameReader(QIODevice* device, QObject* parent) :
    AbstractReader(device, parent)
{
    paused = false;
    // initial settings
    settingsValid = false;
    _numChannels = 5;
    hasSizeByte = false;
    isSizeField2B = false;
    frameSize = 20;
    setSyncWord("23 23");
    checksumEnabled = false;
    onNumberFormatChanged(NumberFormat_float); // set sample size
    debugModeEnabled = true;
    checkSettings();

    // init connections
    // connect for widget?

    reset();
}


bool FrameReader::checkSettings()
{

    // SYNS MUST BE BECAUSE FRAME
    if (!syncWord.size()) {
        qDebug() << "Invalid Sync for frame";
        settingsValid = false;
        return settingsValid;
    }

    // must be for example float = 4 * 2;
    if (!hasSizeByte &&
            (frameSize % (_numChannels*sampleSize) != 0))
    {
        qDebug() << "Invalid frameSize for frame without sizeByte";
        settingsValid = false;
        return settingsValid;
    }

    /*
     * widget -> showAll okey or window
     */
    settingsValid = true;
    return settingsValid;

}


unsigned FrameReader::readData()
{
    unsigned numBytesRead = 0;

    if (!settingsValid) {
        qDebug() << "Can`t read. Settings wrong";
        return numBytesRead;
    }

    unsigned bytesAvailable;

    while ((bytesAvailable = _device->bytesAvailable())) {
        if (!gotSync)   // read sync start
        {
            char ch;
            _device->getChar(&ch);
            numBytesRead++;
            if (ch == syncWord[sync_i++]) // correct syns
            {
                if (sync_i == static_cast<unsigned>(syncWord.length()))
                    gotSync = true;
            }
            else {
                if (debugModeEnabled) qDebug() << "Missde syns byte";
            }
        }
        else if (hasSizeByte && !gotSize)
        {} /* At this time not implemented */
        else // read data bytes
        {
            unsigned size_tmp = (checksumEnabled ? frameSize+1 : frameSize);
            if (bytesAvailable < size_tmp) {
                break;
            } else {
                readFrameDataAndCheck();
                numBytesRead += size_tmp;
                reset();
            }
        }
    }

    return numBytesRead;

}


void FrameReader::readFrameDataAndCheck()
{
    // just read
    if (paused){
        _device->read(checksumEnabled ? frameSize+1 : frameSize);
        return;
    }

    /* IF set for example 5*float x2, -> frameSize = 40 */
    unsigned numOfPackagesToRead = frameSize / (_numChannels * sampleSize);
    SamplePack samples(numOfPackagesToRead, _numChannels);

    // MAYBE just read bytes?
    for (int i = 0; i < numOfPackagesToRead; ++i)
    {
        for (int j = 0; j < _numChannels; ++j)
            samples.data(j)[i] = (this->*readSample)();
    }

    bool checksumPassed = false;
    unsigned char currCheckSum = 0;
    if (checksumEnabled)
    {
        _device->read((char*)&currCheckSum, 1);
        if (_checkMethod == CHECK_SUMM){
            checksumPassed = (_checkSumFrame == currCheckSum);
        } else {
            checksumPassed = (currCheckSum == _calcCRC());
        }
    }

    if (!checksumEnabled || checksumPassed){
        feedOut(samples);
    } else {
        qDebug() << "CHECK SUMM FAILED!!!";
    }

}


QByteArray FrameReader::_setSync(QString& str){
    QString text = str.remove(' ');

    // format must be 2 hex
    if (text.size() % 2 == 1) {
        qDebug() << "Syns must be in format 2 hex";
        return QByteArray();
    } else {
        return QByteArray::fromHex(text.toLatin1());
    }
}

unsigned char Crc8Fast(unsigned char *pcBlock, unsigned char len);
unsigned char FrameReader::_calcCRC()
{
    if (charCurrBufferInd == 0) return 0;
    return Crc8Fast(&charCurrBuffer[0], charCurrBufferInd);
}


template<typename T> double FrameReader::readSampleAs()
{
    T data;

    _device->read((char*) &data, sizeof(data));

    if (checksumEnabled)
    {
        if (_checkMethod == CHECK_PACKET_METHOD::CHECK_SUMM){
            for (unsigned i = 0; i < sizeof(data); i++)
                _checkSumFrame += ((unsigned char*) &data)[i];
        } else {
            // crc8
            if ((charCurrBufferInd+sizeof(data)) > MAX_BYTE_SIZE_FRAME){
                qDebug() << "MAX SIZE OF PACKET ERROR!!!";
                return 0;
            }
            for (unsigned i = 0; i < sizeof(data); i++)
                charCurrBuffer[charCurrBufferInd++] = ((unsigned char*) &data)[i];
        }

    }

    //data = qFromLittleEndian(data);
    return (double)data;
}


void FrameReader::onNumberFormatChanged(NumberFormat numberFormat)
{
    switch(numberFormat)
    {
        case NumberFormat_uint8:
            sampleSize = sizeof(quint8);
            readSample = &FrameReader::readSampleAs<quint8>;
            break;
        case NumberFormat_int8:
            sampleSize = sizeof(qint8);
            readSample = &FrameReader::readSampleAs<qint8>;
            break;
        case NumberFormat_uint16:
            sampleSize = sizeof(quint16);
            readSample = &FrameReader::readSampleAs<quint16>;
            break;
        case NumberFormat_int16:
            sampleSize = sizeof(qint16);
            readSample = &FrameReader::readSampleAs<qint16>;
            break;
        case NumberFormat_uint32:
            sampleSize = sizeof(quint32);
            readSample = &FrameReader::readSampleAs<quint32>;
            break;
        case NumberFormat_int32:
            sampleSize = sizeof(qint32);
            readSample = &FrameReader::readSampleAs<qint32>;
            break;
        case NumberFormat_float:
            sampleSize = sizeof(float);
            readSample = &FrameReader::readSampleAs<float>;
            break;
        case NumberFormat_double:
            sampleSize = sizeof(double);
            readSample = &FrameReader::readSampleAs<double>;
            break;
        case NumberFormat_INVALID:
            Q_ASSERT(1); // never
            break;
    }

    checkSettings();
    reset();
}

void FrameReader::onNumOfChannelsChanged(unsigned value)
{
    _numChannels = value;
    checkSettings();
    reset();
    updateNumChannels();
    emit numOfChannelsChanged(value);
}

void FrameReader::onSyncWordChanged(QByteArray word)
{
    syncWord = word;
    checkSettings();
    reset();
}

void FrameReader::reset()
{
    sync_i = 0;
    gotSync = false;
    gotSize = false;
    if (hasSizeByte) frameSize = 0;
    _checkSumFrame = 0;
    charCurrBufferInd = 0;
}

unsigned FrameReader::numChannels() const
{
    return _numChannels;
}

void FrameReader::setNumChannels(unsigned ch)
{
    _numChannels = ch;
}

void FrameReader::setHasSizeByte(bool sb)
{
    hasSizeByte = sb;
}

void FrameReader::setFrameSize(unsigned size)
{
    frameSize = size;
}

void FrameReader::setSyncWord(QString str)
{
    this->syncWord = _setSync(str);
}

void FrameReader::setCheckSumEnable(bool en)
{
    checksumEnabled = en;
}

void FrameReader::setDebugModeEnable(bool d)
{
    debugModeEnabled = d;
}




/* =================== CRC CALCULACIOTION ==================== */

const unsigned char Crc8Table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

unsigned char Crc8Fast(unsigned char *pcBlock, unsigned char len)
{
    unsigned char crc = 0xFF;

    while (len--)
        crc = Crc8Table[crc ^ *pcBlock++];

    return crc;
}
















