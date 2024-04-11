#include "abstractreader.h"

AbstractReader::AbstractReader(QIODevice* device, QObject* parent) :
    QObject(parent)
{
    _device = device;
    bytesRead = 0;
}


void AbstractReader::pause(bool enabled)
{
    paused = enabled;
}

void AbstractReader::enable(bool enabled)
{
    if (enabled)
    {
        QObject::connect(_device, &QIODevice::readyRead,
                         this, &AbstractReader::onDataReady);
    }
    else
    {
        QObject::disconnect(_device, 0, this, 0);
        disconnectSinks();
    }
}


void AbstractReader::onDataReady()
{
    bytesRead += readData();
}

unsigned AbstractReader::getBytesRead()
{
    unsigned r = bytesRead;
    bytesRead = 0;
    return r;
}
