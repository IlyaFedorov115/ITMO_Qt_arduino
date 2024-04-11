#ifndef ABSTRACTREADER_H
#define ABSTRACTREADER_H

#include <QObject>
#include <QIODevice>
#include <QWidget>
#include <QTimer>
#include "sourcedata.h"


class AbstractReader : public QObject, public SourceData
{
    Q_OBJECT
public:
    explicit AbstractReader(QIODevice* device, QObject* parent = 0);

    /** widget for data fotmat pannel **///virtual QWidget* settingsWidget() = 0;

    /// Reader read if enable
    virtual void enable(bool enabled = true);

    /// Read and 'zero' the byte counter
    unsigned getBytesRead();

signals:
    void numOfChannelsChanged(unsigned);

public slots:
    /*** Pauses the reading. Don`t commit data.*/
    void pause(bool enabled);

protected:
    /// Reader read from this device in `readData()`
    QIODevice* _device;

    /// paused in `readData()`
    bool paused;

    /**
     * when `readyRead` signal from device. This is
     */
    virtual unsigned readData() = 0;

private:
    unsigned bytesRead;

private slots:
    // slot update `bytesRead` and call `readData()`
    void onDataReady();
};

#endif // ABSTRACTREADER_H
