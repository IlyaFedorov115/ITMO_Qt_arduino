#ifndef SOURCEDATA_H
#define SOURCEDATA_H

#include <QList>
#include "sinkdata.h"
#include "samplepack.h"

class SourceData
{
public:
    virtual ~SourceData();
    virtual unsigned numChannels() const = 0;

    void connectSink(SinkData* sink);
    void disconnect(SinkData* sink);
    void disconnectSinks();

protected:
    /// Feeds in given data to all sinks
    virtual void feedOut(const SamplePack& data) const;
    void updateNumChannels() const;
private:
    QList<SinkData*> sinks;
};

#endif // SOURCEDATA_H
