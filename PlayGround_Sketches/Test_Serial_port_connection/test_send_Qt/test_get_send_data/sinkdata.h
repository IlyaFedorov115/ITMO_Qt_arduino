#ifndef SINKDATA_H
#define SINKDATA_H

#include <QList>
#include "samplepack.h"

class SourceData;


class SinkData
{
public:
    virtual ~SinkData() {};

    /// Connects a sink to get any data that this sink gets.
    void connectFollower(SinkData* sink);

    void disconnectFollower(SinkData* sink);

    /// Returns the connected source. `nullptr` - not connected.
    const SourceData* connectedSource() const;
    SourceData* connectedSource();

protected:
    /// call this function to feed followers.
    virtual void feedIn(const SamplePack& data);

    /// Is set by connected source. Call to update followers.
    virtual void setNumChannels(unsigned nc);

    void setSource(SourceData* s);
    friend SourceData;

private:
    QList<SinkData*> followers;
    SourceData* source = nullptr;
    unsigned _numChannels;
};



#endif // SINKDATA_H
