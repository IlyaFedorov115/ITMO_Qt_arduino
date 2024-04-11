#include <QtGlobal>
#include "sinkdata.h"

void SinkData::connectFollower(SinkData* sink)
{
    Q_ASSERT(!followers.contains(sink));

    followers.append(sink);
    sink->setNumChannels(_numChannels);
}

void SinkData::disconnectFollower(SinkData* sink)
{
    Q_ASSERT(followers.contains(sink));

    followers.removeOne(sink);
}

void SinkData::feedIn(const SamplePack& data)
{
    for (auto sink : followers)
    {
        sink->feedIn(data);
    }
}


void SinkData::setNumChannels(unsigned nc)
{
    _numChannels = nc;
    for (auto sink : followers)
    {
        sink->setNumChannels(nc);
    }
}


void SinkData::setSource(SourceData* s)
{
    Q_ASSERT((source == nullptr) != (s == nullptr));
    source = s;
}

const SourceData* SinkData::connectedSource() const
{
    return source;
}


SourceData* SinkData::connectedSource()
{
    return const_cast<SourceData*>(static_cast<const SinkData&>(*this).connectedSource());
}
