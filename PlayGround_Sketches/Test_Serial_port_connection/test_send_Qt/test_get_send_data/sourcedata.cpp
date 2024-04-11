#include <QtGlobal>

#include "sourcedata.h"


SourceData::~SourceData()
{
    for (auto sink : sinks)
    {
        sink->setSource(nullptr);
    }
}

void SourceData::connectSink(SinkData* sink)
{
    Q_ASSERT(!sinks.contains(sink));

    auto prevSource = sink->connectedSource();
    if (prevSource != nullptr)
    {
        prevSource->disconnect(sink);
    }

    sinks.append(sink);
    sink->setSource(this);
    sink->setNumChannels(numChannels());
}

void SourceData::disconnect(SinkData* sink)
{
    Q_ASSERT(sinks.contains(sink));
    Q_ASSERT(sink->connectedSource() == this);

    sink->setSource(nullptr);
    sinks.removeOne(sink);
}

void SourceData::disconnectSinks()
{
    while (!sinks.isEmpty())
    {
        auto sink = sinks.takeFirst();
        sink->setSource(nullptr);
    }
}

void SourceData::feedOut(const SamplePack& data) const
{
    for (auto sink : sinks)
    {
        sink->feedIn(data);
    }
}

void SourceData::updateNumChannels() const
{
    for (auto sink : sinks)
    {
        sink->setNumChannels(numChannels());
    }
}

