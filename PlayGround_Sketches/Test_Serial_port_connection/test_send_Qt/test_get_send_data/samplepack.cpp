
#include <cstring>
#include <QtGlobal>


#include "samplepack.h"

SamplePack::SamplePack(unsigned numSamples, unsigned numChans)
{
    Q_ASSERT(numSamples > 0 && numChans > 0);

    _numSamples = numSamples;
    _numChannels = numChans;

     // packet of samples
    _yData = new double[_numSamples * _numChannels]();
}


SamplePack::SamplePack(const SamplePack& other) :
    SamplePack(other.numSamples(), other.numChannels())
{
    size_t dataSize = sizeof(double) * numSamples();

    memcpy(_yData, other._yData, dataSize * numChannels());
}


SamplePack::~SamplePack()
{
    delete[] _yData;
}


unsigned SamplePack::numChannels() const
{
    return _numChannels;
}

unsigned SamplePack::numSamples() const
{
    return _numSamples;
}



double* SamplePack::data(unsigned channel) const
{
    Q_ASSERT(channel < _numChannels);

    return &_yData[channel * _numSamples];
}


double* SamplePack::data(unsigned channel)
{
    return const_cast<double*>(static_cast<const SamplePack&>(*this).data(channel));
}

double* SamplePack::dataAll() {
    return _yData;
}
