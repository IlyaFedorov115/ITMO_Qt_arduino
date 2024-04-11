#ifndef SAMPLEPACK_H
#define SAMPLEPACK_H

class SamplePack
{
public:
    /**
     * @param numSamples number of samples
     * @param numChans number of channels (floats in sample)
     */
    SamplePack(unsigned numSamples, unsigned numChans);
    SamplePack(const SamplePack& other);
    ~SamplePack();

    unsigned numChannels() const;
    unsigned numSamples() const;
    double* data(unsigned channel) const;

    double* data(unsigned channel);
    double* dataAll();

private:
    unsigned _numSamples, _numChannels;
    double* _yData;
};
#endif // SAMPLEPACK_H
