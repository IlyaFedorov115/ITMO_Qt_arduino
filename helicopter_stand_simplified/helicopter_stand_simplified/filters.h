#ifndef FILTERS_H
#define FILTERS_H

struct SimpleLowPass {
  float K = 0.1;
  float prevVal_ = 0.0;
  in16_t prevVal_int = 0;
  void reset() {
    prevVal_ = 0.0;
    prevVal_int = 0;
  }

  float filter(float val) {
    return prevVal_ * (1-K) + val * K;
  }

  int16_t filter(int16_t val) {
    return prevVal_int * (1-K) + val * K;
  }

};



#endif