#ifndef FILTERS_H
#define FILTERS_H

struct SimpleLowPass {
  float K = 0.1;
  float prevVal_ = 0.0;
  //int16_t prevVal_int = 0;
  void reset() {
    prevVal_ = 0.0;
   // prevVal_int = 0;
  }

  SimpleLowPass(){}
  SimpleLowPass(float k) : K(k) {

  }

  float filter(float val) {
    prevVal_ = prevVal_ * (1-K) + val * K;
    return prevVal_;
  }
/*
  int16_t filter(int16_t val) {
    prevVal_int = prevVal_int * (1-K) + val * K;
    return prevVal_int;
  }
*/
};



#endif