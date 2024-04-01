#include "pidcontroller.h"

PidController::PidController()
    : PidController(0,0,0)
{

}

PidController::PidController(double kp, double ki, double kd)
    : _Kp(kp), _Kd(kd), _Ki(ki)
{
    this->reset();
}


void PidController::setKpCoef(double coef) {
    this->_Kp = coef;
}
void PidController::setKdCoef(double coef) {
    this->_Kd = coef;
}
void PidController::setKiCoef(double coef) {
    this->_Ki = coef;
}


double PidController::getKpCoef(){
    return this->_Kp;
}
double PidController::getKdCoef(){
    return this->_Kd;
}
double PidController::getKiCoef(){
    return this->_Ki;
}

void PidController::reset()
{
    _prevError = 0.0;
    _integral = 0.0;
    _sumError = 0.0;
    _sumIntegral = 0.0;
}

double PidController::calculate(double dt, double error)
{
    if (dt*1000 <= 5.0) return _hist.lastP + _hist.lastI + _hist.lastD;
    // calc proportional
    this->_hist.lastP = _Kp * error;

    // calc Integral
    _sumIntegral += error * dt;
    this->_hist.lastI = _Ki * _sumIntegral;

    // calc deriviation
    double deriv = (error - _prevError) / dt;
    this->_hist.lastD = _Kd * deriv;

    // output
    double output = _hist.lastP + _hist.lastI + _hist.lastD;

    _prevError = error;


    return output;
}


double PidController::getLastPcomp(){
    return this->_hist.lastP;
}
double PidController::getLastIcomp(){
    return this->_hist.lastI;
}
double PidController::getLastDcomp(){
    return this->_hist.lastD;
}
