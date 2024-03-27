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
    // calc proportional
    double pOut = _Kp * error;

    // calc Integral
    _sumIntegral += error * dt;
    double iOut = _Ki * _sumIntegral;

    // calc deriviation
    double deriv = (error - _prevError) / dt;
    double dOut = _Kd * deriv;

    // output
    double output = pOut + iOut + dOut;

    _prevError = error;
    return output;
}
