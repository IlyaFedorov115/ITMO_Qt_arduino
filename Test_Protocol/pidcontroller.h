#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


class PidController
{
public:
    PidController();
    PidController(double kp, double ki, double kd);
    void setKpCoef(double coef);
    void setKdCoef(double coef);
    void setKiCoef(double coef);

    double getKpCoef();
    double getKdCoef();
    double getKiCoef();

    void reset();
    double calculate(double dt, double error);

private:
    double _Kp;
    double _Kd;
    double _Ki;
    double _prevError;
    double _integral;
    double _sumIntegral;
    double _sumError;
    double _Ts;
};

#endif // PIDCONTROLLER_H
