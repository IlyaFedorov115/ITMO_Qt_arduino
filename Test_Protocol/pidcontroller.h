#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

/*
 *  Interface controller with one method calculate dt and error
 */

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

    double getLastPcomp();
    double getLastIcomp();
    double getLastDcomp();

private:
    double _Kp;
    double _Kd;
    double _Ki;
    double _prevError;
    double _integral;
    double _sumIntegral;
    double _sumError;
    double _Ts;

    struct history {
        double lastP = 0.0;
        double lastI = 0.0;
        double lastD = 0.0;
    };
    history _hist;
};

#endif // PIDCONTROLLER_H
