#ifndef MAINPROGRAM_H
#define MAINPROGRAM_H

#include <QObject>
#include <QPair>
#include <QElapsedTimer>
#include "serialportmanager.h"
#include "filelogger.h"


#define QT_RUNTIME_DEBUG


class PidController;
// maybe this public by SerialPortManager
class MainProgram : public QObject, public IHandlerAngleReceive
{
    Q_OBJECT
    struct State {
        bool start = false;
        uint dtMs = 100;              //ms
        float targetAngle = 0.0;                            // radians
        int getAngleComponent() { return _angleComponent; }
        void setAngleComponent(uint cp) { _angleComponent = (cp >= 0 && cp < 3) ? cp : 0;} // or error

        qint64 lastTime; // log and calc step for pid
        qint64 timeStartSim;
        bool setStartLastTime = false;

        qint64 delayStartControl = 0;
        QPair<uint, uint> signalLimitPwm;
        QPair<float, float> signalLimitVolts;

        void setTargetAngleDegree(float target);
        void setTargetAngleRad(float target);

        bool err_InRad = true;
        bool err_targetFisrt4Err = false;

        float degreeProtection = -71;
        bool pidSendDegree = false;

    private:
        int _angleComponent = 0; // for different mpu installation maybe yaw or pitch
    };

public slots:
    void sl_getQuartAngle(float w, float x, float y, float z);
    void sl_getEulerAngle(float x, float y, float z);                  // degree
    void sl_getYawPitchRollAngle(float yaw, float pitch, float roll);  // radians
    void sl_getRawAngle(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ);


public:
    MainProgram();
    void setPortManager(SerialPortManager* portManager);
    void reset();
    void init();
    void run();
    void stop();
    ~MainProgram();

    virtual void handleYawPitchRoll(float y, float p, float r) override;

private:
    float _calcError(float targetAngle, float currAngle); // rads;
    void handleGetAngle(float angle);
    bool _checkProtectionDegree(float angle);

    SerialPortManager* _portManager = nullptr;
    PidController* _pidController;
    State _state;
    QElapsedTimer _timer;

    /* loggers */
    bool isLog = true;
    logging::FileLogger<4>* _lgPid;
    logging::FileLogger<8>* _lgMain;
    void _logMainInfo(float pwm, float pid_out, float angleRad, float errRad, float dt, float time);

    static float getSignalSat(float signal, float minLim, float maxLim);
    template <class X, class M, class N, class O, class Q>
      int map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
        if (x > in_max) x = in_max;
        if (x < in_min) x = in_min;
        int res =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return res;
     }

};

#endif // MAINPROGRAM_H
