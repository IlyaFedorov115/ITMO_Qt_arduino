#include "mainprogram.h"
#include "serialportmanager.h"
#include <QDebug>
#include <QtMath>
#include <QDateTime>
#include "pidcontroller.h"
#include "vtolprotocol.h"



void MainProgram::sl_getQuartAngle(float w, float x, float y, float z)
{

}

void MainProgram::sl_getEulerAngle(float x, float y, float z)
{

}

void MainProgram::sl_getYawPitchRollAngle(float yaw, float pitch, float roll)
{
    qDebug() << "YPR: " << yaw << " " << pitch << " " << roll << " target: " << _state.targetAngle;
    qDebug() << "YPR deg: " << qRadiansToDegrees(yaw) << " "
             << qRadiansToDegrees(pitch) << " " << qRadiansToDegrees(roll) <<
                " target: " << qRadiansToDegrees(_state.targetAngle);
    if (_state.getAngleComponent() == 0) handleGetAngle(yaw);
    else if (_state.getAngleComponent() == 1) { handleGetAngle(pitch);}
    else if (_state.getAngleComponent() == 2) handleGetAngle(roll);
    else qDebug() << "Wrong id for angle component!!";
    //qRadiansToDegrees(yaw);
}

void MainProgram::sl_getRawAngle(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ)
{

}

float MainProgram::getSignalSat(float signal, float minLim, float maxLim)
{
    if (signal > maxLim) return maxLim;
    if (signal < minLim) return minLim;
    return signal;
}

MainProgram::MainProgram()
{
    // start andle = 87/71
    this->_pidController = new PidController(1.6, 0.5, 0.000);
   /* for test */
    _state.setTargetAngleDegree(35);
    _state.dtMs = 25;
    _state.start = false;
    _state.setAngleComponent(1);

    _state.signalLimitPwm.first = 1200;
    _state.signalLimitPwm.second = 2000;
    _state.signalLimitVolts.first = 0.0;
    _state.signalLimitVolts.second = 11.1;


    /* Loggers */
    _lgPid = new logging::FileLogger<4>("C:\\ITMO_2024_WORK\\ITMO_Qt_arduino\\Logs_experiments", "PID", logging::LogFileType::CSV);
    _lgPid->setColumnNames({"P", "I", "D", "time"});
    _lgMain = new logging::FileLogger<8>("C:\\ITMO_2024_WORK\\ITMO_Qt_arduino\\Logs_experiments", "PWM_ANGLE", logging::LogFileType::CSV);
    _lgMain->setColumnNames({"pwm", "pid_out", "theta_rad", "theta_deg", "err(rad)", "err(deg)", "dt", "time"});
}

void MainProgram::setPortManager(SerialPortManager *portManager)
{
    // close old??
    _portManager = portManager;
    init();
}

void MainProgram::reset()
{
    _state.setStartLastTime = false;
    this->_pidController->reset();

}

void MainProgram::init()
{
    if (!this->_portManager) return;
    QObject::connect(this->_portManager, &SerialPortManager::sig_GetEulerAngle,
                     this, &MainProgram::sl_getEulerAngle);
    QObject::connect(this->_portManager, &SerialPortManager::sig_GetQuartAngle,
                     this, &MainProgram::sl_getQuartAngle);
    QObject::connect(this->_portManager, &SerialPortManager::sig_GetRawAngle,
                     this, &MainProgram::sl_getRawAngle);
   // QObject::connect(this->_portManager, &SerialPortManager::sig_GetYawPitchRoll,
   //                  this, &MainProgram::sl_getYawPitchRollAngle);
    _portManager->setHandlerAngle(this);

}

void MainProgram::run()
{
    this->reset();
    this->_state.lastTime = QDateTime::currentMSecsSinceEpoch();
    this->_state.timeStartSim = QDateTime::currentMSecsSinceEpoch();
    this->_state.start = true;

    qDebug() << "Init HW";
    /* init Hardware */
    this->_portManager->sendAngleType(vtol_protocol::MsgProps::ANGLE_TYPE::YPR);
    this->_portManager->sendTimerStepHW(_state.dtMs);
    this->_portManager->sendStartSim();
    qDebug() << "Send start signals";
    /* loggers */
    try {
        _lgPid->startLog();
        _lgMain->startLog();
    } catch (const std::exception& e) {
        qDebug() << "Error on init logging " << e.what();
    }
}

void MainProgram::stop()
{
    this->_portManager->sendStopSim();
    this->_state.start = false;

    /* Loggers */
    _lgPid->stopLog();
    _lgMain->stopLog();
}

MainProgram::~MainProgram()
{
    delete _pidController;
    delete _lgPid;
    delete _lgMain;
}

void MainProgram::handleYawPitchRoll(float y, float p, float r)
{
    this->sl_getYawPitchRollAngle(y, p, r);
}

float MainProgram::_calcError(float targetAngle, float currAngle)
{
    float err = (_state.err_targetFisrt4Err) ? targetAngle - currAngle : currAngle - targetAngle;
    if (!_state.err_InRad) {
        return qRadiansToDegrees(err);
    }
    return err;
}

void MainProgram::handleGetAngle(float angle)
{
    /* calc PWM signal */
    if (this->_state.start) {

        // ignore first step when dt = 0
        // init TImer method?
        if (!_state.setStartLastTime) {
            _state.lastTime = QDateTime::currentMSecsSinceEpoch();
            _state.setStartLastTime = true;
            _timer.start();
            return;
        }

        /* calc error angle */
        float errAngle = _calcError(_state.targetAngle, angle);

        if(!this->_checkProtectionDegree(qRadiansToDegrees(angle))) return;  // protection

        auto dt = QDateTime::currentMSecsSinceEpoch() - this->_state.lastTime;
        auto dt2 = _timer.elapsed();
        auto timeSim = (QDateTime::currentMSecsSinceEpoch() - _state.timeStartSim) / 1000.0;

        float err2Pid = (_state.pidSendDegree) ? qRadiansToDegrees(errAngle) : errAngle;
        /* calc and send signal */
        err2Pid = errAngle;
        double pidVolt = _pidController->calculate((double)dt / 1000.0, err2Pid); // volts
        double satPid = getSignalSat(pidVolt, _state.signalLimitVolts.first, _state.signalLimitVolts.second);
        int pwmSig = map_Generic(satPid, _state.signalLimitVolts.first, _state.signalLimitVolts.second,
                                _state.signalLimitPwm.first, _state.signalLimitPwm.second);
        this->_portManager->sendPwmSignal(
            pwmSig
        );

#ifdef QT_RUNTIME_DEBUG
        qDebug() << "send pwm: " << pwmSig
                 << " PID return: " << pidVolt << " dt = " << (dt / 1000.0) << " err = " << errAngle
                 << " timeSim: " << timeSim << " dt2 = " << dt2;
#endif

        /* update time */
        this->_state.lastTime = QDateTime::currentMSecsSinceEpoch();
        _timer.restart();

        /* logging */
        if (this->isLog)
        {
            // handle slot pid -> send _pidController pointer and timeSim
            // log 2 file and to graph
            this->_lgPid->writeLog({_pidController->getLastPcomp(),
                                   _pidController->getLastIcomp(),
                                   _pidController->getLastDcomp(), timeSim});
            _logMainInfo(pwmSig, pidVolt, angle, errAngle, dt, timeSim);
        }
    }
}

bool MainProgram::_checkProtectionDegree(float angle)
{
    if (angle < _state.degreeProtection) {
        qDebug() << "====PROTECTION STOP =====";
        this->stop();
        return false;
    }
    return true;
}

void MainProgram::_logMainInfo(float pwm, float pid_out, float angleRad, float errRad, float dt, float time)
{
    // {"pwm", "pid_out", "ϴ(rad)", "ϴ(deg)", "err(rad)", "err(deg)", "dt", "time"}
    if (_lgMain){
        _lgMain->writeLog({
           pwm, pid_out, angleRad, qRadiansToDegrees(angleRad),
           errRad, qRadiansToDegrees(errRad), dt, time
        });
    }
}

void MainProgram::State::setTargetAngleDegree(float target)
{
    this->targetAngle = qDegreesToRadians(target);
}

void MainProgram::State::setTargetAngleRad(float target)
{
    this->targetAngle = target;
}

