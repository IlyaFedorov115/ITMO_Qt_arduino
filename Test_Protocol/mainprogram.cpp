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
    this->_pidController = new PidController(0.83, 1.6, 0.0043);
   /* for test */
    _state.setTargetAngleDegree(60);
    _state.dtMs = 25;
    _state.start = false;
    _state.setAngleComponent(1);

    _state.signalLimitPwm.first = 1160;
    _state.signalLimitPwm.second = 2000;
    _state.signalLimitVolts.first = 0.0;
    _state.signalLimitVolts.second = 11.1;
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
    QObject::connect(this->_portManager, &SerialPortManager::sig_GetYawPitchRoll,
                     this, &MainProgram::sl_getYawPitchRollAngle);


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
}

void MainProgram::stop()
{
    this->_portManager->sendStopSim();
    this->_state.start = false;
}

MainProgram::~MainProgram()
{

    delete _pidController;
}

void MainProgram::handleGetAngle(float angle)
{
    if (this->_state.start) {
        // ignore first step when dt = 0
        if (!_state.setStartLastTime) {
            _state.lastTime = QDateTime::currentMSecsSinceEpoch();
            _state.setStartLastTime = true;
            return;
        }
        float err = (_state.targetFisrt4Err) ? _state.targetAngle - angle : angle - _state.targetAngle;
        auto dt = QDateTime::currentMSecsSinceEpoch() - this->_state.lastTime;

        double pidVolt = _pidController->calculate(dt / 1000.0, err); // volts
        double satPid = getSignalSat(pidVolt, _state.signalLimitVolts.first, _state.signalLimitVolts.second);
        this->_portManager->sendPwmSignal(
            map_Generic(satPid, _state.signalLimitVolts.first, _state.signalLimitVolts.second,
                        _state.signalLimitPwm.first, _state.signalLimitPwm.second)
        );

        qDebug() << "send pwm: " << map_Generic(satPid, _state.signalLimitVolts.first, _state.signalLimitVolts.second,
                                                            _state.signalLimitPwm.first, _state.signalLimitPwm.second)
                 << " PID return: " << pidVolt << " dt = " << (dt / 1000.0) << " err = " << err
                 << " timeSim: " << (QDateTime::currentMSecsSinceEpoch() - _state.timeStartSim) / 1000.0;

        this->_state.lastTime = QDateTime::currentMSecsSinceEpoch();
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

