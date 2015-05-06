#ifndef STEPMOTOR_CONTROLLER_H
#define STEPMOTOR_CONTROLLER_H
#include <QSerialPort>          // for inheritance of QSerialport
#include "sstream"              // for std::stringstream
#include <iostream>             // for std::cout
#include <QtConcurrent>
enum Mode{
    SpeedMode,
    AngleMode
};

class StepMotor_Controller : public QSerialPort
{
    Q_OBJECT
public:
    StepMotor_Controller();
    ~StepMotor_Controller();
    // basic serial configuration
    void Open(const QString &comport, const int baudrate);
    bool IsOpen() { return this->isOpen(); }

    // control angle
    // query command

    // control
    void GoHome();
    void Stop();
    // setting
    void SetHome();
    // open notify response
    void OpenANSWMode();

    void SetMaxVelocity(double speed_rpm);
    void rotatethread();

private:
    Mode mode;
    bool stop_flag;
    QByteArray response;
signals:
    void Complete();
    void PositionAttained();

public slots:
    // input angle in degree
    void RotateAbsoluteAngle(const double abs_ang);
    void RotateRelativeAngle(const double rel_ang);
private slots:

    void Response();



};

#endif // STEPMOTOR_CONTROLLER_H
