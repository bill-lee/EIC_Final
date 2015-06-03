#ifndef DCMOTOR_CONTROLLER_H
#define DCMOTOR_CONTROLLER_H

#include <QSerialPort>          // for inheritance of QSerialport
#include <QDataStream>
#include <sstream>              // for std::stringstream
#include <iostream>             // for std::cout
#include <QString>
#include <QtConcurrent>
#include <queue>
#ifndef Mypi
#define Mypi
const double pi = 3.1415926535897932384626433832795;
#endif  // Mypi

enum command{
    OK = 0,
    NP = 1,
    POS = 2
};
class DCMotor_Controller: public QSerialPort
{
    Q_OBJECT
public:
    DCMotor_Controller();
    ~DCMotor_Controller();
    void SetHome();

    void GoHome();

    void Open(const QString &comport, const int baudrate);

    void SetMaxVelocity(double v_rpm);

    void SetMaxVelocityThread(double v_rpm);

    void RotateRelativeDistancce(int value);

    // distance: cm
    void SetNotifyPosition(double distance);

    void SetNPmode();

    void SetVelocity(double speed);

    void Stop();

    void SetANSWmode(int mode);

    void Disconnect();


    bool ReadData(QString& data, int num);
public slots:
    long GetPose();

    void Response();

signals:
    void SaveOdometer(int);

    void PositionAttained();

    void ReturnPosition(int);

private:
    std::queue<command> command_queue;
};

#endif // DCMOTOR_CONTROLLER_H
