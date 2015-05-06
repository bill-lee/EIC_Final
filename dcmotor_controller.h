#ifndef DCMOTOR_CONTROLLER_H
#define DCMOTOR_CONTROLLER_H

#include <QSerialPort>          // for inheritance of QSerialport
#include <sstream>              // for std::stringstream
#include <iostream>             // for std::cout
#include <QString>

#ifndef Mypi
#define Mypi
const double pi = 3.1415926535897932384626433832795;
#endif  // Mypi

class DCMotor_Controller: public QSerialPort
{
    Q_OBJECT
public:
    DCMotor_Controller();
    ~DCMotor_Controller();
    void sethome();

    void gohome();

    void Open(const QString &comport, const int baudrate);

    void SetMaxVelocity(double v_rpm);

    void RotateRelativeDistancce(int value);

    // distance: cm
    void SetNotifyPosition(double distance);

    void SetVelocity(double speed);

    void Stop();

    void SetANSWmode(int mode);

    void GetPose();

    bool ReadData(QString& data, int num);

signals:
    void SaveOdometer(int);
private:
};

#endif // DCMOTOR_CONTROLLER_H
