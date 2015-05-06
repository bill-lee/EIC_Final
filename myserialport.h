#ifndef MYSERIALPORT_H
#define MYSERIALPORT_H
#include "QSerialPort"
#include <iostream>
class MySerialPort : private QSerialPort
{
public:

    MySerialPort() : QSerialPort()
    {
    }

    virtual ~MySerialPort();
    void Open(const QString &comport, const int baudrate);

private:



};

#endif // MYSERIALPORT_H
