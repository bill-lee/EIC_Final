#include "myserialport.h"

MySerialPort::~MySerialPort()
{

}

void MySerialPort::Open(const QString &comport, const int baudrate)
{
    this->QSerialPort::close();
    this->QSerialPort::setPortName(comport);
    switch (baudrate) {
    case 9600:
        this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
        break;
    case 19200:
        this->QSerialPort::setBaudRate(QSerialPort::Baud19200);
        break;
    case 38400:
        this->QSerialPort::setBaudRate(QSerialPort::Baud38400);
        break;
    case 115200:
        this->QSerialPort::setBaudRate(QSerialPort::Baud115200);
        break;
    default:
        this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
    }

    this->QSerialPort::setDataBits(QSerialPort::Data8);
    this->QSerialPort::setParity(QSerialPort::NoParity);
    this->QSerialPort::setStopBits(QSerialPort::OneStop);
    this->QSerialPort::setFlowControl(QSerialPort::NoFlowControl);
    this->QSerialPort::open(QSerialPort::ReadWrite);
}

