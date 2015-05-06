#ifndef LASERCONTROLLER_H
#define LASERCONTROLLER_H
#include "qextserialport.h"
#include<QString>
#include<vector>
#include"opencv2/opencv.hpp"

class LaserController
{
public:
    LaserController(QString c="COM1",int b=9600):comPortName(c),baudRate(b),parityCheck(PAR_NONE),flow(FLOW_OFF),dataBit(DATA_8),stopBits(STOP_1)
    {
         port=new  QextSerialPort;
    }
    bool Open(QString comPort,int baudRate);
    bool IsOpen(){return port->isOpen();}
    void TriggerLaser(void);
    bool ReadData(std::vector<double>& scanData);
    bool Close();
private:

    QString       comPortName;    //comport�W��
    int           baudRate;       //�j�v
    ParityType    parityCheck;    //�P���ˬd�X
    FlowType      flow;           //��������
    DataBitsType  dataBit;        //��Ƥj�p
    StopBitsType  stopBits;       //����bit

    QextSerialPort *port;        //RS232����
};

#endif // LASERCONTROLLER_H
