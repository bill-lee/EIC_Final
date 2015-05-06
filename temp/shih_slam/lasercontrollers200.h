#ifndef LASERCONTROLLERS200_H
#define LASERCONTROLLERS200_H
#include "qextserialport.h"
#include<QString>
#include<vector>
#include<iostream>
#include<fstream>
#include"opencv2/opencv.hpp"

const double S200_MAX_RANGE=8706;// 8706cm
const double S200_ANGLE_RESOLUTION=0.5;

class LaserControllerS200
{
public:
    LaserControllerS200(QString c="COM1",int b=9600):comPortName(c),baudRate(b),parityCheck(PAR_NONE),flow(FLOW_OFF),dataBit(DATA_8),stopBits(STOP_1)
    {
      port=new  QextSerialPort;
    }
     bool Open(QString comPort,int baudRate);   //open comport
     bool IsOpen(){return port->isOpen();}      //check is opened
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

#endif // LASERCONTROLLERS200_H
