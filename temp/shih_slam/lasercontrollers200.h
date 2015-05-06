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
    QString       comPortName;    //comport名稱
    int           baudRate;       //鮑率
    ParityType    parityCheck;    //同位檢查碼
    FlowType      flow;           //滿載控制
    DataBitsType  dataBit;        //資料大小
    StopBitsType  stopBits;       //停止bit

    QextSerialPort *port;        //RS232指標

};

#endif // LASERCONTROLLERS200_H
