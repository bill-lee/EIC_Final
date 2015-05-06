#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "qextserialport.h"
#include<QMessageBox>
#include<sstream>
#include<string>
#include<QString>
#include<iostream>

enum{angleMode,speedMode}; //angleMode:DC���A���F speedMode: DC���F

class MotorController
{
public:
    //�غc�禡�u�ݿ�Jcomport�P�j�v
    MotorController(QString c="COM1",int b=9600,bool state=speedMode):comPortName(c),baudRate(b),motorType(state),parityCheck(PAR_NONE),flow(FLOW_OFF),dataBit(DATA_8),stopBits(STOP_1){
        port=new  QextSerialPort;  //new�@�ӷs������
    };
    ~MotorController(){  this->Stop();   delete port;   };

    bool Open(QString comPort,int baudRate);    //�Ѽ�1:comport �Ѽ�2:�j�v
    bool Open();                                //�Ѽ�1:comport �Ѽ�2:�j�v
    bool Close();
    bool IsOpen(){ return port->isOpen(); };    //�|�Q�w�q��inline�禡
    bool ReadData(QString& data,int num);

    //////////////////////////////////////////////////////////////////
    //  control command function
    /////////////////////////////////////////////////////////////////
    void SetHome();  //�]�w��l��m
    void GoHome();   //�^���l��m
    void SetMotorType(bool type){ motorType=type; };
    void SetVelocity(int speed);
    void SetVelocity(int speed,int node);  //node�O���F�Τ@��rs232�����챱�   �����|�Ѱ��F�^�Ǯ榡 �G�L�k�W��
    void SetAbsoluteAngle(double angle);
    void SetRelatedAngle(double angle);
    void SetNotifyPosition(double distance);
    void SetAnsMode(int mode);      //0: No asynchronous responses 1: Permit asynchronous response 2: All commands with confirmation and asynchronous
    void GetPose();
    void GetPose(int node);
    void Reset();
    void Stop();

  //  void ShowPortInformation();  //

    // ShowPortInformation
private:

    QString       comPortName;    //comport�W��
    int           baudRate;       //�j�v
    bool          motorType;     //���F�ϥμҦ�
    ParityType    parityCheck;    //�P���ˬd�X
    FlowType      flow;           //��������
    DataBitsType  dataBit;        //��Ƥj�p
    StopBitsType  stopBits;       //����bit

    QextSerialPort *port;        //RS232����

};

#endif // MOTORCONTROLLER_H
