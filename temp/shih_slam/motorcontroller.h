#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "qextserialport.h"
#include<QMessageBox>
#include<sstream>
#include<string>
#include<QString>
#include<iostream>

enum{angleMode,speedMode}; //angleMode:DC伺服馬達 speedMode: DC馬達

class MotorController
{
public:
    //建構函式只需輸入comport與鮑率
    MotorController(QString c="COM1",int b=9600,bool state=speedMode):comPortName(c),baudRate(b),motorType(state),parityCheck(PAR_NONE),flow(FLOW_OFF),dataBit(DATA_8),stopBits(STOP_1){
        port=new  QextSerialPort;  //new一個新的物件
    };
    ~MotorController(){  this->Stop();   delete port;   };

    bool Open(QString comPort,int baudRate);    //參數1:comport 參數2:鮑率
    bool Open();                                //參數1:comport 參數2:鮑率
    bool Close();
    bool IsOpen(){ return port->isOpen(); };    //會被定義為inline函式
    bool ReadData(QString& data,int num);

    //////////////////////////////////////////////////////////////////
    //  control command function
    /////////////////////////////////////////////////////////////////
    void SetHome();  //設定初始位置
    void GoHome();   //回到初始位置
    void SetMotorType(bool type){ motorType=type; };
    void SetVelocity(int speed);
    void SetVelocity(int speed,int node);  //node是為了用一條rs232控制兩科控制器   但不會解馬達回傳格式 故無法上手
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

    QString       comPortName;    //comport名稱
    int           baudRate;       //鮑率
    bool          motorType;     //馬達使用模式
    ParityType    parityCheck;    //同位檢查碼
    FlowType      flow;           //滿載控制
    DataBitsType  dataBit;        //資料大小
    StopBitsType  stopBits;       //停止bit

    QextSerialPort *port;        //RS232指標

};

#endif // MOTORCONTROLLER_H
