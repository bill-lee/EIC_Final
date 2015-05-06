#include "motorcontroller.h"
///////////////////////////////////////////////////////
//很重要喔-->只需要加入
//1.qextserialbase.cpp
//2.qextserialport.cpp
//3.win_qextserialport.cpp
///////////////////////////////////////////////////////
using namespace std;

bool MotorController::Open(QString comPort, int Rate)
{

    BaudRateType baudRateSymbol;
    //用來選擇所要的鮑率
    switch(Rate){
    case 9600:
        baudRateSymbol=BAUD9600;
        break;

    case 19200:
        baudRateSymbol=BAUD19200;
        break;

    case 38400:
        baudRateSymbol=BAUD38400;
        break;

    case 76800:
        baudRateSymbol=BAUD76800;
        break;

    case 115200:
        baudRateSymbol=BAUD115200;
        break;

    default:
        baudRateSymbol=BAUD9600;

    };

    port->close();  //reset用
    port->setPortName(comPort);  //設定comport
    port->setBaudRate(baudRateSymbol);  //包率
    port->setFlowControl(flow);  //滿載控制
    port->setParity(parityCheck);  //同位元
    port->setDataBits(dataBit);  //資料大小
    port->setStopBits(stopBits);  //停止bit
    port->open(QIODevice::ReadWrite| QIODevice::Unbuffered);

    return port->isOpen();
}
bool MotorController::Open(){  //建構函式有設值才可呼叫這個

    if(comPortName.isEmpty()||baudRate<0){
        QMessageBox::information(0,"Error","Please set comPort and baudRate");
        return false;
    }
    BaudRateType baudRateSymbol;
    switch(baudRate){
    case 9600:
        baudRateSymbol=BAUD9600;
        break;

    case 19200:
        baudRateSymbol=BAUD19200;
        break;

    case 38400:
        baudRateSymbol=BAUD38400;
        break;

    case 76800:
        baudRateSymbol=BAUD76800;
        break;

    case 115200:
        baudRateSymbol=BAUD115200;
        break;

    default:
        baudRateSymbol=BAUD9600;

    };
    port->close();  //reset用
    port->setPortName(comPortName);  //設定comport
    port->setBaudRate(baudRateSymbol);  //包率
    port->setFlowControl(flow);  //滿載控制
    port->setParity(parityCheck);  //同位元
    port->setDataBits(dataBit);  //資料大小
    port->setStopBits(stopBits);  //停止bit
    port->open(QIODevice::ReadWrite);


    return port->isOpen();
}

bool MotorController::Close()
{

    stringstream  velocity;  //使用字串串流
    velocity<<"V0\n";
    port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    port->close();
    return port->isOpen();
}

void MotorController::SetVelocity(int speed)
{

    if(motorType==speedMode){
        stringstream  velocity;  //使用字串串流
        velocity<<"V"<<speed<<"\n";
        ////////////////////////////////////////////////////
        // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
        ///////////////////////////////////////////////////
        port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    }
    else
        QMessageBox::information(0,0,"Please check your motor mode!");

}
void MotorController::SetVelocity(int speed,int node)
{

    if(motorType==speedMode){
        stringstream  velocity;  //使用字串串流
        velocity<<node<<"V"<<speed<<"\n";
        ////////////////////////////////////////////////////
        // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
        ///////////////////////////////////////////////////
        port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    }
    else
        QMessageBox::information(0,0,"Please check your motor mode!");

}
void MotorController::Stop()
{
    stringstream  velocity;  //使用字串串流
    velocity<<"V0\n";
    port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
}

void MotorController::SetHome() //1:DC伺服馬達 0: DC馬達
{
    const char* temp;

    if(motorType==angleMode){   //step motor have to enable to invoke the function
        temp="EN\n";
        port->write(temp,strlen(temp));
    }

    temp="HO\n";
    port->write(temp,strlen(temp));

}

void MotorController::GoHome()
{
    const char* temp;
    temp="LA0\n";
    port->write(temp,strlen(temp));
    temp="M\n";
    port->write(temp,strlen(temp));

}

void MotorController::GetPose()
{
    stringstream  pose;  //使用字串串流
    pose << "POS\n";
    port->write(pose.str().c_str(),strlen(pose.str().c_str()));

}
void MotorController::GetPose(int node)
{
    stringstream  pose;  //使用字串串流
    pose<<node<<"POS\n";
    port->write(pose.str().c_str(),strlen(pose.str().c_str()));


}
bool MotorController::ReadData(QString& data,int num)
{



    QByteArray temp=port->read(num);

    data=QString(temp);

    return !temp.isEmpty();


}

void MotorController::Reset()
{
    stringstream  pose;  //使用字串串流
    pose<<"RESET\r";
    port->write(pose.str().c_str(),strlen(pose.str().c_str()));

}

void MotorController::SetAbsoluteAngle(double angle)
{

    stringstream  absoluteAngle;  //使用字串串流
    absoluteAngle<<"LA"<<angle<<"\n";
    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));
    absoluteAngle.clear();
    absoluteAngle<<"M\r";
    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));


}

void MotorController::SetRelatedAngle(double angle)
{

    stringstream  relatedAngle;  //使用字串串流
    relatedAngle<<"LR"<<angle<<"\n";
    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));
    relatedAngle.clear();
    relatedAngle<<"M\r";
    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));


}

void MotorController::SetNotifyPosition(double distance)
{

    stringstream  Position;  //使用字串串流
    char str[25];
    itoa((int)distance, str, 10);  //為了避免科學記號的產生

    Position<<"NP"<<str<<"\n";

    ////////////////////////////////////////////////////
    // 使用DC motor要下的控制命令 V  (+:正轉  -:反轉)  --->命令下一次就可以維持轉動
    ///////////////////////////////////////////////////
    port->write(Position.str().c_str(),strlen(Position.str().c_str()));


}

void MotorController::SetAnsMode(int mode)
{
    stringstream  command;  //使用字串串流
    command<<"ANSW"<<mode<<"\n";

    port->write(command.str().c_str(),strlen(command.str().c_str()));


}

