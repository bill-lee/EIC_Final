#include "motorcontroller.h"
///////////////////////////////////////////////////////
//�ܭ��n��-->�u�ݭn�[�J
//1.qextserialbase.cpp
//2.qextserialport.cpp
//3.win_qextserialport.cpp
///////////////////////////////////////////////////////
using namespace std;

bool MotorController::Open(QString comPort, int Rate)
{

    BaudRateType baudRateSymbol;
    //�Ψӿ�ܩҭn���j�v
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

    port->close();  //reset��
    port->setPortName(comPort);  //�]�wcomport
    port->setBaudRate(baudRateSymbol);  //�]�v
    port->setFlowControl(flow);  //��������
    port->setParity(parityCheck);  //�P�줸
    port->setDataBits(dataBit);  //��Ƥj�p
    port->setStopBits(stopBits);  //����bit
    port->open(QIODevice::ReadWrite| QIODevice::Unbuffered);

    return port->isOpen();
}
bool MotorController::Open(){  //�غc�禡���]�Ȥ~�i�I�s�o��

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
    port->close();  //reset��
    port->setPortName(comPortName);  //�]�wcomport
    port->setBaudRate(baudRateSymbol);  //�]�v
    port->setFlowControl(flow);  //��������
    port->setParity(parityCheck);  //�P�줸
    port->setDataBits(dataBit);  //��Ƥj�p
    port->setStopBits(stopBits);  //����bit
    port->open(QIODevice::ReadWrite);


    return port->isOpen();
}

bool MotorController::Close()
{

    stringstream  velocity;  //�ϥΦr���y
    velocity<<"V0\n";
    port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    port->close();
    return port->isOpen();
}

void MotorController::SetVelocity(int speed)
{

    if(motorType==speedMode){
        stringstream  velocity;  //�ϥΦr���y
        velocity<<"V"<<speed<<"\n";
        ////////////////////////////////////////////////////
        // �ϥ�DC motor�n�U������R�O V  (+:����  -:����)  --->�R�O�U�@���N�i�H�������
        ///////////////////////////////////////////////////
        port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    }
    else
        QMessageBox::information(0,0,"Please check your motor mode!");

}
void MotorController::SetVelocity(int speed,int node)
{

    if(motorType==speedMode){
        stringstream  velocity;  //�ϥΦr���y
        velocity<<node<<"V"<<speed<<"\n";
        ////////////////////////////////////////////////////
        // �ϥ�DC motor�n�U������R�O V  (+:����  -:����)  --->�R�O�U�@���N�i�H�������
        ///////////////////////////////////////////////////
        port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
    }
    else
        QMessageBox::information(0,0,"Please check your motor mode!");

}
void MotorController::Stop()
{
    stringstream  velocity;  //�ϥΦr���y
    velocity<<"V0\n";
    port->write(velocity.str().c_str(),strlen(velocity.str().c_str()));
}

void MotorController::SetHome() //1:DC���A���F 0: DC���F
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
    stringstream  pose;  //�ϥΦr���y
    pose << "POS\n";
    port->write(pose.str().c_str(),strlen(pose.str().c_str()));

}
void MotorController::GetPose(int node)
{
    stringstream  pose;  //�ϥΦr���y
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
    stringstream  pose;  //�ϥΦr���y
    pose<<"RESET\r";
    port->write(pose.str().c_str(),strlen(pose.str().c_str()));

}

void MotorController::SetAbsoluteAngle(double angle)
{

    stringstream  absoluteAngle;  //�ϥΦr���y
    absoluteAngle<<"LA"<<angle<<"\n";
    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));
    absoluteAngle.clear();
    absoluteAngle<<"M\r";
    port->write(absoluteAngle.str().c_str(),strlen(absoluteAngle.str().c_str()));


}

void MotorController::SetRelatedAngle(double angle)
{

    stringstream  relatedAngle;  //�ϥΦr���y
    relatedAngle<<"LR"<<angle<<"\n";
    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));
    relatedAngle.clear();
    relatedAngle<<"M\r";
    port->write(relatedAngle.str().c_str(),strlen(relatedAngle.str().c_str()));


}

void MotorController::SetNotifyPosition(double distance)
{

    stringstream  Position;  //�ϥΦr���y
    char str[25];
    itoa((int)distance, str, 10);  //���F�קK��ǰO��������

    Position<<"NP"<<str<<"\n";

    ////////////////////////////////////////////////////
    // �ϥ�DC motor�n�U������R�O V  (+:����  -:����)  --->�R�O�U�@���N�i�H�������
    ///////////////////////////////////////////////////
    port->write(Position.str().c_str(),strlen(Position.str().c_str()));


}

void MotorController::SetAnsMode(int mode)
{
    stringstream  command;  //�ϥΦr���y
    command<<"ANSW"<<mode<<"\n";

    port->write(command.str().c_str(),strlen(command.str().c_str()));


}

