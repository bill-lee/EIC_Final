#include "datagrabthread.h"

using namespace std;


//constructor
DataGrabThread::DataGrabThread(QObject *parent) :
    QThread(parent), rightMotor(NULL), leftMotor(NULL), lms291(NULL), stopped(true), rightOdemtryValue(0), leftOdemtryValue(0), rightCommand(0), leftCommand(0)
{

}
//destructor
DataGrabThread::~DataGrabThread()
{
    leftMotor->Stop();
    rightMotor->Stop();

}
void DataGrabThread::SetMotorPointer(DCMotor_Controller *right, DCMotor_Controller *left)
{
    rightMotor=right;
    leftMotor=left;
}
void DataGrabThread::SetLaserPointer(Laser_LMS291_Controller *laser)
{
    lms291=laser;
}

void DataGrabThread::run()
{

    stopped=false;
    commandFlag=true;
    motionType=-1;
    leftMotor->SetANSWmode(0);
    rightMotor->SetANSWmode(0);
  //  rightMotor->SetVelocity(-50);
    //leftMotor->SetVelocity(50);



    while(!stopped)
    {
        clock_t startTime=clock();
        bool flagR=false,flagL=false;
        bool sendFlag=false;

        //read odometry data
        rightMotor->GetPose();
        leftMotor->GetPose();
        lms291->TriggerLaser();


        QString leftSignal,rightSignal;

        QThread::msleep(20);


        flagR=rightMotor->ReadData(rightSignal,1024);
        flagL=leftMotor->ReadData(leftSignal,1024);



        if(flagR>0)
        {

            int rr=rightSignal.toStdString().find("\r\n");
            rightSignal.toStdString().erase(rr);

            rightOdemtryValue =rightSignal.toInt();
            sendFlag=true;


        }

        if(flagL>0)
        {

            int rr=leftSignal.toStdString().find("\r\n");
            leftSignal.toStdString().erase(rr);


            leftOdemtryValue=leftSignal.toInt();
            sendFlag=true;


        }
        //check
        if(motionType==1) //�e�i
        {
            if((-rightOdemtryValue>rightCommand||leftOdemtryValue>leftCommand)&&commandFlag==true)
            {
                cout<<"fuck "<<rightOdemtryValue<<"  "<<leftOdemtryValue<<endl;
                emit this->sendCommandAccomplishment();

            }
            //cout<<rightCommand<<" "<<leftCommand<<endl;

        }
        else if(motionType==2) //����
        {
            if(-rightOdemtryValue>rightCommand&&commandFlag==true)
            {
                emit this->sendCommandAccomplishment();
                cout<<"fuck "<<rightOdemtryValue<<"  "<<leftOdemtryValue<<endl;

            }

        }
        else if(motionType==3) //�k��
        {
            if(leftOdemtryValue>leftCommand&&commandFlag==true)
            {
                cout<<"fuck "<<rightOdemtryValue<<"  "<<leftOdemtryValue<<endl;
                emit this->sendCommandAccomplishment();

            }

        }


        std::vector<double> laserData;
        lms291->ReadData(laserData);
        clock_t endTime=clock();
        double total=(double)(endTime-startTime)/CLK_TCK;

        if(sendFlag==true&&laserData.size()>0)
        {

            cout<<"1_thread:"<<rightOdemtryValue<<" "<<leftOdemtryValue<<" time:"<<total<<endl;
            //cout<<motionType<<" "<<rightOdemtryValue<<" "<<leftOdemtryValue<<" "<<laserData.size()<<endl;
            emit this->sendOdemtryLaserData(rightOdemtryValue,leftOdemtryValue,laserData );

        }
        else
             QThread::msleep(20);


    }

}

void DataGrabThread::SetStopped(bool stop)
{

    stopped=stop;

}

void DataGrabThread::SetMotionCommand(int type, double value)
{
    motionType=type;

    if(type==1||type==2)
        rightCommand+=value;

    if(type==1||type==3)
        leftCommand+=value;


}


