#ifndef DATAGRABTHREAD_H
#define DATAGRABTHREAD_H
#include <QReadWriteLock>
#include <QThread>
#include <QMutex>
#include <vector>
#include <iostream>
#include <sstream>
#include <ctime>
//#include "motorcontroller.h"
//#include "lasercontrollers200.h"
#include "laser_lms291_controller.h"
#include "dcmotor_controller.h"


class DataGrabThread : public QThread
{
    Q_OBJECT
public:
    explicit DataGrabThread(QObject *parent = 0);
    virtual ~DataGrabThread();
    void SetMotorPointer(DCMotor_Controller* right, DCMotor_Controller* left);
    void SetLaserPointer(Laser_LMS291_Controller* laser);
    void SetStopped(bool stop);
    void SetMotionCommand(int type,double value);
    void SetCommandFlag(bool flag){ commandFlag=flag; }

signals:
    void sendOdemtrydData(const int right,const int left);
    void sendOdemtryLaserData(const int right,const int left,const std::vector<double>& data);
    void sendCommandAccomplishment();
    void GetRightPoseSignal();
    void GetLeftPoseSignal();
    void TriggerLaserSignal();

public slots:


protected:
     void run();

private:

    DCMotor_Controller* rightMotor;
    DCMotor_Controller* leftMotor;
    Laser_LMS291_Controller* lms291;
    volatile bool stopped;
    int rightOdemtryValue;
    int leftOdemtryValue;
    double rightCommand;
    double leftCommand;
    int motionType;  //1: forward 2:right 3:left
    bool commandFlag;






};
#endif // DATAGRABTHREAD_H
