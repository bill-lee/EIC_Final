#include "eic_test.h"
#include "ui_eic_test.h"

EIC_Test::EIC_Test(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EIC_Test),
    myekfslam(new lab405::MyEFKSLAM()),
//    myrobot(new lab405::MyRobot()),
    sensorsetup(new SensorSetup())
{
    Robot_Thread = new QThread(this);
    myekfslam->myrobot->moveToThread(Robot_Thread);
    Robot_Thread->start();

    ui->setupUi(this);

    // Step Motor default
//    myrobot->step_motor->Open(sensorsetup->GetStepCOM(), sensorsetup->GetStepBaud().toInt());
//    if (myrobot->step_motor->isOpen())
//        ui->textBrowser->append("[Step]\t"
//                                + sensorsetup->GetStepCOM()
//                                + " Baud: " + sensorsetup->GetStepBaud()
//                                + "\tis opened");
//    else
//    {
//        ui->textBrowser->setTextColor(QColor(255, 0, 0));
//        ui->textBrowser->append("[Error] Cannot open "
//                                + sensorsetup->GetStepCOM()
//                                + " Baud: "
//                                + sensorsetup->GetStepBaud());
//        ui->textBrowser->setTextColor(QColor(0, 0, 0));
//    }
//    // Left DC Motor default
//    myrobot->left_dcmotor->Open(sensorsetup->GetLMotorCOM(), sensorsetup->GetLMotorBaud().toInt());
//    if (myrobot->left_dcmotor->isOpen())
//        ui->textBrowser->append("[LMotor]\t"
//                                + sensorsetup->GetLMotorCOM()
//                                + " Baud: "
//                                + sensorsetup->GetLMotorBaud()
//                                + "\tis opened");
//    else
//    {
//        ui->textBrowser->setTextColor(QColor(255, 0, 0));
//        ui->textBrowser->append("[Error] Cannot open "
//                                + sensorsetup->GetLMotorCOM()
//                                + " Baud: "
//                                + sensorsetup->GetLMotorBaud());
//        ui->textBrowser->setTextColor(QColor(0, 0, 0));
//    }
//    // Right DC Motor default
//    myrobot->right_dcmotor->Open(sensorsetup->GetRMotorCOM(), sensorsetup->GetRMotorBaud().toInt());
//    if (myrobot->right_dcmotor->isOpen())
//        ui->textBrowser->append("[RMotor]\t"
//                                + sensorsetup->GetRMotorCOM()
//                                + " Baud: "
//                                + sensorsetup->GetRMotorBaud()
//                                + "\tis opened");
//    else
//    {
//        ui->textBrowser->setTextColor(QColor(255, 0, 0));
//        ui->textBrowser->append("[Error] Cannot open "
//                                + sensorsetup->GetRMotorCOM()
//                                + " Baud: "
//                                + sensorsetup->GetRMotorBaud());
//        ui->textBrowser->setTextColor(QColor(0, 0, 0));
//    }

//    // SICK LMS291 default
//    myrobot->laser_lms291->Open(sensorsetup->GetLaserCOM(), sensorsetup->GetLaserSetBaud().toInt());
//    if (myrobot->right_dcmotor->isOpen())
//        ui->textBrowser->append("[RMotor]\t"
//                                + sensorsetup->GetLaserCOM()
//                                + " Baud: "
//                                + sensorsetup->GetLaserSetBaud()
//                                + "\tis opened");
//    else
//    {
//        ui->textBrowser->setTextColor(QColor(255, 0, 0));
//        ui->textBrowser->append("[Error] Cannot open "
//                                + sensorsetup->GetLaserCOM()
//                                + " Baud: "
//                                + sensorsetup->GetLaserSetBaud());
//        ui->textBrowser->setTextColor(QColor(0, 0, 0));
//    }
    timer = new QTimer();
    timer->setInterval(10);
    //
    connect(this, SIGNAL(test()), myekfslam->myrobot, SLOT(rotatethread()));
    // create action class for menu
    CreateAction();
    // create menu
    CreateMenus();
    // set unvisible for pushbotton_disconnect
    ui->pushButton_robot_disconnect->setVisible(false);

    ui->pushButton_19->setVisible(false);

    // shih ekf slam connect
//    connect(testEKFTimer, SIGNAL(timeout()), this, SLOT(EKF_Timer()));

    //data collect thread
//    collector=new DataGrabThread;
//    connect(collector,SIGNAL(sendOdemtryLaserData(const int ,const int ,const std::vector<double>)),this,SLOT(setOdometryLaserData(const int ,const int ,const std::vector<double>)));
//    connect(collector,SIGNAL(sendCommandAccomplishment()),this,SLOT(SetMotionCommand()));

//    SLAM_Robot = new Shih_MyRobot;
    myekfslam->myrobot = new lab405::MyRobot;
    odoValuePrevious.x=0; //x:right odo y:left odo
    odoValuePrevious.y=0;
    odoValueCurrent.x=0; //x:right odo y:left odo
    odoValueCurrent.y=0;

    //set parameters
    paused=false;
    motionMode=false;

    scenesTimer=new QTimer;
    connect(scenesTimer,SIGNAL(timeout()),this,SLOT(singleSceneAcquisition()));


}

EIC_Test::~EIC_Test()
{
//    robotOutputFile.close();
//    laserOutputFile.close();
//    robotSceneFile.close();

//    delete scenesTimer;
//    delete SLAM_Robot;

//    collector->wait();
//    delete collector;
    // wait thread for destroyed
    Robot_Thread->wait();


    delete ui;
}

void EIC_Test::on_pushButton_step_motor_clicked()
{

}

void EIC_Test::on_pushButton_step_com_clicked()
{
    myekfslam->myrobot->step_motor->Open(ui->comboBox_step_comport->currentText(), ui->comboBox_step_baudrate->currentText().toInt());
    if (myekfslam->myrobot->step_motor->isOpen())
        ui->textBrowser->append(ui->comboBox_step_comport->currentText()
                                + " Baud:" + ui->comboBox_step_baudrate->currentText()
                                + " is opened");
    else
        ui->textBrowser->append("cannot open " +
                                ui->comboBox_step_comport->currentText()
                                + " Baud:" + ui->comboBox_step_baudrate->currentText());
}

void EIC_Test::on_pushButton_relangle_clicked()
{
    if (myekfslam->myrobot->step_motor->isOpen())
    {
        myekfslam->myrobot->step_motor->RotateRelativeAngle(ui->doubleSpinBox_step_relangle->value());
        ui->textBrowser->append(QString("[relative] Step motor move %1 degrees").arg(ui->doubleSpinBox_step_relangle->value()));
    }
    else
        ui->textBrowser->append("Please open the step motor first!");

}

void EIC_Test::on_pushButton_step_absangle_clicked()
{
    if (myekfslam->myrobot->step_motor->isOpen())
    {
        myekfslam->myrobot->step_motor->RotateRelativeAngle(ui->doubleSpinBox_step_absangle->value());
        ui->textBrowser->append(QString("[absolute] Step motor move to %1 degrees").arg(ui->doubleSpinBox_step_relangle->value()));
    }
    else
        ui->textBrowser->append("Please open the step motor first!");
}

void EIC_Test::on_pushButton_laser291_open_clicked()
{
    myekfslam->myrobot->laser_lms291->Open(ui->comboBox_laser291_comport->currentText()
                   , ui->comboBox_laser291_baudrate->currentText().toInt()
                   , ui->comboBox_laser291_pre_baudrate->currentText().toInt());
    if (myekfslam->myrobot->laser_lms291->isOpen())
    {
        ui->textBrowser->append(ui->comboBox_laser291_comport->currentText()
                                + " preBaud: " + ui->comboBox_laser291_pre_baudrate->currentText()
                                + " Baud: " + ui->comboBox_laser291_baudrate->currentText()
                                + " is opened");
        // change the previous baudrate combobox index
        ui->comboBox_laser291_pre_baudrate->setCurrentIndex(ui->comboBox_laser291_baudrate->currentIndex());
    }
    else
        ui->textBrowser->append("[Error] Cannot open " +
                                ui->comboBox_laser291_comport->currentText()
                                + " Baud: " + ui->comboBox_laser291_baudrate->currentText());
}

void EIC_Test::on_pushButton_laser291_trigger_clicked()
{

//    disconnect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckOpenHeader()));
//    connect(myrobot->laser_lms291, SIGNAL(readyRead()), myrobot->laser_lms291, SLOT(CheckCommBaud()));
//    connect(myrobot->laser_lms291, SIGNAL(headcorrect()), this, SLOT(StartToGetData()));
//    myrobot->laser_lms291->TriggerContinuousMode();
    myekfslam->myrobot->laser_lms291->TriggerOneScan();
}

void EIC_Test::StartToGetData()
{
    disconnect(myekfslam->myrobot->laser_lms291, SIGNAL(readyRead()), myekfslam->myrobot->laser_lms291, SLOT(CheckCommBaud()));
    connect(myekfslam->myrobot->laser_lms291, SIGNAL(readyRead()), myekfslam->myrobot->laser_lms291, SLOT(DataSegment()));
//    connect(myrobot->laser_lms291, SIGNAL(GetOneLaserScan(QByteArray*)), myrobot->laser_lms291, SLOT(ProcessScanThread(QByteArray*)));
}

void EIC_Test::on_pushButton_laser291_stop_clicked()
{
    cv::destroyAllWindows();
    disconnect(myekfslam->myrobot->laser_lms291, SIGNAL(readyRead()), myekfslam->myrobot->laser_lms291, SLOT(DataSegment()));
    disconnect(myekfslam->myrobot->laser_lms291, SIGNAL(GetOneLaserScan(QByteArray*)), myekfslam->myrobot->laser_lms291, SLOT(ProcessScanThread(QByteArray*)));
    myekfslam->myrobot->laser_lms291->StopContinuousMode();
}

void EIC_Test::on_pushButton_step_sethome_clicked()
{
    myekfslam->myrobot->step_motor->SetHome();
    ui->textBrowser->append(QString("Set Home Done!"));
}

void EIC_Test::on_pushButton_step_gohome_clicked()
{
    myekfslam->myrobot->step_motor->GoHome();
    ui->textBrowser->append(QString("Go Home!"));
}

void EIC_Test::on_pushButton_step_stop_clicked()
{
    myekfslam->myrobot->step_motor->Stop();
    ui->textBrowser->append(QString("Stop!"));
}

void EIC_Test::on_pushButton_robot_data_acquisition_clicked()
{
    ui->lineEdit_robot_filename->setText(QDateTime::currentDateTime().toString() + QTime::currentTime().toString().replace(QString(":"), QString("")));
    ui->textBrowser->append(QString("Start data acquisition!"));
    myekfslam->myrobot->DataAcquisition(ui->doubleSpinBox_robot_startangle->value(),
                             int(ui->doubleSpinBox_robot_laser_count->value()),
                             360.0, QString("test")
                             , ui->checkBox_robot_withcolor->isChecked());
}



void EIC_Test::on_pushButton_clicked()
{
    connect(myekfslam->myrobot->step_motor, SIGNAL(PositionAttained()), myekfslam->myrobot->laser_lms291, SLOT(StartOneScan()));
    // open notify command
    myekfslam->myrobot->step_motor->OpenANSWMode();
    // go to the initial position
//    myrobot->step_motor->RotateAbsoluteAngle(0);
    myekfslam->myrobot->step_motor->RotateRelativeAngle(1.0);


}

void EIC_Test::on_pushButton_step_absangle_destroyed()
{

}

void EIC_Test::on_pushButton_2_clicked()
{

    myekfslam->myrobot->laser_lms291->TriggerOneScan();

//    const uint8_t dataheader[7] = {0x02, 0x80, 0xD6, 0x02, 0xB0, 0x69, 0x01};
//    int N = 7;
////    const uint8_t temp[7] = {0x02, 0x80, 0xD6, 0x02, 0x12, 0x69, 0x01};
//    QByteArray temp;
//    temp.append(0x02);
//    temp.append(0x80);
//    temp.append(0xD6);
//    temp.append(0x02);
//    temp.append(0xB0);
//    temp.append(0x69);
//    temp.append(0x01);

//    if (N == sizeof(dataheader))
//    {
//        std::cout << "test" << std::endl;
//        if (Laser_LMS291_Controller::CheckHeader(temp, HeaderType::DataHeader))
//            std::cout << "ok" << std::endl;
            //    }
}

void EIC_Test::CommandResponse()
{
    ui->textBrowser->append("Laser LMS291 Command Success");
}

void EIC_Test::on_pushButton_robot_stop_clicked()
{
    myekfslam->myrobot->StopDataAcquisition();
}

void EIC_Test::on_pushButton_3_clicked()
{
    myekfslam->myrobot->laser_lms291->StopContinuousMode();
}

void EIC_Test::on_pushButton_4_clicked()
{
    myekfslam->myrobot->laser_lms291->TriggerContinuousMode();

}

void EIC_Test::on_pushButton_5_clicked()
{
    myekfslam->myrobot->laser_lms291->StopContinuousMode();
}

void EIC_Test::on_pushButton_6_clicked()
{

}

void EIC_Test::on_pushButton_7_clicked()
{
    myekfslam->myrobot->laser_lms291->TriggerContinuousMode();
}

void EIC_Test::on_pushButton_8_clicked()
{
    myekfslam->myrobot->laser_lms291->clear();
}

void EIC_Test::on_pushButton_9_clicked()
{
    myekfslam->myrobot->laser_lms291->StopContinuousMode();
}

void EIC_Test::on_pushButton_robot_closeall_clicked()
{
    if (myekfslam->myrobot->laser_lms291->isOpen())
        myekfslam->myrobot->laser_lms291->close();
    if (myekfslam->myrobot->step_motor->isOpen())
        myekfslam->myrobot->step_motor->close();
}

void EIC_Test::ShowCameraOpened()
{
    ui->textBrowser->append("[Debug] Camera is opened!");
}

void EIC_Test::ShowFinishedDataAcquisition(int _size)
{
    ui->textBrowser->append(QString("[Finished] Data acquisition total: buffer size = %1").arg(_size));
}

void EIC_Test::ShowDataSegement(int count)
{
    ui->textBrowser->append(QString("[Debug] Data segment: %1 lasercount").arg(count));
}

void EIC_Test::ShowPushBuffer(int count)
{
    ui->textBrowser->append(QString("[Debug] Push to buffer: %1 lasercount").arg(count));
}

void EIC_Test::ShowFinishOneScan(int count, bool laser, bool image)
{
    ui->textBrowser->append(QString("[Debug]-Finish %1 laser: %2, image: %3-").arg(count).arg(laser).arg(image));
}

void EIC_Test::ShowFinishWriteData()
{
    ui->textBrowser->append("Write Data Finished!");
}

void EIC_Test::on_pushButton_12_clicked()
{
    // create image folder
    QString image_foldername = QString("./") + ui->lineEdit_robot_filename->text();
    if(!QDir(image_foldername).exists()) //check folder exist
        QDir().mkdir(image_foldername);
    // open camera
    cv::VideoCapture capture;
    if(!capture.isOpened())
    {
        capture.open(ui->lineEdit_robot_camera_num->text().toInt());
    }
    if (capture.isOpened())
    {
        ui->textBrowser->append("Camera is opened!");
    }

    cv::Mat img, frame;
    capture >> img;  // capture image from camera

    if(img.empty())
    {
        ui->textBrowser->append("img is empty!");
        return;
    }
//    cv::undistort(img, frame, cameraIntrinsicMatrix, distortionCoefficients, cameraIntrinsicMatrix );
    // undistort calibration
    QString imgName = image_foldername + QString("/test.jpg");

    cv::imwrite(imgName.toStdString(), img);
}

void EIC_Test::on_pushButton_dcmotor_right_comport_clicked()
{
    myekfslam->myrobot->right_dcmotor->Open(ui->comboBox_dcmotor_right_comport->currentText(),
                                 ui->comboBox_dcmotor_right_baudrate->currentText().toInt());
    if (myekfslam->myrobot->right_dcmotor->isOpen())
        ui->textBrowser->append(
                    "Right DC Motor: "
                    + ui->comboBox_dcmotor_right_comport->currentText()
                    + " Baud:" + ui->comboBox_dcmotor_right_baudrate->currentText()
                    + " is opened");
    else
        ui->textBrowser->append("[Error] Right DC Motor: cannot open " +
                                ui->comboBox_dcmotor_right_comport->currentText()
                                + " Baud:" + ui->comboBox_dcmotor_right_baudrate->currentText());
}

void EIC_Test::on_pushButton_dcmotor_left_comport_clicked()
{
    myekfslam->myrobot->left_dcmotor->Open(ui->comboBox_dcmotor_left_comport->currentText(),
                                ui->comboBox_dcmotor_left_baudrate->currentText().toInt());
    if (myekfslam->myrobot->left_dcmotor->isOpen())
        ui->textBrowser->append(
                    "Right DC Motor: "
                    + ui->comboBox_dcmotor_left_comport->currentText()
                    + " Baud:" + ui->comboBox_dcmotor_left_baudrate->currentText()
                    + " is opened");
    else
        ui->textBrowser->append("[Error] Right DC Motor: cannot open " +
                                ui->comboBox_dcmotor_left_comport->currentText()
                                + " Baud:" + ui->comboBox_dcmotor_left_baudrate->currentText());
}

void EIC_Test::on_pushButton_dcmotor_front_clicked()
{
    myekfslam->myrobot->odo_filename = ui->lineEdit_dcmotor_odometer->text().toStdString();
    if (myekfslam->myrobot->right_dcmotor->isOpen() && myekfslam->myrobot->left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Go Forward %1m").arg(ui->doubleSpinBox_dcmotor_distance->value()));
        myekfslam->myrobot->GoForward(ui->doubleSpinBox_dcmotor_distance->value(),
                           ui->doubleSpinBox_dcmotor_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(myekfslam->myrobot->right_dcmotor->isOpen())
                                .arg(myekfslam->myrobot->left_dcmotor->isOpen()));
    }
//    ui->textEdit->append("=====================");
//    RightMotor->SetVelocity(0);
//    LeftMotor->SetVelocity(0);
//    double speed=ui->label_13->text().toDouble();
//    double position=ui->doubleSpinBox_5->value();  //cm



//    if(position!=0)
//    {
//        cout<<"set1:"<<position<<endl;

//        ui->textEdit->append("Go forward:"+QString::number(position)+"cm");

//        //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:325]
//        position=(4096*3.333*14)*(position/32.5)/(CV_PI);

//        cout<<"set2:"<<position<<endl;

//        RightMotor->SetNotifyPosition(-position);
//        LeftMotor->SetNotifyPosition(position);

//        cv::waitKey(100);


//    }

//    LeftMotor->SetVelocity(speed);
//    RightMotor->SetVelocity(-speed);




//    QString flag="DC motor : Front!  speed:"+QString::number(speed);

//    ui->textEdit->append(flag);
//    ui->textEdit->append("=====================");
}

void EIC_Test::on_pushButton_dcmotor_back_clicked()
{
    myekfslam->myrobot->odo_filename = ui->lineEdit_dcmotor_odometer->text().toStdString();
    if (myekfslam->myrobot->right_dcmotor->isOpen() && myekfslam->myrobot->left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Go Backward %1m").arg(ui->doubleSpinBox_dcmotor_distance->value()));
        myekfslam->myrobot->GoBackward(ui->doubleSpinBox_dcmotor_distance->value(),
                           ui->doubleSpinBox_dcmotor_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(myekfslam->myrobot->right_dcmotor->isOpen())
                                .arg(myekfslam->myrobot->left_dcmotor->isOpen()));
    }
}

void EIC_Test::on_pushButton_dcmotor_left_clicked()
{
    myekfslam->myrobot->odo_filename = ui->lineEdit_dcmotor_odometer->text().toStdString();
    if (myekfslam->myrobot->right_dcmotor->isOpen() && myekfslam->myrobot->left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Turn Left %1 degrees").arg(ui->doubleSpinBox_dcmotor_angle->value()));
        myekfslam->myrobot->TurnLeft(ui->doubleSpinBox_dcmotor_angle->value(),
                          ui->doubleSpinBox_dcmotor_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(myekfslam->myrobot->right_dcmotor->isOpen())
                                .arg(myekfslam->myrobot->left_dcmotor->isOpen()));
    }
}

void EIC_Test::on_pushButton_dcmotor_right_clicked()
{
    myekfslam->myrobot->odo_filename = ui->lineEdit_dcmotor_odometer->text().toStdString();
    if (myekfslam->myrobot->right_dcmotor->isOpen() && myekfslam->myrobot->left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Turn Right %1 degrees").arg(ui->doubleSpinBox_dcmotor_angle->value()));
        myekfslam->myrobot->TurnRight(ui->doubleSpinBox_dcmotor_angle->value(),
                          ui->doubleSpinBox_dcmotor_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(myekfslam->myrobot->right_dcmotor->isOpen())
                                .arg(myekfslam->myrobot->left_dcmotor->isOpen()));
    }
}

void EIC_Test::on_pushButton_dcmotor_stop_clicked()
{
    if (myekfslam->myrobot->right_dcmotor->isOpen() && myekfslam->myrobot->left_dcmotor->isOpen())
    {
        myekfslam->myrobot->right_dcmotor->Stop();
        myekfslam->myrobot->left_dcmotor->Stop();
        ui->textBrowser->append(QString("Stop"));
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(myekfslam->myrobot->right_dcmotor->isOpen())
                                .arg(myekfslam->myrobot->left_dcmotor->isOpen()));
    }
}

void EIC_Test::CreateAction()
{
    setupAct = new QAction(tr("&Setup"), this);
    setupAct->setShortcuts(QKeySequence::New);
    setupAct->setStatusTip(tr("Open sensor setup menu"));
    connect(setupAct, SIGNAL(triggered()), sensorsetup, SLOT(show()));
}

void EIC_Test::CreateMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));

    sensorMenu = menuBar()->addMenu(tr("&Sensor"));
    sensorMenu->addAction(setupAct);
}

void EIC_Test::FindCurrentNodeEnd(const cv::Mat &gridMap, double intervalDistance, const cv::Point2d &currentStart, const cv::Point2d &goal, cv::Point2d &currentEnd)
{
    double min=10000;
    cv::Point2d nodeTemp;

    nodeTemp.x=currentStart.x+10;
    nodeTemp.y=currentStart.y;

    cout<<"currentStart"<<currentStart<<" "<<gridMap.cols<<gridMap.rows<<endl;
    cout<<"FinalEnd"<<goal.x<<" "<<goal.y<<endl;
    int box_size=intervalDistance;  //128 pixels, each pixel*gridsize
     ///////////// picture oversize constraint
     for(int i=0;i!=(box_size+1);++i)
         for(int j=-box_size;j!=(box_size+1);++j)
         {
             int x=i+currentStart.x;
             int y=j+currentStart.y;


             double temp=gridMap.ptr<uchar>(y)[x];
             if(temp==253)  //1:occ 127:unknow 253:free
             {

                 double suby=abs((double)goal.y-y);
                 double subx=abs((double)goal.x-x);
                 double r=sqrt(pow(subx,2.0)+pow(suby,2.0));
                   //cout<<r<<" "<<subx<<" "<<suby<<endl;
                 if(min>r)
                 {


                     nodeTemp.x=x;
                     nodeTemp.y=y;
                     min=r;
                 }

             }

         }
  //  return cv::Point2d(nodeTemp.x,nodeTemp.y);

    currentEnd.x=nodeTemp.x;
    currentEnd.y=nodeTemp.y;
}


void EIC_Test::on_pushButton_robot_trigger_clicked()
{
    if (!ui->pushButton_robot_connect->isVisible())
    {
        ui->progressBar_robot->setMaximum(ui->doubleSpinBox_robot_laser_count->value() - 1);
        myekfslam->myrobot->DataAcquisitionConti(ui->doubleSpinBox_robot_startangle->value(),
                                      int(ui->doubleSpinBox_robot_laser_count->value()),
                                      ui->doubleSpinBox_robot_endangle->value(),
                                      ui->lineEdit_robot_filename->text(),
                                      ui->checkBox_robot_withcolor->isChecked(),
                                      ui->lineEdit_robot_camera_num->text().toInt());
    }
    else
    {
        ui->textBrowser->setTextColor(QColor(255, 0, 0));
        ui->textBrowser->append("Please Connect First!");
        ui->textBrowser->setTextColor(QColor(0, 0, 0));
    }

}

void EIC_Test::on_pushButton_robot_disconnect_clicked()
{
    disconnect(myekfslam->myrobot, SIGNAL(CameraOpened()), this, SLOT(ShowCameraOpened()));
    disconnect(myekfslam->myrobot, SIGNAL(ProgressBarSignal(int)), ui->progressBar_robot, SLOT(setValue(int)));
//    disconnect(myrobot, SIGNAL(DatasegmentSignal(int)), this, SLOT(ShowDataSegement(int)));
//    disconnect(myrobot, SIGNAL(PushBufferSignal(int)), this, SLOT(ShowPushBuffer(int)));
    disconnect(myekfslam->myrobot, SIGNAL(FinishedDataAquisition(int)), this, SLOT(ShowFinishedDataAcquisition(int)));
    disconnect(myekfslam->myrobot, SIGNAL(CheckFinishOneScan(int, bool, bool)), this, SLOT(ShowFinishOneScan(int, bool, bool)));
    disconnect(myekfslam->myrobot, SIGNAL(CompleteWriteData()), this, SLOT(ShowFinishWriteData()));
    myekfslam->myrobot->DisconnectDataAcquisition(ui->checkBox_robot_withcolor->isChecked());

    ui->textBrowser->append(QString("Disconnect Data Aquisition!"));

    ui->pushButton_robot_connect->setVisible(true);
    ui->pushButton_robot_disconnect->setVisible(false);
}

void EIC_Test::on_pushButton_robot_connect_clicked()
{
    if (myekfslam->myrobot->laser_lms291->isOpen() && myekfslam->myrobot->step_motor->isOpen())
    {
        connect(myekfslam->myrobot, SIGNAL(CameraOpened()), this, SLOT(ShowCameraOpened()));
        connect(myekfslam->myrobot, SIGNAL(ProgressBarSignal(int)), ui->progressBar_robot, SLOT(setValue(int)));
        connect(myekfslam->myrobot, SIGNAL(FinishedDataAquisition(int)), this, SLOT(ShowFinishedDataAcquisition(int)));
//        connect(myrobot, SIGNAL(DatasegmentSignal(int)), this, SLOT(ShowDataSegement(int)));
//        connect(myrobot, SIGNAL(PushBufferSignal(int)), this, SLOT(ShowPushBuffer(int)));
        connect(myekfslam->myrobot, SIGNAL(CheckFinishOneScan(int, bool, bool)), this, SLOT(ShowFinishOneScan(int, bool, bool)));
        connect(myekfslam->myrobot, SIGNAL(CompleteWriteData()), this, SLOT(ShowFinishWriteData()));

        myekfslam->myrobot->ConnectDataAcquisition(ui->checkBox_robot_withcolor->isChecked());
        ui->textBrowser->append(QString("Connect Finished!"));

        ui->pushButton_robot_connect->setVisible(false);
        ui->pushButton_robot_disconnect->setVisible(true);
    }
    else
    {
        ui->textBrowser->setTextColor(QColor(255, 0, 0));
        ui->textBrowser->append(QString("Please Open Devices First! laser: %1, step: %2").arg(myekfslam->myrobot->laser_lms291->isOpen()).arg(myekfslam->myrobot->step_motor->isOpen()));
        ui->textBrowser->setTextColor(QColor(0, 0, 0));
    }
}

void EIC_Test::on_pushButton_robot_stoptrigger_clicked()
{
    myekfslam->myrobot->StopDataAcquisitionConti();
    ui->textBrowser->append(QString("Stop Data Acquisition!"));
}

void EIC_Test::on_pushButton_10_clicked()
{
    myekfslam->myrobot->ConnectSpinData();
    myekfslam->myrobot->SpinData();
    myekfslam->myrobot->step_motor->SetMaxVelocity(100);
    myekfslam->myrobot->step_motor->RotateRelativeAngle(360);

//    // for allowing boost::shared_ptr legal for connect
//    qRegisterMetaType<boost::shared_ptr<cv::Mat>> ("boost::shared_ptr<cv::Mat>");
//    connect(myrobot, SIGNAL(ImageReadyToSave(boost::shared_ptr<cv::Mat>)),
//            myrobot, SLOT(SaveImageThread(boost::shared_ptr<cv::Mat>)));

//    connect(timer, SIGNAL(timeout()), myrobot, SLOT(TakePictureThread()));
//    myrobot->test();
//    timer->start();
//    myrobot->step_motor->RotateRelativeAngle(360);
}

void EIC_Test::on_pushButton_SLAM_clicked()
{

    qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
    connect(myekfslam->myrobot->laser_slam, SIGNAL(GetContiOneScan(boost::shared_ptr<QByteArray>)),
           myekfslam->myrobot->laser_slam, SLOT(ShowScan(boost::shared_ptr<QByteArray>)));
    myekfslam->myrobot->laser_slam->SetContiOutput(true);

    myekfslam->myrobot->laser_slam->TriggerContinuousMode();


    //    myrobot->laser_slam->TriggerOneScan();

}

void EIC_Test::on_pushButton_laser_slam_open_clicked()
{
    myekfslam->myrobot->laser_slam->Open(ui->comboBox_laser_slam_comport->currentText()
                   , ui->comboBox_laser_slam_baudrate->currentText().toInt()
                   , ui->comboBox_laser_slam_pre_baudrate->currentText().toInt());
    if (myekfslam->myrobot->laser_slam->isOpen())
    {
        ui->textBrowser->append(ui->comboBox_laser_slam_comport->currentText()
                                + " preBaud: " + ui->comboBox_laser_slam_pre_baudrate->currentText()
                                + " Baud: " + ui->comboBox_laser_slam_baudrate->currentText()
                                + " is opened");
        // change the previous baudrate combobox index
        ui->comboBox_laser_slam_pre_baudrate->setCurrentIndex(ui->comboBox_laser_slam_baudrate->currentIndex());
    }
    else
        ui->textBrowser->append("[Error] Cannot open " +
                                ui->comboBox_laser_slam_comport->currentText()
                                + " Baud: " + ui->comboBox_laser_slam_baudrate->currentText());
}

void EIC_Test::on_pushButton_SLAM_2_clicked()
{
    myekfslam->myrobot->laser_slam->StopContinuousMode();
}

void EIC_Test::on_pushButton_11_clicked()
{
//    std::vector<std::vector<double> > laserSets;
//    std::vector<cv::Point2d> odoSets;
//    LoadLaserData("./laserFile1.txt",laserSets);
//    LoadOdomData("./robotFile1.txt",odoSets);


//    LandmarkMapping maper;
//    LineExtraction extrator(50);
//    CornerExtraction extractor1;
//    OccupancyGridMapping gridMapperTest(800,800,4,1,3000,(double)CV_PI*(3.0/2.0),0.06);
//    EKF_SLAM EKFer;
//    gridMapperTest.InitGridMap();



//     vector<vector<Line> > lines;


//     for(int i=0;i!=5;++i)
//     {
//         std::vector<cv::Point2d> points;
//        std::vector<Line> line;
//        std::vector<Corner> cor;
//        maper.RangesDataToPointsData(laserSets[i],points);  //[0] the same data toss three times
//        extrator.SplitAndMerge(points,line);
//        lines.push_back(line);

//        extractor1.ExtractCorners(laserSets[i],cor);

//        cout<<"line num:"<<line.size()<<endl;
//        for(int j=0;j!=line.size();++j)
//            cout<<i<<""<<j<<" "<<line[j].lineMean<<endl;

//        cv::imshow("1313",extractor1.cornerImg);
//        cv::waitKey(0);

//     }



//     EKFer.Initial(lines);



//   // }

//     /*
//        for(int h=0;h!=line.size();++h)
//        {
//            Feature test;
//            test.featureMean=line[h].lineMean;
//            test.featureCovariance=line[h].lineCovariance;
//            EKFer.AddNewLandmark(test);

//        }

//*/








//    for(int i=5;i !=laserSets.size();++i)
//    {

//        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
//        cout << "No. " << i - 10 << " Scan   encorder order: " << odoSets[i] << endl;
//        //gridMapperTest.GetLocalGridMap(laserSets[i],img);
//        std::vector<cv::Point2d> points;
//        std::vector<Line> line;
//        std::vector<Corner> cor;
//        maper.RangesDataToPointsData(laserSets[i],points);
//        extrator.SplitAndMerge(points,line);





//        EKFer.MotionPrediction(odoSets[i]);

//        RobotState robotpath=EKFer.GetRobotPose();




//        for(int i=0;i!=line.size();++i)
//        {
//            RobotState tst;
//            cout << "No. " << i << " Line Feature " << line[i].lineMean << endl;
//            //SLAM_Robot.mapper.DrawLine(line[i],robotpath ,cv::Scalar(255,255,0),1,cv::Point2d(150,500),1,landmark);

//        }


//        cout << "========================" << endl;
//        EKFer.DataAssociation(line, cor);


//        cv::Mat img=maper.GetLocalLandmarkMap(points,line,vector<Corner>());
//        cv::Mat img1;

//        gridMapperTest.GetLocalGridMap(laserSets[i],img1);
//        //gridMapperTest.InsertLocalGridMap();

//        maper.InsertLocalLandmarkMap(points,robotpath);
//        cv::Mat landmark=maper.GetLandmarkMap();

//        vector<Feature> map;
//        EKFer.GetLandmarkSets(map);
//        cout << "========================" << endl;
//        cout << "Current Map Number: " << map.size() << endl;
//        cout << "Current Line Feature Size: " << line.size() << endl;
//        for(int i=0;i!=map.size();++i)
//        {
//            cout<<"No. " << i <<" Map "<<map[i].featureMean<<endl;
//            RobotState tst;
//            //SLAM_Robot.mapper.DrawLine(map[i],tst ,cv::Scalar(0,255,0),1,cv::Point2d(150,500),1,landmark);

//        }
//        cout<<"-------------------------------------"<<endl;
//        RobotState robot=EKFer.GetRobotPose();
//        maper.DrawRobotPoseWithErrorEllipse(robot,landmark);

//        cv::imshow("123",img);
//        cv::imshow("local girdmapping ",img1);
//        cv::imshow("landmark",landmark);
//        cv::waitKey(0);
//        cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;

//    }
}

void EIC_Test::on_pushButton_15_clicked()
{
//    vector<vector<double> > laserSets;
//    vector<cv::Point2d> odoSets;
//    LoadLaserData("z0606_EKF_laserFile.txt",laserSets);  //laserFile1   z0606_EKF_laserFile  orak_EKF_laserFile
//    LoadOdomData("z0606_EKF_odoFile.txt",odoSets);  //robotFile1   z0606_EKF_odoFile  orak_EKF_odoFile
//    EKF_SLAM  EKFer;
//    LandmarkMapping mapper;
//    OccupancyGridMapping gridMapper(1000,1000,4,2,3000,(double)CV_PI*(3.0/2.0),0.05);
//    LineExtraction extrator(10);
//    CornerExtraction cornerEx;
//    PathPlanning planner;
//    Shi
//    ShiMyRobot funcionRobot;

//    ICP icper;
//    icper.Intial();




//    cv::Mat img2(1000,1000,CV_8UC3);
//    img2=cv::Scalar::all(0);
//    cv::Mat gridMap;
//    RobotState robotPosition;
//    robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
//    robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
//    robotPosition.robotPositionMean.ptr<double>(2)[0]=0;

//    double robotWidtht=58;



//    std::vector<cv::Point2d> map1;
//    map1.reserve(10000);


//    ///////////////////////
//    //EKF Intial

//    vector<vector<Line> > lines;


//    ///////////////////////
//    //set start and end node
//    cv::Point2d finalEnd;
//    finalEnd.x=400;
//    finalEnd.y=500;

//    cv::Point2d currentStart;
//    currentStart.x=gridMapper.GetGridMapOriginalPoint().x;
//    currentStart.y=gridMapper.GetGridMapOriginalPoint().y;


//    for(int i=0;i!=6;++i)
//    {
//        std::vector<cv::Point2d> points;
//        std::vector<Line> line;
//        std::vector<Corner> cor;
//        mapper.RangesDataToPointsData(laserSets[i],points);  //[0] ¦P¼Ë¤@µ§¸ê®Æ¥á¤T¦¸
//        extrator.SplitAndMerge(points,line);
//       // cornerEx.ExtractCorners(laserSets[i],cor);
//        lines.push_back(line);




//    }

//    EKFer.Initial(lines);




//    ////////////////////////////////////
//    //for data analysis
//    ofstream outnum;
//    outnum.open("number.txt");
//    if(!outnum)
//        cerr<<"error"<<endl;


//    for(int i=6;i !=laserSets.size()-10;i++)
//    {
//         clock_t startTime=clock();
//        std::vector<cv::Point2d> points,points_pre;

//        std::vector<Line> line;
//        std::vector<Corner> cor;


//        mapper.RangesDataToPointsData(laserSets[i],points);  //change
//       // mapper.RangesDataToPointsData(laserSets[i-1],map1);
//        extrator.SplitAndMerge(points,line);


//        //motion prediction
//        double left=odoSets[i].x;
//        double right=odoSets[i].y;
//        EKFer.MotionPrediction(cv::Point2d(right,left));


///*
//        double leftOdeValue=odoSets[i].x;
//        double rightOdeValue=odoSets[i].y;
//        double tempX=(leftOdeValue+rightOdeValue)/2.0*cos(robotPosition.robotPositionMean.ptr<double>(2)[0]+(leftOdeValue-rightOdeValue)*1/(2*robotWidtht));
//        double tempY=(leftOdeValue+rightOdeValue)/2.0*sin(robotPosition.robotPositionMean.ptr<double>(2)[0]+(leftOdeValue-rightOdeValue)*1/(2*robotWidtht));
//        double tempThtea=((leftOdeValue-rightOdeValue)/robotWidtht)*1;
//*/


//        EKFer.count=0;

//        EKFer.DataAssociation(line,cor);

//        robotPosition=EKFer.GetRobotPose();




//        int ummm=EKFer.GetLandmarkNum();
//        outnum<<ummm<<" "<<EKFer.count<<" "<<line.size()<<endl;

//        ////////////////////////////////////////////////////////////////////////
//        //ICP correction
//        /*
//        robotPosition.robotPositionMean.ptr<double>(0)[0]+=tempX;
//        robotPosition.robotPositionMean.ptr<double>(1)[0]+=tempY;
//        robotPosition.robotPositionMean.ptr<double>(2)[0]+=tempThtea;
//        */
//        cout<<"predict motion: "<<robotPosition.robotPositionMean<<endl;

//        cv::Mat imgICP(1000,1000,CV_8UC3);
//        imgICP=cv::Scalar::all(0);
//        mrpt::slam::CSimplePointsMap m1,m2;

//        mrpt::poses::CPose2D initialPose(0,0,0);

//        vector<cv::Point2d> temp;
//        temp.reserve(points.size());
//        for(int j=0;j!=points.size();j++)  //this is  j
//        {
//           // cv::Point2d temp;

//            //temp.x=cos(phi)*points[i].x-sin(phi)*points[i].y+x;
//            //temp.y=sin(phi)*points[i].x+cos(phi)*points[i].y+y;

//            double x=cos(robotPosition.robotPositionMean.ptr<double>(2)[0])*points[j].x-sin(robotPosition.robotPositionMean.ptr<double>(2)[0])*points[j].y+robotPosition.robotPositionMean.ptr<double>(0)[0];
//            double y=sin(robotPosition.robotPositionMean.ptr<double>(2)[0])*points[j].x+cos(robotPosition.robotPositionMean.ptr<double>(2)[0])*points[j].y+robotPosition.robotPositionMean.ptr<double>(1)[0];



//            temp.push_back(cv::Point2d(x,y));

//            //cv::circle(imgICP, cv::Point(points[j].x/100.0*(1.0/0.05)+500,points[j].y/100.0*(1.0/0.05)+500), 1, cv::Scalar(0,255,0), -1  );
//            cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(0,0,255), -1  );
//        }


//        double percent=0.5;  //0.5
//        for(int j=map1.size()*percent;j!=map1.size();j++)
//        {


//            cv::circle(imgICP, cv::Point(map1[j].x/100.0*(1.0/0.05)+500,map1[j].y/100.0*(1.0/0.05)+500), 1, cv::Scalar(0,255,0), -1  );



//         }
//        //mapper.InsertLocalLandmarkMap(points,robotPosition);
//        //imgICP=mapper.GetLandmarkMap();
//       // cv::Point3d adjustPose=icper.Align(map1,temp,percent);


//        cv::Point3d adjustPose;
//        adjustPose.x=0;
//        adjustPose.y=0;
//        adjustPose.z=0;



//        //­×¥¿«áªº¾÷¾¹¤H¦ì¸m
//        RobotState robotpath1;

//        robotpath1.robotPositionMean.ptr<double>(0)[0]=adjustPose.x+robotPosition.robotPositionMean.ptr<double>(0)[0];
//        robotpath1.robotPositionMean.ptr<double>(1)[0]=adjustPose.y+robotPosition.robotPositionMean.ptr<double>(1)[0];
//        robotpath1.robotPositionMean.ptr<double>(2)[0]=adjustPose.z+robotPosition.robotPositionMean.ptr<double>(2)[0];

//        robotpath1.robotPositionCovariance=robotPosition.robotPositionCovariance.clone();

//        EKFer.SetRobotPose(robotpath1);
//        //robotpath1.robotPositionMean=    robotPosition.robotPositionMean.clone();


//        cout<<"encoder:"<<odoSets[i]<<endl;
//        cout << "Mean of estimation: " << adjustPose << endl;
//        cout<<"update motion: "<<robotpath1.robotPositionMean<<endl;


//        for(int k=0;k!=points.size();++k){



//            double x=cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x-sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(0)[0];
//            double y=sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x+cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(1)[0];
//                 //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
//            map1.push_back(cv::Point2d(x,y));
//            cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

//        }





//       // robotPosition.robotPositionMean= robotpath1.robotPositionMean.clone();


//       ////////////////////////////////////////////////////////////////////////
//        //mapping process
//        cv::Mat localMap;
//        mapper.InsertLocalLandmarkMap(points,robotpath1);
//        gridMapper.InsertLocalGridMap(laserSets[i],robotpath1);
//        gridMapper.GetLocalGridMap(laserSets[i],localMap);
//        img2=mapper.GetLandmarkMap();
//        gridMapper.GetOccupancyGridMap(gridMap);
//       // mapper.DrawRobotPoseWithErrorEllipse(robotpath1,img2,true);
//        cout<<"============================="<<endl;
//        ////////////////////////////////////////////////////////////////////////
//        //path planning
//        vector<cv::Point2d> pathPoints;
//        cv::Point2d tempEnd;
//        cv::Mat plannignGridMap;
//        planner.SetGridMap(gridMap);
//        planner.GetPathPlanningMap(plannignGridMap);
//        funcionRobot.FindCurrentNodeEnd(plannignGridMap,50,currentStart,finalEnd,tempEnd);  //30*5=150cm
//        planner.SetStartNode(currentStart);
//        planner.SetEndNode(tempEnd);
//        planner.AStartPlanning(pathPoints);

//        cv::Mat temp1;
//        cv::cvtColor(plannignGridMap,temp1,CV_GRAY2BGR);

//        currentStart.x=robotpath1.robotPositionMean.ptr<double>(0)[0]/100.0*(1.0/0.05)+150;
//        currentStart.y=robotpath1.robotPositionMean.ptr<double>(1)[0]/100.0*(1.0/0.05)+500;
//        cout<<"======================="<<endl;
//        for(int j=0;j!=pathPoints.size();++j)
//        {
//            cv::circle(temp1, cv::Point(pathPoints[j].x,pathPoints[j].y), 4 , cv::Scalar(0,0 ,255), -1  );
//           // cout<<"[path]:"<<pathPoints[j].x<<" "<<pathPoints[j].y<<endl;

//        }
//        cv::line( temp1, cv::Point(tempEnd.x,tempEnd.y), cv::Point(finalEnd.x,finalEnd.y), cv::Scalar(0,0,255), 7,CV_AA);
//        cv::circle(temp1, cv::Point(currentStart.x,currentStart.y), 4, cv::Scalar(0,0,0), 2  );
//        cv::circle(temp1, cv::Point(tempEnd.x,tempEnd.y), 4, cv::Scalar(0,0,255), 2  );
//        cv::circle(temp1, cv::Point(finalEnd.x,finalEnd.y), 4, cv::Scalar(0,0,255), 2  );


//        clock_t endTime=clock();
//       double total=(double)(endTime-startTime)/CLK_TCK;


//        //cout<<"robotPose:"<<robotpath.robotPositionMean<<endl;
//        cv::imshow("imgICP",imgICP);
//        cv::imshow("123123",img2);
//        cv::imshow("1233333",gridMap);
//        cv::imshow("plannignGridMap",plannignGridMap);
//        cv::imshow("plannignGridMapTemp",temp1);
//        cv::imshow("localMap",localMap);
//        QString name="temp/"+QString::number(i)+"plannignGridMap.jpg";
//        cv::imwrite(name.toStdString(),temp1);
//        cv::waitKey(1);

//        cout<<"single step Time:"<<total<<" "<<pathPoints.size()<<endl;

//    }



}

void EIC_Test::on_pushButton_16_clicked()
{
    myekfslam->myrobot->offlineSLAM_filename = ui->lineEdit_SLAM_filename->text().toStdString();
    myekfslam->myrobot->laser_slam->TriggerOneScan();
}

void EIC_Test::on_pushButton_18_clicked()
{
    qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
    connect(myekfslam->myrobot->laser_slam, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
            myekfslam->myrobot, SLOT(SaveOfflineSLAMThread(boost::shared_ptr<QByteArray>)));
    connect(myekfslam->myrobot->laser_slam, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
           myekfslam->myrobot->laser_slam, SLOT(ShowScan(boost::shared_ptr<QByteArray>)));
    // debug message
    connect(myekfslam->myrobot, SIGNAL(OfflineSLAM_ScanSize(int)),
            this, SLOT(Show_OfflineSLAM_ScanSize(int)));
    // odo
    connect(myekfslam->myrobot, SIGNAL(EmitOdoMeter(Odotype, int)),
            myekfslam->myrobot, SLOT(SaveOdoMeter(Odotype, int)));
    // UI
    ui->pushButton_18->setVisible(false);
    ui->pushButton_19->setVisible(true);
}

void EIC_Test::on_pushButton_19_clicked()
{
    qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
    disconnect(myekfslam->myrobot->laser_slam, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
            myekfslam->myrobot, SLOT(SaveOfflineSLAMThread(boost::shared_ptr<QByteArray>)));
    disconnect(myekfslam->myrobot->laser_slam, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
           myekfslam->myrobot->laser_slam, SLOT(ShowScan(boost::shared_ptr<QByteArray>)));
    disconnect(myekfslam->myrobot, SIGNAL(EmitOdoMeter(Odotype, int)),
            myekfslam->myrobot, SLOT(SaveOdoMeter(Odotype, int)));
    ui->pushButton_18->setVisible(true);
    ui->pushButton_19->setVisible(false);
}

void EIC_Test::on_pushButton_17_clicked()
{
    const std::string filename = "./" + ui->lineEdit_SLAM_filename->text().toStdString() + ".txt";
    myekfslam->myrobot->SaveOfflineSLAMFile(filename);
}

void EIC_Test::on_pushButton_20_clicked()
{
//    MyRobot* SLAM_Robot;
//    checkBit=true;
//    motionMode=false;
//    sceneCnt=0;
//    saveFileIndex=0;
//    sceneNum=ui->spinBox_5->value();
//    gridDistance=abs(ui->spinBox_3->value()-ui->spinBox->value())/(sceneNum-1);
//    gridDistance=gridDistance*SLAM_Robot->gridMapper.GetPixel_meterFactor()*100;
//    cout<<"gridDistance:"<<gridDistance<<endl;
//    //intial motor
//    SLAM_Robot->leftMotor->SetHome();
//    SLAM_Robot->rightMotor->SetHome();
//    SLAM_Robot->refenceMap.clear();
//   // SLAM_Robot->lineExtracter.SetDistanceThreshold(20);
//    vector<vector<Line> > lines;


//    robotOutputFile.open("0606_EKF_odoFile.txt");
//    laserOutputFile.open("0606_EKF_laserFile.txt");
//    robotSceneFile.open("0606_EKF_sceneData.txt");

//    if(!robotOutputFile||!laserOutputFile||!robotSceneFile)
//    {
//        QMessageBox::information(this,"Error","Can't open file");
//    }

//    CornerExtraction cornerEx;
//    for(int i=0;i!=4;++i)  //6
//    {
//         vector<double> temp;
//         std::vector<cv::Point2d> temp1;
//         std::vector<Line> temp2;
//         std::vector<Corner> cor;
//         SLAM_Robot->laserS200->TriggerLaser();
//         Sleep(400);
//         SLAM_Robot->laserS200->ReadData(temp);
//         if(temp.size()==0||i==0)
//             continue;
//         SLAM_Robot->mapper.RangesDataToPointsData(temp,temp1);

//         SLAM_Robot->lineExtracter.SplitAndMerge(temp1,temp2);
//         cornerEx.ExtractCorners(temp,cor);

//         cout<<"laser points:"<<temp.size()<<"  line num:"<<temp2.size()<<"  cor num:"<<cor.size()<<endl;
//         cv::Mat img=SLAM_Robot->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

//         cv::Mat img1=SLAM_Robot->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
//         cv::Mat img2=SLAM_Robot->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


//         lines.push_back(temp2);
//         SLAM_Robot->gridMapper.InsertLocalGridMap(temp,cv::Point3d(0,0,0));

//         //////////////////////////////////////////////
//         //save file
//         saveFileIndex++;
//         robotOutputFile<<0<<" "<<0<<endl;  //encoder
//         for(int i=0;i!=temp.size();++i)
//             laserOutputFile<<temp[i]<<" ";

//         laserOutputFile<<endl;//´«¦æ


//         //////////////////////////////////////////////
//         //for(int i=0;i!=temp.size();++i)
//        // {
//           //  if(temp[i]<=2996)
//          //      cout<<temp[i]<<endl;
//         //}

//       //  cv::Mat tt;
//        // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
//        // cv::imshow("tt",tt);
//         cv::imshow("1341",img);
//         cv::imshow("13411",img1);
//         cv::imshow("13411222 ",cornerEx.cornerImg);
//         cv::waitKey(10);
//    }

//    SLAM_Robot->EKFRuner.Initial(lines);
//    cout<<"EKF Intial"<<endl;

//    SLAM_Robot->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
//    SLAM_Robot->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
//    SLAM_Robot->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
//    //set start end points
//    SLAM_Robot->currentStart.x=SLAM_Robot->gridMapper.GetGridMapOriginalPoint().x;
//    SLAM_Robot->currentStart.y=SLAM_Robot->gridMapper.GetGridMapOriginalPoint().y;

//    SLAM_Robot->FinalEnd.x=  SLAM_Robot->gridMapper.GetGridMapOriginalPoint().x+200;
//    SLAM_Robot->FinalEnd.y= SLAM_Robot->gridMapper.GetGridMapOriginalPoint().y;


//    SLAM_Robot->currentEnd.x=  ui->spinBox_3->value();
//    SLAM_Robot->currentEnd.y= ui->spinBox_4->value();

//    testEKFTimer->start(1500);
//    readFlag=true;
//    collector->start();

//    cv::destroyAllWindows();
}

void EIC_Test::on_pushButton_14_clicked()
{

}

void EIC_Test::Show_OfflineSLAM_ScanSize(int size)
{
    ui->textBrowser->append(QString("Scan Size is %1").arg(size));
}

void EIC_Test::on_pushButton_21_clicked()
{
    std::ifstream ofile("./0327SLAM_5.txt");
    int count = 0;
    int temp;
    while(ofile >> temp)
    {
        count++;
    }
    std::cout << "count = " << count << std::endl;
}


void EIC_Test::on_pushButton_22_clicked()
{
//    AStarSearch<CvMatSearchNode> astarsearch;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//     if (pcl::io::loadPCDFile<pcl::PointXYZ> ("1110_2.pcd", *cloud) == -1) //* load the file  1110_2
//     {
//         QMessageBox::information(0, "Error!", "Please Load File Again!");

//     }
//    //«Ç¤º «Ç¥~  (800,800,4,1,3000,(double)CV_PI*(3.0/2.0),0.08);
//     OccupancyGridMapping test(800,800,2,1,8000,CV_PI*2.0,0.05);
//    // OccupancyGridMapping test(800,800,2,1,3000,(double)CV_PI*(3.0/2.0),0.08);
//     cv::Point3d robot(0,0,0);
//     cv::Mat temp;
//     cv::Point2d gridMapOrginal(150.572,500.034);
//     test.SetGridMapOriginalPoint(gridMapOrginal);
//     CvMatSearchNode nodeStart;
//     nodeStart.x =gridMapOrginal.x;
//     nodeStart.y =gridMapOrginal.y;
//     cout<<"start:"<<nodeStart.x<<" "<<nodeStart.y<<" "<<CvMatSearchNode::GetGridMapValue(nodeStart.x,nodeStart.y) <<endl;

//     CvMatSearchNode nodeEnd;
//     nodeEnd.x = 300;   //200
//     nodeEnd.y = 500;  //495
//     cout<<"End:"<<nodeEnd.x<<" "<<nodeEnd.y<<" "<<CvMatSearchNode::GetGridMapValue(nodeEnd.x,nodeEnd.y)<<endl;



//     test.InsertPointCloudGridMap(*cloud,robot);
//     test.GetOccupancyGridMap(temp);

//     cv::imwrite("path.jpg",temp);
//  ////////////
///*
//     PathPlanning planner;
//     planner.SetGridMap(temp);
//     planner.SetStartNode(gridMapOrginal);
//     planner.SetEndNode(cv::Point2d(300,495));
//     vector<cv::Point2d> rr;
//     planner.AStartPlanning(rr);


//     for(int i=0;i!=rr.size();++i)
//        cout<<rr[i]<<endl;


//         cout<<rr.size()<<endl;

//*/




//     int dilation_type = cv::MORPH_RECT;


//     //if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
//     //else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
//     //else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }


//     int dilation_size=4;

//     cv::Mat element = cv::getStructuringElement( dilation_type,
//     cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//     cv::Point( -1, -1 ) );
//        cv::Mat dilation_dst;


//     cv::erode( temp, dilation_dst, element );


//     cv::imshow("dilation_dst",dilation_dst);

//     MyRobot* SLAM_Robot;
//     SLAM_Robot=new MyRobot;

//     SLAM_Robot->FinalEnd.x=nodeEnd.x;
//     SLAM_Robot->FinalEnd.y=nodeEnd.y;

//     cv::Point2d ff;
//     SLAM_Robot->FindCurrentNodeEnd(dilation_dst,128,gridMapOrginal,cv::Point2d(nodeEnd.x,nodeEnd.y),ff);
//     CvMatSearchNode nodeTemp;
//     nodeTemp.x=ff.x;
//     nodeTemp.y=ff.y;
//     cout<<ff.x<<" ! "<<ff.y<<endl;
//     cout<<nodeTemp.x<<" ! "<<nodeTemp.y<<endl;


///*
//     //////////////////////////////////////////////
//     //check goal
//     int box_size=125;
//    // if(nodeEnd.x>=(gridMapOrginal.x+box_size)&&nodeEnd.y>=(gridMapOrginal.y+box_size)||nodeEnd.y<=(gridMapOrginal.y-box_size))

//     cv::Mat img(dilation_dst.size(),dilation_dst.type());

//     CvMatSearchNode nodeTemp;
//     double min=10000;

//     nodeTemp.x=gridMapOrginal.x+10;
//     nodeTemp.y=gridMapOrginal.y;

//     for(int i=0;i!=box_size;++i)
//         for(int j=-box_size;j!=box_size;++j)
//         {
//             int x=i+(int)gridMapOrginal.x;
//             int y=j+(int)gridMapOrginal.y;
//             double temp=dilation_dst.ptr<uchar>(y)[x];
//              if(temp==253)  //1:occ 127:unknow 253:free
//              {

//                double suby=abs((double)nodeEnd.y-y);
//                double subx=abs((double)nodeEnd.x-x);
//                double r=sqrt(pow(subx,2.0)+pow(suby,2.0));

//                if(min>r)
//                {


//                    nodeTemp.x=x;
//                    nodeTemp.y=y;
//                    min=r;
//                }

//              }

//         }


//     cout<<"temp:"<<nodeTemp.x<<" "<<nodeTemp.y<<endl;
//     //box filter
//      cv::imshow("123123123ff",img);
//      cv::waitKey(1);

//*/




//   ///////////////
//     CvMatSearchNode::gridMapN=dilation_dst.clone();

//     astarsearch.SetStartAndGoalStates( nodeStart, nodeTemp );

//     unsigned int SearchState;
//     unsigned int SearchSteps = 0;

//     do
//     {
//         SearchState = astarsearch.SearchStep();

//         SearchSteps++;

//     }
//     while( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_SEARCHING ); //§PÂ_¬O§_°µ§¹¸ô®|³Wµe

//     if( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_SUCCEEDED )
//     {
//         cout << "Search found goal state\n";
//         CvMatSearchNode *node = astarsearch.GetSolutionStart();

//         int steps = 0;

//         node->PrintNodeInfo();
//         //CvMatSearchNode::gridMap.ptr<uchar>(node->y)[node->x]=128;
//         cv::circle(temp, cv::Point(node->x,node->y), 1, cv::Scalar(128,128,128), -1  );
//         for( ;; )
//         {
//                 node = astarsearch.GetSolutionNext();

//                 if( !node )
//                 {
//                         break;
//                 }

//                 //CvMatSearchNode::gridMap.ptr<uchar>(node->y)[node->x]=128;
//                 cv::circle(temp, cv::Point(node->x,node->y), 1, cv::Scalar(128,128,128), -1  );
//                 node->PrintNodeInfo();
//                 steps ++;

//         };
//         cout << "Solution steps " << steps << endl;
//         // Once you're done with the solution you can free the nodes up
//         astarsearch.FreeSolutionNodes();
//     }
//     else if( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_FAILED )
//     {
//             cout << "Search terminated. Did not find goal state\n";

//     }

//     astarsearch.EnsureMemoryFreed();


//     //cv::Point3d robot1(200,0,0);
//     //test.InsertPointCloudGridMap(*cloud,robot1);

//     cv::circle(temp, cv::Point(nodeStart.x,nodeStart.y), 3, cv::Scalar(0,0,0), -1  );
//     cv::circle(temp, cv::Point(nodeEnd.x,nodeEnd.y), 3, cv::Scalar(0,0,0), -1  );
//     cv::circle(temp, cv::Point(nodeTemp.x,nodeTemp.y), 3, cv::Scalar(0,0,0), -1  );


//     cv::imshow("123fffff",temp);
//     cv::waitKey(0);
//     pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//     viewer.showCloud (cloud);
//     while (!viewer.wasStopped ())
//     {
//     }
}

void EIC_Test::on_pushButton_SLAM_resetcount_clicked()
{
    myekfslam->myrobot->ResetCount();
}

void EIC_Test::on_pushButton_slam_EKFslam_clicked()
{
//    myekfslam->myrobot->laser_slam->DisconnectReadyRead();
//    myekfslam->myrobot->right_dcmotor->SetHome();
//    myekfslam->myrobot->left_dcmotor->SetHome();
//    collector->SetMotorPointer(myekfslam->myrobot->right_dcmotor, myekfslam->myrobot->left_dcmotor);
//    collector->SetLaserPointer(myekfslam->myrobot->laser_slam);
//    connect(collector, SIGNAL(GetRightPoseSignal()), myekfslam->myrobot->right_dcmotor, SLOT(GetPose()));
//    connect(collector, SIGNAL(GetLeftPoseSignal()), myekfslam->myrobot->left_dcmotor, SLOT(GetPose()));
//    connect(collector, SIGNAL(TriggerLaserSignal()), myekfslam->myrobot->laser_slam, SLOT(TriggerLaser()));
//    // shih's EKF slam
//    // EKFtimer flag
//    checkBit = true;
//    motionMode = false;
//    sceneCnt = 0;
//    saveFileIndex = 0;
//    // number of scenes
//    sceneNum = ui->spinBox_slam_sceneNum->value();
//    gridDistance = abs(ui->spinBox_slam_x->value() - ui->spinBox_slam_x0->value())/(sceneNum - 1);
//    gridDistance = gridDistance*myekfslam->gridMapper.GetPixel_meterFactor()*100;
//    std::cout << "gridDistance:" << gridDistance << std::endl;

//    // intial motor
//    myekfslam->myrobot->left_dcmotor->SetHome();
//    myekfslam->myrobot->right_dcmotor->SetHome();
//    myekfslam->refenceMap.clear();
//    // SLAM_Robot->lineExtracter.SetDistanceThreshold(20);
//    vector<vector<Line> > lines;


//    robotOutputFile.open("20150503_EKF_odoFile.txt");
//    laserOutputFile.open("20150503_EKF_laserFile.txt");
//    robotSceneFile.open("20150503_EKF_sceneData.txt");

//    if(!robotOutputFile.is_open()||!laserOutputFile.is_open()||!robotSceneFile.is_open())
//    {
//        QMessageBox::information(this,"Error","Can't open file");
//    }

////    qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
////    connect(myrobot->laser_slam, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
////            myrobot, SLOT(ProcessSingleTriggerSLAM(boost::shared_ptr<QByteArray>)));


//    CornerExtraction cornerEx;
//    // repeat features
//    for(int i = 0; i != 4; ++i)  // 6
//    {
//        vector<double> temp;
//        std::vector<cv::Point2d> temp1;
//        std::vector<Line> temp2;
//        std::vector<Corner> cor;
//        // triiger one scan

//        myekfslam->myrobot->laser_slam->TriggerLaser();
//        while(myekfslam->myrobot->laser_slam->waitForReadyRead(400))
//        {
//        }

//        myekfslam->myrobot->laser_slam->ReadData(temp);
////        std::cout << "temp size = " << temp.size() << std::endl;
////        for (int j = 0; j < temp.size(); j++)
////        {
////            std::cout << temp.at(j) << " ";
////        }
////        std::cout <<std::endl;

////        SLAM_Robot->laserS200->ReadData(temp);
//        if(temp.size()==0||i==0)
//            continue;
//        myekfslam->mapper.RangesDataToPointsData(temp, temp1);

//        myekfslam->lineExtracter.SplitAndMerge(temp1, temp2);
//        cornerEx.ExtractCorners(temp,cor);

//        std::cout << "laser points:" << temp.size() << "  line num:" << temp2.size() << "  cor num:" << cor.size() << std::endl;
//        cv::Mat img = myekfslam->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

//        cv::Mat img1 = myekfslam->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
//        cv::Mat img2 = myekfslam->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


//        lines.push_back(temp2);
//        myekfslam->gridMapper.InsertLocalGridMap(temp, cv::Point3d(0,0,0));

//        //////////////////////////////////////////////
//        //save file
//        saveFileIndex++;
//        robotOutputFile << 0 << " " << 0 << endl;  //encoder
//        for(int i = 0; i != temp.size(); ++i)
//            laserOutputFile << temp[i] << " ";

//        laserOutputFile << endl;


//        //////////////////////////////////////////////
//        //for(int i=0;i!=temp.size();++i)
//        // {
//        //  if(temp[i]<=2996)
//        //      cout<<temp[i]<<endl;
//        //}

//        //  cv::Mat tt;
//        // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
//        // cv::imshow("tt",tt);
//        cv::imshow("1341",img);
//        cv::imshow("13411",img1);
//        cv::imshow("13411222 ",cornerEx.cornerImg);
//        while(cv::waitKey(10) == 27)
//        {

//        }
//    }

//    myekfslam->MapInitial(lines);
//    std::cout << "EKF Intial" <<endl;
//    myekfslam->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
//    myekfslam->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
//    myekfslam->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
//    //set start end points
//    myekfslam->currentStart.x = myekfslam->gridMapper.GetGridMapOriginalPoint().x;
//    myekfslam->currentStart.y = myekfslam->gridMapper.GetGridMapOriginalPoint().y;

//    myekfslam->FinalEnd.x = myekfslam->gridMapper.GetGridMapOriginalPoint().x + 200;
//    myekfslam->FinalEnd.y = myekfslam->gridMapper.GetGridMapOriginalPoint().y;


//    myekfslam->currentEnd.x = ui->spinBox_slam_x->value();
//    myekfslam->currentEnd.y = ui->spinBox_slam_y->value();

//    testEKFTimer->start(1500);
//    readFlag = true;
//    collector->start();

//    cv::destroyAllWindows();

}

void EIC_Test::EKF_Timer()
{
//    if(checkBit != true)
//    {

//        myekfslam->myrobot->right_dcmotor->Stop();
//        myekfslam->myrobot->left_dcmotor->Stop();
//        return;

//    }

//    clock_t startTime=clock();
//    //img=cv::Scalar::all(0);

//    saveFileIndex++;
//    readFlag=true;
//    double DeltaRight=((double)((odoValueCurrent.x-odoValuePrevious.x)*32.5*CV_PI)/(4096*14.0*3.33333));
//    double DeltaLeft=((double)((odoValueCurrent.y-odoValuePrevious.y)*32.5*CV_PI)/(4096*14.0*3.33333));



//    ui->textBrowser->setFontWeight( QFont::Normal );
//    ui->textBrowser->setTextColor( QColor( "blue" ) );
//    ui->textBrowser->append("[encoder]:("+QString::number(DeltaRight)+","+QString::number(DeltaLeft)+")");
//    cout<<"3:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<odoValuePrevious.x<<" , "<<odoValuePrevious.y<<endl;
//    cout<<"3:"<<DeltaRight<<" "<<DeltaLeft<<" time:"<<endl;

//    odoValuePrevious.x=odoValueCurrent.x;
//    odoValuePrevious.y=odoValueCurrent.y;

//    std::vector<cv::Point2d> points;
//    std::vector<Line> line;
//    std::vector<Corner> cor;

//    ////////////////////////////////////////////////////////////////////////
//    //feature extraction
//    myekfslam->mapper.RangesDataToPointsData(myekfslam->rawLaserScanData,points);
//    myekfslam->lineExtracter.SplitAndMerge(points,line);

//    ////////////////////////////////////////////////////////////////////////
//    //motion estimation
//    //cout<<DeltaRight<<" "<<odoValueCurrent.x<<" "<<odoValuePrevious.x<<endl;DeltaLeft
//    myekfslam->EKFRuner.MotionPrediction(cv::Point2d(DeltaLeft,DeltaRight));

//    //SLAM_Robot->gridMapper.InsertLocalGridMap();

//    myekfslam->EKFRuner.DataAssociation(line,cor);
//    myekfslam->robotPosition=myekfslam->EKFRuner.GetRobotPose();



//    // QMessageBox::information(this, "Error!", "ok1!");

//    ////////////////////////////////////////////////////////////////////////
//    //ICP correction
//    vector<cv::Point2d> temp;
//    dataConvertRobotToWorld(points,myekfslam->robotPosition,temp);

//    double percent=0.5;
//    //cv::Point3d adjustPose=SLAM_Robot->icper.Align(SLAM_Robot->refenceMap,temp,percent);
//    cv::Point3d adjustPose;
//    adjustPose.x=0;
//    adjustPose.y=0;
//    adjustPose.z=0;
//    ////////////////////////////////////////////////////////////////////////
//    //­×¥¿«áªº¾÷¾¹¤H¦ì¸m

//    RobotState robotpath1;

//    robotpath1.robotPositionMean.ptr<double>(0)[0]=adjustPose.x+myekfslam->robotPosition.robotPositionMean.ptr<double>(0)[0];
//    robotpath1.robotPositionMean.ptr<double>(1)[0]=adjustPose.y+myekfslam->robotPosition.robotPositionMean.ptr<double>(1)[0];
//    robotpath1.robotPositionMean.ptr<double>(2)[0]=adjustPose.z+myekfslam->robotPosition.robotPositionMean.ptr<double>(2)[0];
//    robotpath1.robotPositionCovariance=myekfslam->robotPosition.robotPositionCovariance.clone();

//    //xyz¹ê»Ú§ë¼v¦Ü¥­­±ªü
//    cv::Point2d imagePose=myekfslam->gridMapper.GetRobotCenter(robotpath1);

//    myekfslam->currentStart.x=imagePose.x;
//    myekfslam->currentStart.y=imagePose.y;

//    myekfslam->EKFRuner.SetRobotPose(robotpath1);
//    ///////////////////////////////////// ///////////////////////////////////

//    //add new map
//    for(int k=0;k!=points.size();++k)
//    {

//        double x=cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x-sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(0)[0];
//        double y=sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x+cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(1)[0];
//        //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
//        myekfslam->refenceMap.push_back(cv::Point2d(x,y));
//        //  cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

//    }

//    ////////////////////////////////////////////////////////////////////////
//    //mapping process

//    myekfslam->mapper.InsertLocalLandmarkMap(points,robotpath1);
//    landMarkImg = myekfslam->mapper.GetLandmarkMap();
//    myekfslam->gridMapper.InsertLocalGridMap(myekfslam->rawLaserScanData,robotpath1);
//    myekfslam->gridMapper.GetOccupancyGridMap(gridImg);

//    myekfslam->mapper.DrawRobotPoseWithErrorEllipse(robotpath1,landMarkImg,true);


//    ////////////////////////////////////////////////////////////////////////
//    //path planning
//    cv::Point2d tempEnd;
//    cv::Mat plannignGridMap;
//    double search_rect=80;
//    myekfslam->planner.SetGridMap(gridImg);
//    myekfslam->planner.GetPathPlanningMap(plannignGridMap);
//    cv::circle(plannignGridMap,myekfslam->currentStart, 2, cv::Scalar(255,255,255), -1 );
//    myekfslam->FindCurrentNodeEnd(plannignGridMap,search_rect,myekfslam->currentStart,myekfslam->currentEnd,tempEnd);


//    double rsrart=sqrt(pow(myekfslam->currentStart.x,2.0) +pow(myekfslam->currentStart.y,2.0)  );
//    double rend=sqrt(pow(myekfslam->currentEnd.x,2.0) +pow(myekfslam->currentEnd.y,2.0)  );
//    double rFinal=sqrt(pow(myekfslam->FinalEnd.x,2.0) +pow(myekfslam->FinalEnd.y,2.0)  );
//    double rTempEnd=sqrt(pow(tempEnd.x,2.0) +pow(tempEnd.y,2.0)  );

//    cv::Mat color_plannignGridMap;
//    cv::cvtColor(plannignGridMap,color_plannignGridMap,CV_GRAY2BGR);


//    //////////////////////////////////////////////
//    //save file
//    robotOutputFile<<DeltaRight<<" "<<DeltaLeft<<endl;
//    for(int i=0;i!=myekfslam->rawLaserScanData.size();++i)
//        laserOutputFile<<myekfslam->rawLaserScanData[i]<<" ";

//    laserOutputFile<<endl;//´«¦æ


//    //////////////////////////////////////////////

//    if(sceneCnt==sceneNum)  //»`¶°¨ì³õ´º¼Æ
//    {
//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);
//        testEKFTimer->stop();
//        scenesTimer->stop();
//        ui->textBrowser->setFontWeight( QFont::Normal );
//        ui->textBrowser->setTextColor( QColor( "blue" ) );
//        ui->textBrowser->append("[System Message]:Motor stop!");
//        return;
//    }

//    if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=gridDistance*sceneCnt)
//        //if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=sceneCnt*5)
//    {

//        checkBit=false;

//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);

//        motionMode=true;
//        ui->pushButton_2->click();

//        testEKFTimer->stop();
//        sceneCnt++;
//        //QMessageBox::information(this,"dd","ff");
//        cout<<"scen:"<<sceneCnt<<"  robotpose:"<<robotpath1.robotPositionMean<<"   index:"<<saveFileIndex<<endl;
//        robotSceneFile<<sceneCnt<<" "<<robotpath1.robotPositionMean.ptr<double>(0)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(1)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(2)[0]<<" "<<saveFileIndex<<endl;
//        return;
//    }



//    //if( (rend<=rsrart))  //¥b®|¤j©ó©Îµ¥©ó2*pixelFactor;

//    if( rsrart <= rTempEnd && threcont - ui->doubleSpinBox->value() >= myekfslam->commandSets.size())
//    {
//        myekfslam->commandSets = std::queue<std::pair<int,double>> ();
//        myekfslam->planner.SetStartNode(myekfslam->currentStart);

//        do
//        {

//            myekfslam->planner.SetEndNode(tempEnd);
//            myekfslam->planner.AStartPlanning(myekfslam->robotPathSets);
//            //  QMessageBox::information(this, "Error!", "2_2!!"+QString::number(SLAM_Robot->robotPathSets.size()));
//            //  cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

//            //  cout<<"tempEnd1:"<<tempEnd<<endl;
//            //  cv::imshow("show",color_plannignGridMap);
//            //  cv::waitKey(1);


//            // QMessageBox::information(this, "Error!", "2_1_1");


//            if(myekfslam->robotPathSets.size()!=0)
//            {
//                myekfslam->PathSmoothing();

//                myekfslam->TrajectoryGenerationSmoothing();
//                threcont=myekfslam->commandSets.size();


//            }
//            else
//            {


//                tempEnd.x=myekfslam->currentStart.x+10;
//                tempEnd.y=myekfslam->currentStart.y;


//            }
//            //cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,0,255), 2  );
//            // cout<<"tempEnd2:"<<tempEnd<<endl;

//            //  cv::imshow("show",color_plannignGridMap);
//            //  cv::waitKey(1);
//            // QMessageBox::information(this, "Error!", "2_1_2");


//        }while(myekfslam->robotPathSets.size()==0);

//        this->SetMotionCommand();

//        /*
//        if(SLAM_Robot->robotPathSets.size()!=0)
//        {
//            SLAM_Robot->PathSmoothing();

//            SLAM_Robot->TrajectoryGenerationSmoothing();
//            threcont=SLAM_Robot->commandSets.size();

//            this->SetMotionCommand();
//        }
//*/
//        ui->textBrowser->append("=====================");
//        ui->textBrowser->append("[robotPathSets size]:"+QString::number(myekfslam->robotPathSets.size()));
//        for(int i=0;i!=myekfslam->robotPathSets.size();++i)
//        {

//            cv::circle(color_plannignGridMap, cv::Point(myekfslam->robotPathSets[i].x,myekfslam->robotPathSets[i].y), 4 , cv::Scalar(0,0 ,255), -1  );
//            ui->textBrowser->setFontWeight( QFont::DemiBold );
//            ui->textBrowser->setTextColor( QColor( "red" ) );
//            ui->textBrowser->append("[robotPathSets]:"+QString::number(myekfslam->robotPathSets[i].x)+" "+QString::number(myekfslam->robotPathSets[i].y));

//        }
//        ui->textBrowser->append("=====================");
//    }






//    // else
//    // ui->textBrowser->append("=====no path fuck========");


//    cv::line( color_plannignGridMap, tempEnd, myekfslam->currentEnd, cv::Scalar(0,0,255), 7,CV_AA);

//    cv::circle(color_plannignGridMap, myekfslam->currentStart, 4, cv::Scalar(0,255,0), 2  );
//    cv::circle(color_plannignGridMap, myekfslam->currentEnd, 4, cv::Scalar(0,255,0), 2  );
//    cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

//    clock_t endTime=clock();
//    double total=(double)(endTime-startTime)/CLK_TCK;

//    ui->textBrowser_slam->setFontWeight( QFont::DemiBold );
//    ui->textBrowser_slam->setTextColor( QColor( "red" ) );
//    ui->textBrowser_slam->append("[pose]:" + QString::number(robotpath1.robotPositionMean.ptr<double>(0)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(1)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(2)[0]));

//    std::cout << "[Robot pose]:" << robotpath1.robotPositionMean << std::endl;

//    std::cout << "single step Time:" << total << std::endl;
//    cv::imshow("gridImg",gridImg);
//    cv::imshow("landMarkImg",landMarkImg);
//    cv::imshow("color_plannignGridMap",color_plannignGridMap);
//    cv::imshow("plannignGridMap",plannignGridMap);
//    static int count=0;
//    QString name="Img/"+QString::number(count)+".jpg";
//    cv::imwrite(name.toStdString(),color_plannignGridMap);
//    cv::imwrite("Img/plannignGridMap.jpg",plannignGridMap);
//    cv::imwrite("Img/gridImg.jpg",gridImg);
//    cv::imwrite("Img/landMarkImg.jpg",landMarkImg);
//    count++;

//    cv::waitKey(1);




//    //if(motionMode==true)

}

void EIC_Test::singleSceneAcquisition()
{


    if(checkBit!=true)
    {

        myekfslam->myrobot->right_dcmotor->Stop();
        myekfslam->myrobot->left_dcmotor->Stop();


    }





    clock_t startTime=clock();




//    if(paused && SetupFormer.openDetectPeople!=true)  //avoid people
//    {
//        cv::waitKey(100);
//        return;
//    }


    cv::Mat frame,showImg,frameTemp;
    vector<cv::Rect> found, found_filtered; //°»´ú¦æ¤H®Ø®Ø
    found.reserve(100);
    found_filtered.reserve(100);

//    if(SLAM_Robot->webcam->isOpened()&&SetupFormer.renderColor)
//    {
//        cv::Mat iplImg;
//        *(SLAM_Robot->webcam)>> iplImg;  //cameraÅª¹Ï
//        if(iplImg.empty())
//            return;

//        cv::undistort( iplImg, frame, SLAM_Robot->cameraIntrinsicMatrix, SLAM_Robot->cameradistortionCoefficients, SLAM_Robot->cameraIntrinsicMatrix );  //±N¹Ï¤ù¶i¦æ­×¥¿
//        //°µ§á¦±®Õ¥¿
//        QString imgName=windowName+"/"+QString::number(laserCnt)+".jpg";
//        cv::imwrite(imgName.toStdString(),frame);


//        ///////////////////////////////
//        if(SetupFormer.openDetectPeople==true)
//        {


//            int len = std::max(frame.cols, frame.rows);
//            //cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f((double)len/2,(double)len/2), 90, 1);
//            cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f((double)len/2,(double)len/2), 90, 1);

//            cv::warpAffine(frame, frameTemp, rot_matrix, cv::Size(len,len));

//            cv::Rect roi;
//            roi.x=0;
//            roi.y=0;
//            roi.width=480;
//            roi.height=640;

//            frameTemp=frameTemp(roi);

//            //hog.detectMultiScale(frameTemp, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
//            hog.detectMultiScale(frameTemp, found, 0, cv::Size(8,8), cv::Size(24,16), 1.2, 2);
//            size_t i, j;
//            for( i = 0; i < found.size(); i++ )
//            {
//                cv::Rect r = found[i];
//                for( j = 0; j < found.size(); j++ )
//                    if( j != i && (r & found[j]) == r)
//                        break;
//                if( j == found.size() )
//                    found_filtered.push_back(r);
//            }




//            for( int i = 0; i < found_filtered.size(); i++ )
//            {
//                cv::Rect r = found_filtered[i];
//                // the HOG detector returns slightly larger rectangles than the real objects.
//                // so we slightly shrink the rectangles to get a nicer output.


//                r.x += cvRound(r.width*0.1);
//                r.width = cvRound(r.width*0.8);
//                r.y += cvRound(r.height*0.07);
//                r.height = cvRound(r.height*0.8);


//                rectangle(frameTemp, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
//            }

//            // cout<<   found.size()<<endl;
//            cv::imshow("123",frameTemp);
//        }

//        // cout<<found_filtered.size()<<endl;



//        /*
//       if(found.size()>0)
//           paused=true;
//       else
//           paused=false;



//       */

//    }





//    showImg=frame.clone();

//    vector<double> temp;
//    temp.reserve(361);
//    //read data
//    SLAM_Robot->laser291->TriggerLaser();
//    double angle=((double)laserCnt*3000)/SLAM_Robot->GetAllSteps()*60.0 ;

//    SLAM_Robot->stepMotor->SetAbsoluteAngle(angle);
//    Sleep(350);

//    SLAM_Robot->laserLms291->ReadData(temp);


//    double theta=((double)laserCnt)*2*CV_PI/SLAM_Robot->GetAllSteps() ;

//    //TEST
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempT=pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

//    double mean=0;
//    int count=0;
//    for(int i=0;i!=temp.size();++i)
//    {

//        pcl::PointXYZ rawPoint;  //gray  temp
//        pcl::PointXYZRGB rawPointwithColor;  // color temp
//        cv::Mat laserPoint=cv::Mat::zeros(4,1,CV_64F);
//        cv::Mat I=cv::Mat::eye(3,4,CV_64F);
//        double phi=(double)(i*CV_PI)/temp.size();

//        double xImg=temp[i]*cos(phi);
//        double yImg=temp[i]*sin(phi);

//        //transform to 3D
//        rawPoint.x=rawPointwithColor.x=yImg*cos(theta);
//        rawPoint.y=rawPointwithColor.y=yImg*sin(theta);
//        rawPoint.z=rawPointwithColor.z=xImg;
//        SLAM_Robot->singleScene->points.push_back(rawPoint);


//        if(SetupFormer.renderColor&&SLAM_Robot->webcam->isOpened())
//        {
//            laserPoint.ptr<double>(0)[0]=xImg;
//            laserPoint.ptr<double>(1)[0]=yImg;
//            laserPoint.ptr<double>(2)[0]=0;
//            laserPoint.ptr<double>(3)[0]=1;

//            cv::Mat imgPoint=SLAM_Robot->cameraIntrinsicMatrix*I*(SLAM_Robot->laser_cameraHomogeneous)*laserPoint;

//            double u=imgPoint.ptr<double>(0)[0]/imgPoint.ptr<double>(2)[0];
//            double v=imgPoint.ptr<double>(1)[0]/imgPoint.ptr<double>(2)[0];

//            cv::circle(showImg, cv::Point(u,v), 1, cv::Scalar(0,0,255), -1  );


//            if(u>=0&&v>=0&&u<=frame.cols&&v<=frame.rows)
//            {
//                double b=(double)frame.at<cv::Vec3b>(v,u)[0];
//                double g=(double)frame.at<cv::Vec3b>(v,u)[1];
//                double r=(double)frame.at<cv::Vec3b>(v,u)[2];

//                mean+=v;
//                count++;
//                rawPointwithColor.r=r;  rawPointwithColor.g=g;  rawPointwithColor.b=b;

//            }
//            else
//            {

//                rawPointwithColor.r=0;  rawPointwithColor.g=0;  rawPointwithColor.b=0;

//            }


//            tempT->points.push_back(rawPointwithColor);


//        }

//    }


//    //push single scan
//    SLAM_Robot->rawScanSets.push_back(temp);

//    if(SetupFormer.renderColor&&SLAM_Robot->webcam->isOpened())
//    {

//        //cv::imshow("Camera_Laser_fusion",showImg);

//        mean/=count;

//        bool flag=false;

//        if(SetupFormer.openDetectPeople==true)
//        {

//            for( int z = 0; z < found_filtered.size(); z++ )
//            {
//                cv::Rect r = found_filtered[z],t;
//                // the HOG detector returns slightly larger rectangles than the real objects.
//                // so we slightly shrink the rectangles to get a nicer output.
//                t.y=r.x;
//                t.x=frameTemp.rows-r.y-r.height;
//                t.height=r.width;
//                t.width=r.height;



//                // cout<<"ff"<<(double)t.height/t.width<<endl;

//                if(abs(mean-(double)(t.y+t.y+t.height)/2.0)<50/*&&((double)t.height/t.width)<0.45*/)
//                {

//                    cv::Rect c = t;
//                    c.x += cvRound(c.width*0.1);
//                    c.width = cvRound(c.width*0.8);
//                    c.y += cvRound(r.height*0.07);
//                    c.height = cvRound(c.height*0.8);
//                    rectangle(showImg, c.tl(), c.br(), cv::Scalar(0,0,255), 3);

//                    //rectangle(showImg, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
//                    //rectangle(showImg, t.tl(), t.br(), cv::Scalar(0,0,255), 3);
//                    //cout<< t.y <<" "<<t.height<<" "<<mean<<" "<<(double)(t.y+t.y+t.height)/2.0<<" "<<(double)(t.height)/t.width<<endl;
//                    flag=true;


//                    ShowImage(ui->label,showImg);


//                    ui->textBrowser->append("[System Message]: STOP");
//                }



//                //cout<< t.y <<" "<<t.height<<" "<<mean<<" "<<(double)(t.y+t.y+t.height)/2.0<<" "<<(double)(t.height)/t.width<<endl;


//            }

//            if(flag==true)
//            {
//                paused=true;
//                // escape=0;

//            }
//            else
//                paused=false;


//        }

//        cv::Mat rot_matrix = cv::getRotationMatrix2D(cv::Point2f((double)showImg.cols/2,(double)showImg.rows/2), 90, 1);
//        cv::warpAffine(showImg, showImg, rot_matrix, showImg.size());

//        ShowImage(ui->label,showImg);
//        cv::imshow("Img11",showImg.clone());
//        cv::waitKey(1);


//        if(flag==false)
//            *SLAM_Robot->singleColorScene+=*tempT;


//    }

//    if(paused&&SetupFormer.openDetectPeople==true)  //avoid people
//    {



//        if( escape>=5) //±Mªùµ¹¦Û°Ê¤Æ
//        {
//            paused=false;
//            escape=0;
//            cout<< "escape"  <<endl;
//        }

//        cv::waitKey(100);
//        escape++;

//        return;
//    }
//    //³æ²´¬Û¾÷
//    if(digCamCnt*SLAM_Robot->GetShootStep()<=laserCnt&&SLAM_Robot->digitalCam.getState()==CAMERA_OPEN)
//    {
//        ui->textBrowser->setFontWeight( QFont::Normal );
//        ui->textBrowser->setTextColor( QColor( "black" ) );
//        bool shootPicture=SLAM_Robot->digitalCam.takePicture();
//        if(shootPicture)
//        {
//            ui->textBrowser->append("[System Message]: Image"+QString::number(digCamCnt+1)+" has been taken");
//            ui->statusBar->showMessage("Image"+QString::number(digCamCnt+1)+" has been taken");
//            digCamCnt++;
//        }
//    }


//    //show point cloud
//    if(SetupFormer.showScanning)
//    {
//        if(SetupFormer.renderColor)
//            viewer->showCloud(SLAM_Robot->singleColorScene);
//        else
//            viewer->showCloud (SLAM_Robot->singleScene);
//    }

//    //laser trigger message
//    int percent=((double)laserCnt/SLAM_Robot->GetAllSteps())*100;
//    ui->progressBar_2->setValue(percent+1);
//    qApp->processEvents(QEventLoop::ExcludeUserInputEvents);
//    ui->progressBar_2->repaint();
//    ui->textBrowser->setFontWeight( QFont::Normal );
//    ui->textBrowser->setTextColor( QColor( "black" ) );
//    ui->textBrowser->append("[System Message]: Laser trigger:"+QString::number(laserCnt));


//    //save file
//    if(laserCnt==(SLAM_Robot->GetAllSteps())-1||saved==true)
//    {

//        time_t now = time(0); // get current time
//        struct tm* tm = localtime(&now); // get struct filled out

//        QString saveFileName;
//        if(ui->lineEdit->text()=="NULL")
//            saveFileName="0"+QString::number(tm->tm_mon+1)+QString::number(tm->tm_mday)+"_"+QString::number(SLAM_Robot->GetSceneNum());
//        else
//            saveFileName=ui->lineEdit->text();

//        //reset motor position
//        double angle=((double)laserCnt/SLAM_Robot->GetAllSteps())*60.0* 3000.0;
//        SLAM_Robot->stepMotor->SetRelatedAngle(-angle);  //reset position
//        scenesTimer->stop();

//        ui->textBrowser->setFontWeight( QFont::DemiBold );
//        ui->textBrowser->setTextColor( QColor( "red" ) );
//        ui->textBrowser->append("[System Message]: Saved "+saveFileName+" contains "+QString::number(SLAM_Robot->singleScene->points.size ())+" points");


//        ui->textBrowser->append("[System Message]: scenes num:"+QString::number(SLAM_Robot->GetSceneNum()));

//        SLAM_Robot->SaveRawScanSets(QString(saveFileName+".txt").toStdString());

//        //gray point cloud saved
//        SLAM_Robot->singleScene->height = SLAM_Robot->singleScene->points.size ();
//        SLAM_Robot->singleScene->width = 1;
//        pcl::io::savePCDFile(QString(saveFileName+".pcd").toStdString(),*SLAM_Robot->singleScene);



//        if(SLAM_Robot->singleColorScene->points.size ()!=0)
//        {
//            SLAM_Robot->singleColorScene->height = SLAM_Robot->singleColorScene->points.size ();
//            SLAM_Robot->singleColorScene->width = 1;
//            pcl::io::savePCDFile(QString(saveFileName+"color.pcd").toStdString(),*SLAM_Robot->singleColorScene);



//        }

//        delete viewer;  //destroy window
//        SLAM_Robot->AddScenesNum();
//        SLAM_Robot->AddScenes();


//        //SLAM_Robot->gridMapper.InsertPointCloudGridMap
//        ////////////////////////////////////////////////////////////
//        //grid map
//        ////////////////////////////////////////////////////////////

//        if(motionMode==true)
//        {


//            collector->start();
//            Sleep(4500);
//            odoValuePrevious.x=odoValueCurrent.x;
//            odoValuePrevious.y=odoValueCurrent.y;
//            testEKFTimer->start(1500);
//            motionMode=false;
//            checkBit=true;


//        }

//        ui->textBrowser->append("[System Message]: Done!");


//    }

//    laserCnt++;



//    clock_t endTime=clock();
//    double total=(double)(endTime-startTime)/CLK_TCK;

//    // cout<<total<<endl;

}

void EIC_Test::SetMotionCommand()
{
    double value = 0;
    int velocity = myekfslam->GetVelocity();
    //RightMotor,LeftMotor);

    if(checkBit != true)
    {

        myekfslam->myrobot->right_dcmotor->Stop();
        myekfslam->myrobot->left_dcmotor->Stop();
        return;

    }


    if(!myekfslam->commandSets.empty())
    {
        collector->SetCommandFlag(true);
       // cout<<"!!"<<commandSets.front().second<<endl;
        if(myekfslam->commandSets.front().first==1)
        {

            //SLAM_Robot->rightMotor->SetVelocity(0);
            //SLAM_Robot->leftMotor->SetVelocity(0);
          //  commandSets.front().second
            //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:325]
             value=(4096*3.333*14)*(myekfslam->commandSets.front().second/32.5)/(CV_PI);
            cout<<"forward: "<<value<<endl;
            myekfslam->myrobot->left_dcmotor->SetVelocity(velocity);
            myekfslam->myrobot->right_dcmotor->SetVelocity(-velocity);



        }
        else
        {
            double arc=2*CV_PI*myekfslam->GetRobotWidth()*(myekfslam->commandSets.front().second/360.0);
             //double arc=2*CV_PI*SLAM_Robot->GetRobotWidth()*((SLAM_Robot->commandSets.front().second*30.0/45.0)/360.0);
            // cout<<angle<<endl;
            value=(4096*3.333*14)*(arc/32.5)/(CV_PI);

            if(myekfslam->commandSets.front().first==2) //right
            {
                //SLAM_Robot->rightMotor->SetVelocity(-velocity);
                myekfslam->myrobot->right_dcmotor->SetVelocity(-30);
                myekfslam->myrobot->left_dcmotor->SetVelocity(0);

            }
            else if(myekfslam->commandSets.front().first==3) //left
            {
                myekfslam->myrobot->right_dcmotor->SetVelocity(0);
                //SLAM_Robot->leftMotor->SetVelocity(velocity);
                myekfslam->myrobot->left_dcmotor->SetVelocity(30);

            }
            value=abs(value); //test
            cout<<"angle:"<<value<<endl;

        }
        ui->textBrowser_slam_2->setFontWeight( QFont::DemiBold );
        ui->textBrowser_slam_2->setTextColor( QColor( "red" ) );
        ui->textBrowser_slam_2->append("mode:"+QString::number(myekfslam->commandSets.front().first)+ " value:"+QString::number(myekfslam->commandSets.front().second));


        myekfslam->commandSets.front().second=value;
        collector->SetMotionCommand(myekfslam->commandSets.front().first,myekfslam->commandSets.front().second);
        myekfslam->commandSets.pop();

    }
    else
    {
        collector->SetCommandFlag(false);
        myekfslam->myrobot->right_dcmotor->Stop();
        myekfslam->myrobot->left_dcmotor->Stop();
    }

}

void EIC_Test::setOdometryLaserData(const int right, const int left, const std::vector<double> &data)
{
    /*
    clock_t startTime=clock();

    SLAM_Robot->rawLaserScanData=data;

    if(readFlag==true)
    {

        odoValueCurrent.x=-right;  //�ƭȬ��t��
        odoValueCurrent.y=left;

        //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:32.5]

        //cout<<DeltaRight<<" "<<DeltaLeft<<" "<<data.size()<<endl;

        clock_t endTime=clock();
        double total=(double)(endTime-startTime)/CLK_TCK;

        cout<<"2:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<total<<endl;
        readFlag=false;


    }


    */

    clock_t startTime=clock();

    myekfslam->rawLaserScanData=data;

    //  if(readFlag==true)
    //{


    odoValueCurrent.x=-right;  //�ƭȬ��t��
    odoValueCurrent.y=left;

    //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:32.5]

    //cout<<DeltaRight<<" "<<DeltaLeft<<" "<<data.size()<<endl;

    clock_t endTime=clock();
    double total=(double)(endTime-startTime)/CLK_TCK;

    cout<<"2:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<total<<endl;
    readFlag=false;


    //  }
}

void EIC_Test::on_pushButton_23_clicked()
{
//    connect(this, SIGNAL(TestFor(double, double)), myekfslam->myrobot, SLOT(GoForward(double,double)));
//    connect(myekfslam->EKFTimer, SIGNAL(timeout()), myekfslam, SLOT(EKFStepExamine()));
//    connect(myekfslam, SIGNAL(MotorStop()), myekfslam->myrobot, SLOT(MotorStop()));
//    myekfslam->myrobot->right_dcmotor->Disconnect();
//    myekfslam->myrobot->left_dcmotor->Disconnect();
    // turn off all answers, except for position response
//    myekfslam->myrobot->left_dcmotor->SetANSWmode(0);
//    myekfslam->myrobot->left_dcmotor->SetHome();
////    myekfslam->myrobot->right_dcmotor->SetANSWmode(2);
////    myekfslam->myrobot->left_dcmotor->SetNPmode();
////    myekfslam->myrobot->right_dcmotor->SetNPmode();
//    myekfslam->Connect();
////    myekfslam->myrobot->GoForward(0.1, 400);
//    emit TestFor(0.1, 400);
//    myekfslam->EKFTimer->start(1000);

    myekfslam->Initial(ui->spinBox_slam_sceneNum->value(),
                       ui->spinBox_slam_x0->value(),
                       ui->spinBox_slam_x->value(),
                       ui->spinBox_slam_y->value(),
                       ui->doubleSpinBox->value());
}

void EIC_Test::on_pushButton_24_clicked()
{
    PathPlanning pathplan;
    ////////////////////////////////////////////////////////////////////////
    //path planning
    cv::Mat gridImg = cv::Mat(800, 800, CV_8UC1, cv::Scalar(255));
    cv::rectangle(gridImg, cv::Point(200, 200), cv::Point(600, 600), cv::Scalar(0));
    cv::imshow("show", gridImg);
    cv::waitKey(1);
//    cv::Point2d tempEnd;
    cv::Mat plannignGridMap;
//    double search_rect=80;
    pathplan.SetGridMap(gridImg);
    cv::Point2d currentStart = cv::Point2d(100, 100);
    cv::Point2d currentEnd = cv::Point2d(700, 700);
    pathplan.SetStartNode(currentStart);
    pathplan.SetEndNode(currentEnd);

    std::vector<cv::Point2d> pathSets;
    pathplan.AStartPlanning(pathSets);
    std::cout << "pathSets.size() = " << pathSets.size() << std::endl;

    pathplan.GetPathPlanningMap(plannignGridMap);

    cv::imshow("plan", plannignGridMap);
    cv::waitKey(1);
//    cv::circle(plannignGridMap, currentStart, 2, cv::Scalar(255,255,255), -1 );
//    FindCurrentNodeEnd(plannignGridMap, search_rect, currentStart, currentEnd, tempEnd);
}
