#ifndef EIC_TEST_H
#define EIC_TEST_H

#include <QMainWindow>
#include "stepmotor_controller.h"     // lab405 step motor controller
#include "laser_lms291_controller.h"
#include "myrobot.h"
#include <QTimer>
#include <QMessageBox>
#include "sensorsetup.h"                // sensor UI, class SensorSetup
// test
#include <QSerialPort>
#include "iostream"             // for std::cout
#include <fstream>              // for std::ofstream

// temp
#include <vector>
//#include "shih_slam/mytoolkit.h"
//#include "shih_slam/landmarkmapping.h"
//#include "shih_slam/lineextraction.h"
//#include "shih_slam/cornerextraction.h"
//#include "shih_slam/occupancygridmapping.h"
//#include "shih_slam/ekf_slam.h"
//#include "shih_slam/pathplanning.h"
//#include "shih_slam/shih_myrobot.h"
#include "shih_slam/shih_myrobot.h"
#include "shih_slam/datagrabthread.h"

namespace Ui {
class EIC_Test;
}

class EIC_Test : public QMainWindow
{
    Q_OBJECT

public:
    explicit EIC_Test(QWidget *parent = 0);
    ~EIC_Test();

private slots:
    void on_pushButton_step_motor_clicked();

    void on_pushButton_step_com_clicked();

    void on_pushButton_relangle_clicked();

    void on_pushButton_laser291_open_clicked();

    void on_pushButton_laser291_trigger_clicked();

    void StartToGetData();

    void on_pushButton_laser291_stop_clicked();

    void on_pushButton_step_sethome_clicked();

    void on_pushButton_step_gohome_clicked();

    void on_pushButton_step_stop_clicked();

    void on_pushButton_robot_data_acquisition_clicked();

    void on_pushButton_step_absangle_clicked();

    void on_pushButton_clicked();

    void on_pushButton_step_absangle_destroyed();

    void on_pushButton_2_clicked();

    void CommandResponse();

    void on_pushButton_robot_stop_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_robot_closeall_clicked();

    // debug slots
    // data acquisition message

    void ShowCameraOpened();

    void ShowFinishedDataAcquisition(int _size);

    void ShowDataSegement(int);

    void ShowPushBuffer(int);

    void ShowFinishOneScan(int, bool laser, bool image);

    void ShowFinishWriteData();

    void on_pushButton_12_clicked();

    void on_pushButton_dcmotor_right_comport_clicked();

    void on_pushButton_dcmotor_left_comport_clicked();

    void on_pushButton_dcmotor_front_clicked();

    void on_pushButton_dcmotor_back_clicked();

    void on_pushButton_dcmotor_left_clicked();

    void on_pushButton_dcmotor_right_clicked();

    void on_pushButton_dcmotor_stop_clicked();


    void on_pushButton_robot_trigger_clicked();

    void on_pushButton_robot_disconnect_clicked();

    void on_pushButton_robot_connect_clicked();

    void on_pushButton_robot_stoptrigger_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_SLAM_clicked();

    void on_pushButton_laser_slam_open_clicked();

    void on_pushButton_SLAM_2_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_18_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_14_clicked();

    // 2015.04.08
    void Show_OfflineSLAM_ScanSize(int size);

    void on_pushButton_21_clicked();


    void on_pushButton_22_clicked();

    void on_pushButton_SLAM_resetcount_clicked();

    void on_pushButton_slam_EKFslam_clicked();

    // shih
    void EKF_Timer();
    void singleSceneAcquisition();
    void SetMotionCommand();

private:
    Ui::EIC_Test *ui;
    lab405::MyRobot *myrobot;

    // additional setup UI
    SensorSetup *sensorsetup;
    QMenu *fileMenu;
    QMenu *sensorMenu;
    QAction *setupAct;

    void CreateAction();

    void CreateMenus();

    QTimer *timer;

    // shih ekf slam data member
    bool checkBit;
    bool motionMode;
    std::size_t sceneCnt;
    std::size_t saveFileIndex;
    std::size_t sceneNum;
    double gridDistance;
    QTimer *testEKFTimer;
    QTimer *scenesTimer;
    bool readFlag;
    DataGrabThread* collector;
//    Shih_MyRobot *SLAM_Robot;
//    lab405::MyRobot *myrobot;
    cv::Point2d odoValuePrevious; //x:right odo y:left odo
    cv::Point2d odoValueCurrent; //x:right odo y:left odo
    cv::Mat landMarkImg;
    cv::Mat gridImg;
    std::ofstream robotOutputFile;
    std::ofstream laserOutputFile;
    std::ofstream robotSceneFile;
    int threcont;
    bool paused;    // for detect people

signals:
    void test();
//    StepMotor_Controller *step_motor;
//    Laser_LMS291_Controller *laser291;
};

#endif // EIC_TEST_H