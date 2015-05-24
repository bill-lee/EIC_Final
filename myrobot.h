#ifndef MYROBOT_H
#define MYROBOT_H

// robot sensors
#include "stepmotor_controller.h"       // step motor
#include "laser_lms291_controller.h"    // laser scanner lms291
#include "dcmotor_controller.h"         // dc motor

#include <Windows.h>                    // for Sleep(millisecond)

#include <QObject>                          // for class QObject
#include <fstream>                      // for std::ofstream

#include <QTime>                        // for QTime



#include <QMetaType>
Q_DECLARE_METATYPE(boost::shared_ptr<QByteArray>)
Q_DECLARE_METATYPE(boost::shared_ptr<cv::Mat>)

#ifndef Mypi
#define Mypi
const double pi = 3.1415926535897932384626433832795;
#endif  // Mypi
namespace lab405 {
enum Odotype
{
    Forward,
    Backward,
    Leftward,
    Rightward
};

class MyRobot : public QObject
{
    Q_OBJECT
public:
    MyRobot();
    ~MyRobot();
    // trigger data acquisition
    // start_angle in degree
    void DataAcquisition(double start_angle, int _scan_count, double end_angle, QString filename, const bool _with_color);

    void DataAcquisitionConti(double _start_angle, int _scan_count, double _end_angle, QString filename, const bool _with_color, const int cam_num);

    void ConnectDataAcquisition(bool _with_color);

    void DisconnectDataAcquisition(bool _with_color);

    void StopDataAcquisition();

    void StopDataAcquisitionConti();



    void GoBackward(double distance_m, double speed);

    void TurnLeft(double angle, double speed);

    void TurnRight(double angle, double speed);

    // robot sensor object pointer
    StepMotor_Controller *step_motor;

    Laser_LMS291_Controller *laser_lms291;

    Laser_LMS291_Controller *laser_slam;

    DCMotor_Controller *right_dcmotor;

    DCMotor_Controller *left_dcmotor;

    void test();

    void rotate();

    // 2015/03/18
    void SpinData();

    void ConnectSpinData();
    // 2015.03.27

    void SaveOfflineSLAMFile(const std::string &filename);

    // 2015.04.10
    std::string offlineSLAM_filename;

    std::string odo_filename;

    void ResetCount() {odo_cmd_count = 0; offlineSLAM_count = 0;}



    // function





public slots:
    void GoForwardThread(double distance_m, double speed);

    void GoForward(double distance_m, double speed);

    void MotorStop();
private slots:


    // 2015.01.30
    void PushDataToBufferThread(boost::shared_ptr<QByteArray> scan);

    void PushDataToBufferP(QByteArray *scan);

    void RotateToNextScan();

    void StepMotorStatus();

    void StopContiTrigger();

    void ContiTigger();

    void RangeAndColorDataAcquisition();

    // 2015.01.30

    void RangeAndColorDataAcquisitionConti();

    void CheckFinishedDataAcquisition();

    void SaveImageThread(boost::shared_ptr<cv::Mat> ptr);

    void Test();


    void TakePictureThread();

    void rotatethread();

    // 2015.03.27
    void SaveOfflineSLAMThread(boost::shared_ptr<QByteArray> scan);


    void SaveOdoMeter(Odotype type, int value);

    // process single trigger
    void ProcessSingleTriggerSLAM(boost::shared_ptr<QByteArray> scan);


signals:
    // debug signals
    // data acquisition signals
    void ProgressBarSignal(int count);

    void FinishedDataAquisition(int size);

    void CameraOpened();

    void DatasegmentSignal(int);

    void PushBufferSignal(int);

    void CheckFinishOneScan(int, bool, bool);

    void CompleteOneScan();

    void CompleteTakingImage();

    void CompleteLaser();

    void CompleteWriteData();

    void ImageReadyToSave(boost::shared_ptr<cv::Mat> ptr);

    void OfflineSLAM_ScanSize(int);

    void EmitOdoMeter(Odotype type, int value);



private:
    // 2015.01.30
    void PushDataToBuffer(boost::shared_ptr<QByteArray> scan);

    int lasercount;

    int scancount;

    double rot_ang;

    QByteArray databuffer;

    void WriteData();

    QString data_filename;

    QString image_foldername;

    bool is_tigger;

    bool with_color;



    void TakePicture();

    void SaveImage(boost::shared_ptr<cv::Mat> ptr);

    // video capture
    cv::VideoCapture capture;



    double start_angle;
    double end_angle;

    bool completeimage;

    bool completelaser;

    QTime wholetime;

    QTime laser;
    QTime whole;


    // 2015.03.27
    void SaveOfflineSLAM(boost::shared_ptr<QByteArray> scan);

    QByteArray temp_SLAM;
    // 2015.04.10
    std::size_t offlineSLAM_count;


    std::size_t odo_cmd_count;

    cv::Mat cameraIntrinsicMatrix;
    cv::Mat distortionCoefficients;




};
}


#endif // MYROBOT_H
