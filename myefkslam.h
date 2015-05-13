#ifndef MYEFKSLAM_H
#define MYEFKSLAM_H
#include "myrobot.h"
#include <QObject>
namespace lab405 {
class MyEFKSLAM : public QObject
{
    Q_OBJECT
public:
    MyEFKSLAM(double w = 58, int velocity = 30, const cv::Point2d& point=cv::Point2d (150,500));
    ~MyEFKSLAM();
    void Initial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y);
    MyRobot *myrobot;
    // connect the block processes
    void Connect();

    QTimer *EKFTimer;

    // shih's
    OccupancyGridMapping gridMapper;
    LandmarkMapping mapper;
    EKF_SLAM EKFRuner;
    LineExtraction lineExtracter;
    CornerExtraction cornerExtractor;
    PathPlanning planner;
    // mytoolkit
    RobotState robotPosition;
    // data member
    std::vector<double> rawLaserScanData;
    cv::Point2d currentStart;
    cv::Point2d currentEnd;
    cv::Point2d FinalEnd;
    std::queue<std::pair<int,double> > commandSets;
    std::vector<cv::Point2d> robotPathSets;
    std::vector<cv::Point2d> robotPathSetsSmoothing;
//    cv::Point3d robotPose;
    /*----------------------------------------------------------------------------------------------*/
    //laser and camera parameters
    cv::Mat cameraIntrinsicMatrix;
    cv::Mat distortionCoefficients;
    cv::Mat laser_cameraRotationMatrix;
    cv::Mat laser_cameraTranslationMatrix;
    cv::Mat laser_cameraHomogeneous;

    /*----------------------------------------------------------------------------------------------*/
    //ICP
    vector<cv::Point2d> refenceMap;
    inline int GetVelocity(){return robotVelocity; }
    inline int GetRobotWidth(){ return robotWidth;}

    void FindCurrentNodeEnd(const cv::Mat &gridMap,double intervalDistance, const cv::Point2d &currentStart,const cv::Point2d &goal ,cv::Point2d& currentEnd);
    void PathSmoothing();
    void TrajectoryGenerationSmoothing();

public slots:
    void Prediction();
    void Update();

    void Test(int pos);

    void EKFStepExamine();

signals:
    void MotorStop();
private:
    double gridDistance;
    bool checkBit;
    std::size_t sceneCnt;
    std::size_t saveFileIndex;
    std::size_t sceneNum;
    bool motionMode;
    cv::Point2d odoValuePrevious; //x:right odo y:left odo
    cv::Point2d odoValueCurrent; //x:right odo y:left odo
    std::ofstream robotOutputFile;
    std::ofstream laserOutputFile;
    std::ofstream robotSceneFile;
    // shih's
    int robotVelocity;
    double robotWidth;  //cm
    cv::Point2d robotMapOrginal;


};
}
#endif // MYEFKSLAM_H
