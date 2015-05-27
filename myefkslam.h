#ifndef MYEFKSLAM_H
#define MYEFKSLAM_H

// shih's library
#include "shih_slam/occupancygridmapping.h"
#include "shih_slam/landmarkmapping.h"
#include "shih_slam/lineextraction.h"
//#include "shih_slam/ekf_slam.h"
//#include "shih_slam/pathplanning.h"
#include "astarpathplanning.h"  // Lab405, A Star Path Planning
#include "shih_slam/cornerextraction.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include "myrobot.h"
#include <QObject>
#include <stack>
namespace lab405 {
class MyEFKSLAM : public QObject
{
    Q_OBJECT
public:
    MyEFKSLAM(double w = 58.0, int velocity = 100,
              const cv::Point2d& point = cv::Point2d (150,500),
              int num = 0, double gl = 0.25, double gc = 0,
              double t = 1.0, double n = 20.0, int weight = 10,
              double _Kr = 3.5, double _Kl = 3.5, double _dis = 200);
    ~MyEFKSLAM();
    void Initial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold/*??*/, const string &filename_tPoints);
    MyRobot *myrobot;
    // connect the block processes
    void Connect();

    QTimer *EKFTimer;

    // shih's
    OccupancyGridMapping gridMapper;
    LandmarkMapping mapper;
    LineExtraction lineExtracter;
    CornerExtraction cornerExtractor;

    // mytoolkit
    RobotState robotPosition;

    // Lab405
    AStarPathPlanning planner;

    // data member
    std::vector<double> rawLaserScanData;
    cv::Point2d currentStart;
    cv::Point2d pre_start;
    cv::Point2d GoalEnd;
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

    void FindCurrentNodeEnd(const cv::Mat &gridMap,double intervalDistance, const cv::Point2d &currentStart,const cv::Point2d &goal ,cv::Point2d& GoalEnd);
    void PathSmoothing();
    void TrajectoryGenerationSmoothing();

    // for navigation update
    void NavigationUpdate(bool scene);

    void NavigationInitial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold/*??*/, const string &filename_tPoints);

    // P Control Test
    void PControlInitial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold/*??*/, const string &filename_tPoints);

    QTimer *PcontrolTimer;

    void MotionControl(double Kp, const cv::Point2d &para, double FowardAngledegree, double tolerDegree, double distance);

    void MotionControlAlong(double Kp, const cv::Point2d &para, double FowardAngledegree, double tolerDegree, double distance);

public slots:
    void Prediction();


    void Test(int pos);

    void EKFStepExamine();

    // P Control
    void PControlTest();

    void FinishSceneAndStartTimer(int size);

signals:
    void MotorStop();

    void StartSceneScan(int);
private:
    double gridDistance;
    bool checkBit;
    std::size_t sceneCnt;
    std::size_t saveFileIndex;
    std::size_t sceneNum;
    bool motionMode;
    cv::Point2d odoValuePrevious; // x:right odo y:left odo
    cv::Point2d odoValueCurrent; // x:right odo y:left odo
    std::ofstream robotOutputFile;
    std::ofstream laserOutputFile;
    std::ofstream robotSceneFile;
    std::ofstream robotStateFile;
    // shih's
    int robotVelocity;
    double robotWidth;  //cm
    cv::Point2d robotMapOrginal;

    double slam_x0;
    double slam_x;
    double slam_y;

    // Functions
    void MotionPrediction(const double DeltaR, const double DeltaL);
    void DataAssociationAndUpdate(const std::vector<Line> &obsLineFeature,const std::vector<Corner>& obsCornerFeature);
    double _MahalanobisDistance(const cv::Mat& mean,const cv::Mat& covariance);
    void Update(const cv::Mat& H,const cv::Mat& innovation,const cv::Mat& innovationCovariance);
    void AddNewLandmark(const Feature& singleFeature);
    void FeatureSelection(const std::vector<Feature> &obsFeatureSets, std::vector<Feature> &landmark);
    double _EuclideanDistance(const Feature& obsFeautureW, const Feature& candFeautureW);  //���J���w�q���@�ɮy�Ъ��S�x
    void MapInitial(const std::vector<std::vector<Line> >& lineFeature);
    void SetRobotPose(const RobotState& robot);
    void SetMotionCommand();
    void SetMotionCommand2(int type, double value);
    void ControlMotion();

    // EKF SLAM Data Member
    int landmarkNum;
    double gatingLine;  // threshold of matching line features
    double gatingCorner;  // threshold of matching corner featrues
    double deltaT;
    double newFeatureRadius;  // judge by radius, cm
    int weighting;  // unit: count
    const double Kr;
    const double Kl;

    // state vector
    cv::Mat robotCombinedState;  // yt=[xt m]
    cv::Mat robotCombinedCovariance;
    RobotState robotState;  // when the intial part that needed to set value

    std::vector<Feature>  landmarkSets;  // relationshop between map coordinate and landmarks of yt
    //feature selection
    std::list<Feature> candidateFeatureSets;  // features in world coordinate sets
    std::list<int>  candidateWeighting;

    cv::Mat landMarkImg;
    cv::Mat OccupancyGridMapImg;
    // ???
    int temp_threscont;
    int thresh_count;
    // data grabber
    int motionType;  //1: forward 2:right 3:left
    double rightCommand;
    double leftCommand;

    std::size_t num_tP;
    std::vector<cv::Point2d> tP_set;
    std::size_t tP_count;
    std::size_t navi_count;

    double pre_theta;


    std::size_t turn_count;
    std::size_t scene_count;
    std::stack<cv::Point2d> robotScenePosition;
    double scene_between_dis;
};
}
#endif // MYEFKSLAM_H
