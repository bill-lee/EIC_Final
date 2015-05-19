#ifndef MYEFKSLAM_H
#define MYEFKSLAM_H
#include "myrobot.h"
#include <QObject>
namespace lab405 {
class MyEFKSLAM : public QObject
{
    Q_OBJECT
public:
    MyEFKSLAM(double w = 58.0, int velocity = 30,
              const cv::Point2d& point = cv::Point2d (150,500),
              int num = 0, double gl = 0.25, double gc = 0,
              double t = 1.0, double n = 20.0, int weight = 10,
              double _Kr = 3.5, double _Kl = 3.5);
    ~MyEFKSLAM();
    void Initial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold/*??*/);
    MyRobot *myrobot;
    // connect the block processes
    void Connect();

    QTimer *EKFTimer;

    // shih's
    OccupancyGridMapping gridMapper;
    LandmarkMapping mapper;
//    EKF_SLAM EKFRuner;
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
    cv::Point2d odoValuePrevious; // x:right odo y:left odo
    cv::Point2d odoValueCurrent; // x:right odo y:left odo
    std::ofstream robotOutputFile;
    std::ofstream laserOutputFile;
    std::ofstream robotSceneFile;
    // shih's
    int robotVelocity;
    double robotWidth;  //cm
    cv::Point2d robotMapOrginal;

    double slam_x0;
    double slam_x;
    double slam_y;

    // Functions
    void MotionPrediction(const double DeltaR, const double DeltaL);
    void DataAssociation(const std::vector<Line> &obsLineFeature,const std::vector<Corner>& obsCornerFeature);
    double _MahalanobisDistance(const cv::Mat& mean,const cv::Mat& covariance);
    void Update(const cv::Mat& H,const cv::Mat& innovation,const cv::Mat& innovationCovariance);
    void AddNewLandmark(const Feature& singleFeature);
    void FeatureSelection(const std::vector<Feature> &obsFeatureSets, std::vector<Feature> &landmark);
    double _EuclideanDistance(const Feature& obsFeautureW, const Feature& candFeautureW);  //���J���w�q���@�ɮy�Ъ��S�x
    void MapInitial(const std::vector<std::vector<Line> >& lineFeature);
    void SetRobotPose(const RobotState& robot);
    void SetMotionCommand();
    void SetMotionCommand2(int type, double value);
    void ControlMotion(const cv::Point2d& current_start, const cv::Point2d& current_end);

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
    cv::Mat gridImg;
    // ???
    int temp_threscont;
    int thresh_count;
    // data grabber
    int motionType;  //1: forward 2:right 3:left
    double rightCommand;
    double leftCommand;
};
}
#endif // MYEFKSLAM_H
