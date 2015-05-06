#ifndef SHIH_MYROBOT_H
#define SHIH_MYROBOT_H
//#include "icp.h"
//#include "lasercontroller.h"
//#include "lasercontrollers200.h"
//#include "motorcontroller.h"
#include "occupancygridmapping.h"
//#include "landmarkmapping.h"
#include "ekf_slam.h"
#include "lineextraction.h"
#include "cornerextraction.h"
#include "pathplanning.h"
//#include "CanonCamera.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <queue>
//PCL
#include <pcl/io/pcd_io.h>  //Ū���ο��X�I���ɮת�class
#include <pcl/point_types.h>  //pcl�����ƫ��A
#include <pcl/point_representation.h>
// Rong-Siou
#include "laser_lms291_controller.h"
#include "dcmotor_controller.h"
#include "stepmotor_controller.h"

class Shih_MyRobot
{
public:
    /*----------------------------------------------------------------------------------------------*/
    //constuctor
    Shih_MyRobot(double width=58.0,int velocity=30,int num=1200,int s=1,double r=0,double l=0,double w=58,const cv::Point2d& point=cv::Point2d (150,500));
    /*----------------------------------------------------------------------------------------------*/
    //destructor
    virtual ~Shih_MyRobot();
    /*----------------------------------------------------------------------------------------------*/
    //set parameter function
    inline void SetVelocity(int v){robotVelocity=v;}
    inline void SetAllSteps(int n){  allSteps=n;  }
    inline void SetShootStep(const int i){ shootStep=i;}
    void SetOdeValue(const double r, const double l);
    void SetRobotPose(const cv::Point3d& robot){    robotPose=robot;}
    /*----------------------------------------------------------------------------------------------*/
    //get parameter functionpath planning
    inline int GetVelocity(){return robotVelocity; }
    inline int GetAllSteps(){  return allSteps;    }
    inline int GetSceneNum(){  return scenesCnt; }
    inline int GetShootStep(){ return shootStep; }
    inline int GetRobotWidth(){ return robotWidth;}
    inline const cv::Point2d  GetMapOrginalPoint(){   return cv::Point2d(robotMapOrginal);}
    cv::Point2d GetOdoValue(){ return cv::Point2d(rightOdeValue,leftOdeValue);}
    /*----------------------------------------------------------------------------------------------*/
    //function
    void SaveRawScanSets(const string& name);
    void AddScenesNum(){    scenesCnt++;    }
    void AddScenes(){sceneSets.push_back(singleScene);   std::cout<<"scenesNum:"<<sceneSets.size()<<std::endl;}
    void TrajectoryGeneration();
    void PathSmoothing();
    void TrajectoryGenerationSmoothing();
    void MotionModel();
    void FindCurrentNodeEnd(const cv::Mat &gridMap,double intervalDistance, const cv::Point2d &currentStart,const cv::Point2d &goal ,cv::Point2d& currentEnd);
    /*----------------------------------------------------------------------------------------------*/
    //function object
    Laser_LMS291_Controller *laser291;
    Laser_LMS291_Controller *laser_slam;
//    LaserControllerS200* laserS200;
//    LaserController* laserLms291;
    StepMotor_Controller *step_motor;
    DCMotor_Controller *left_motor;
    DCMotor_Controller *right_motor;
//    MotorController* stepMotor;
//    MotorController* leftMotor;
//    MotorController* rightMotor;
    cv::VideoCapture* webcam;
//    CanonCamera digitalCam;
    OccupancyGridMapping gridMapper;
    LandmarkMapping mapper;
    EKF_SLAM EKFRuner;
    LineExtraction lineExtracter;
    CornerExtraction cornerExtractor;
    PathPlanning planner;
//    ICP icper;
    /*----------------------------------------------------------------------------------------------*/
    //laser and camera parameters
    cv::Mat cameraIntrinsicMatrix;
    cv::Mat cameradistortionCoefficients;
    cv::Mat laser_cameraRotationMatrix;
    cv::Mat laser_cameraTranslationMatrix;
    cv::Mat laser_cameraHomogeneous;
    /*----------------------------------------------------------------------------------------------*/
    //point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr singleScene;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr singleColorScene;
    std::vector<std::vector<double> > rawScanSets;
    /*----------------------------------------------------------------------------------------------*/
    //s200 laser data
    std::vector<double> rawLaserScanData;


    std::vector<cv::Point2d> robotPathSets;
    std::vector<cv::Point2d> robotPathSetsSmoothing;
    std::queue<std::pair<int,double> > commandSets;
    cv::Point2d currentStart;
    cv::Point2d currentEnd;
    cv::Point2d FinalEnd;

    cv::Point3d robotPose;
    RobotState robotPosition;

    //ICP
    vector<cv::Point2d> refenceMap;
    /*----------------------------------------------------------------------------------------------*/
private:
    double robotFrameWidth;  //cm
    int robotVelocity;
    int allSteps;
    int scenesCnt;
    double rightOdeValue;
    double leftOdeValue;
    double robotWidth;  //cm
    int shootStep;
    double deltaT;

    cv::Point2d robotMapOrginal;



    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sceneSets;



};

#endif // SHIH_MYROBOT_H
