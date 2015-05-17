#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include "opencv2/opencv.hpp"
#include "mytoolkit.h"
#include <vector>
#include <list>
#include <iostream>
#include "landmarkmapping.h"
class EKF_SLAM
{
public:   //0.5  8     //測試0.25   10
    EKF_SLAM(double w = 58.0, int num = 0, double gl = 0.25, double gc = 0,
             double t = 1, double n = 20, int weight = 10,
             double _Kr = 3.5, double _Kl = 3.5)
        : robotWidth(w), landmarkNum(num), gatingLine(gl), gatingCorner(gc),
          deltaT(t), newFeatureRadius(n), weighting(weight),
          Kr(_Kr), Kl(_Kl)
    {  //n=20
        robotState.robotPositionMean=cv::Scalar::all(0);

        robotState.robotPositionCovariance=cv::Mat::eye(robotState.robotPositionCovariance.size(),robotState.robotPositionCovariance.type());
        robotState.robotPositionCovariance.ptr<double>(0)[0]=0.001;
        robotState.robotPositionCovariance.ptr<double>(1)[1]=0.001;
        robotState.robotPositionCovariance.ptr<double>(2)[2]=0.5;


        robotCombinedState=cv::Mat::zeros(STATE_SIZE+OBS_SIZE*landmarkNum,1,CV_64F);
        robotCombinedCovariance=cv::Mat::eye(STATE_SIZE+OBS_SIZE*landmarkNum,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);
        robotCombinedCovariance.ptr<double>(0)[0]=0.001;
        robotCombinedCovariance.ptr<double>(1)[1]=0.001;
        robotCombinedCovariance.ptr<double>(2)[2]=0.5;


        landmarkSets.clear();
        landmarkSets.reserve(5000);  //預先開設5000個landmark空間
        candidateFeatureSets.clear();
        candidateWeighting.clear();

    }
    /**************************************************************************************/
    //using FeatureSelection to enhance new landmark
    /**************************************************************************************/
    void Initial(const std::vector<std::vector<Line> >& lineFeature);
    /**************************************************************************************/
    void FeatureSelection(const std::vector<Feature> &obsFeatureSets, std::vector<Feature> &landmark);
    /**************************************************************************************/
    //deltaOdometryValue [right=odometryValue.x]  [left=odometryValue.y]
    /**************************************************************************************/
    void MotionPrediction(const cv::Point2d& odometryValue);
    /**************************************************************************************/
    void DataAssociation(const std::vector<Line>& obsLineFeature,const std::vector<Corner>& obsCornerFeature);
    /**************************************************************************************/
    //Kalman filter framework
    /**************************************************************************************/
    void Update(const cv::Mat& H,const cv::Mat& innovation,const cv::Mat& innovationCovariance);
    /**************************************************************************************/
    //add map
    /**************************************************************************************/
    void AddNewLandmark(const Feature& singleFeature);
    /**************************************************************************************/
    void GetLandmarkSets(std::vector<Feature>& map);
    /**************************************************************************************/
    RobotState GetRobotPose(){     return  robotState;           }
    /**************************************************************************************/
    int GetLandmarkNum(){   return landmarkNum; }
    /**************************************************************************************/
    void SetRobotPose(const RobotState& robot);
    /**************************************************************************************/
    void Show()
    {

       std::cout<<"機器人狀態矩陣"<<robotCombinedState<<std::endl;
       // std::cout<<"機器人共變異數矩陣"<<robotCombinedCovariance<<std::endl;
        //std::cout<<"============================="<<std::endl;
    }

    int count;
private:
    //system parameter
    double robotWidth;  //unit:cm
    int landmarkNum;
    double gatingLine;  //線特徵match的距離
    double gatingCorner;  //角特徵match的距離
    double deltaT;
    //state vector
    cv::Mat robotCombinedState;  //yt=[xt m]
    cv::Mat robotCombinedCovariance;
    RobotState robotState;  //when the intial part that needed to set value
    std::vector<Feature>  landmarkSets;  //地圖座標與yt的landmark對應
    //feature selection
    std::list<Feature> candidateFeatureSets;  //在世界座標上的特徵sets
    std::list<int>  candidateWeighting;
    double newFeatureRadius;  //用半徑來判斷 單位cm
    int weighting;  //單位: 次數
    //Distance function
    double _EuclideanDistance(const Feature& obsFeautureW, const Feature& candFeautureW);  //輸入為定義於世界座標的特徵
    double _MahalanobisDistance(const cv::Mat& mean,const cv::Mat& covariance);


    const double Kr;
    const double Kl;
};

#endif // EKF_SLAM_H
