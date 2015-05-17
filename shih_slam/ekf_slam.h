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
public:   //0.5  8     //����0.25   10
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
        landmarkSets.reserve(5000);  //�w���}�]5000��landmark�Ŷ�
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

       std::cout<<"�����H���A�x�}"<<robotCombinedState<<std::endl;
       // std::cout<<"�����H�@�ܲ��Ưx�}"<<robotCombinedCovariance<<std::endl;
        //std::cout<<"============================="<<std::endl;
    }

    int count;
private:
    //system parameter
    double robotWidth;  //unit:cm
    int landmarkNum;
    double gatingLine;  //�u�S�xmatch���Z��
    double gatingCorner;  //���S�xmatch���Z��
    double deltaT;
    //state vector
    cv::Mat robotCombinedState;  //yt=[xt m]
    cv::Mat robotCombinedCovariance;
    RobotState robotState;  //when the intial part that needed to set value
    std::vector<Feature>  landmarkSets;  //�a�Ϯy�лPyt��landmark����
    //feature selection
    std::list<Feature> candidateFeatureSets;  //�b�@�ɮy�ФW���S�xsets
    std::list<int>  candidateWeighting;
    double newFeatureRadius;  //�Υb�|�ӧP�_ ���cm
    int weighting;  //���: ����
    //Distance function
    double _EuclideanDistance(const Feature& obsFeautureW, const Feature& candFeautureW);  //��J���w�q��@�ɮy�Ъ��S�x
    double _MahalanobisDistance(const cv::Mat& mean,const cv::Mat& covariance);


    const double Kr;
    const double Kl;
};

#endif // EKF_SLAM_H
