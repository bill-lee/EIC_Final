#ifndef MYTOOLKIT_H
#define MYTOOLKIT_H

#include "opencv2/opencv.hpp"
#include "stlastar.h"
#include <vector>
#include <iostream>
#include <fstream>

const int OBS_SIZE=2;
const int STATE_SIZE=3;
const int MOT_SIZE=2;

enum{Line_Feature,Corner_Feature}; //Line_Feature, corner_Feature

const double degreesToRadius=(double)3.1415926535897/180.0;
const double radiusToDegree=(double)180.0/3.1415926535897;

class Line
{
public:

    Line(double aa=0.0,double bb=0.0):a(aa),b(bb)
    {
        lineMean=cv::Mat::zeros(OBS_SIZE,1,CV_64F);  //mean =2x1   build a pointer matrix
        lineCovariance=cv::Mat::ones(OBS_SIZE,OBS_SIZE,CV_64F);
    }
    // constructor
    Line(cv::Mat m,cv::Mat c ):lineMean(m),lineCovariance(c){}
    cv::Point2d& GetLineParameter() const {    return cv::Point2d(a,b); }
    void SetLineParameter(const cv::Point2d& lineParameter)
    {
        a=lineParameter.x;  b=lineParameter.y;
    }

    cv::Mat lineMean;  // polar coordinate: phi, r
    cv::Mat lineCovariance;


private:   // Y = aX + b, line coeffient
    double a;
    double b;

};


class Corner
{
public:
        Corner()
        {
            cornerMean=cv::Mat::zeros(OBS_SIZE,1,CV_64F);  //mean =2x1    build a pointer matrix
            cornerCovariance=cv::Mat::ones(OBS_SIZE,OBS_SIZE,CV_64F);
        }
        Corner(cv::Mat m,cv::Mat c ):cornerMean(m),cornerCovariance(c){}

        cv::Mat cornerMean;  // polar coordinate: phi, r
        cv::Mat cornerCovariance;
};
class Feature
{
public:
        Feature(int type=Line_Feature):featureType(type)
        {
            featureMean=cv::Mat::zeros(OBS_SIZE,1,CV_64F);  // mean = 2x1   build a pointer matrix
            featureCovariance=cv::Mat::ones(OBS_SIZE,OBS_SIZE,CV_64F);
        }
        Feature(cv::Mat m,cv::Mat c ):featureMean(m),featureCovariance(c){}
        void SetFeatureType(int t){       featureType=t;          }
        int GetFeatureType() const {return featureType; }
        cv::Mat featureMean;  // polar coordinate: phi, r
        cv::Mat featureCovariance;

private:
        int featureType;
};


class RobotState
{
public:
    RobotState()
    {
         robotPositionMean=cv::Mat::zeros(STATE_SIZE,1,CV_64F);  // mean = 2x1   build a pointer matrix
         robotPositionCovariance=cv::Mat::ones(STATE_SIZE,STATE_SIZE,CV_64F);
    }
    RobotState(cv::Mat m,cv::Mat c ):robotPositionMean(m),robotPositionCovariance(c){}

    cv::Mat robotPositionMean;//[x ; y ; phi]
    cv::Mat robotPositionCovariance;

};


class CvMatSearchNode
{
public:
        unsigned int x;	 // the (x,y) positions of the node
        unsigned int y;

        static cv::Mat gridMapN;

        CvMatSearchNode() { x = y = 0; }
        CvMatSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }

        float GoalDistanceEstimate( CvMatSearchNode &nodeGoal );
        bool IsGoal( CvMatSearchNode &nodeGoal );
        bool GetSuccessors( AStarSearch<CvMatSearchNode> *astarsearch, CvMatSearchNode *parent_node );
        float GetCost( CvMatSearchNode &successor );
        bool IsSameState( CvMatSearchNode &rhs );

        void PrintNodeInfo();
        static double GetGridMapValue( int x1, int y1 );


};




// point->polar to Cartesian
void PolarToCartesian(const cv::Point2d &polarPoint,cv::Point2d &cartesianPoint);
// point->Cartesian to polar
void CartesianToPolar(const cv::Point2d &cartesianPoint,cv::Point2d &polarPoint);
// feature in polar, from robot to world
// Feture
void ConvertRobotToWorld(const Line& localLine,const RobotState& robotPose, Line& globalLine);
void ConvertRobotToWorld(const Corner& localCorner,const RobotState& robotPose, Corner& globalCorner);
void ConvertRobotToWorld(const Feature& localFeature,const RobotState& robotPose, Feature& globalFeature);
// feature in polar, from world to robot
void ConvertWorldToRobot(const Feature& globalFeature,const RobotState& robotPose, Feature& localLine) ;
// feature in catesian, from robot to world
void dataConvertRobotToWorld(const std::vector<cv::Point2d>& localPointSets,const RobotState& robotPose, std::vector<cv::Point2d>& globalPointSets);
// load offine laser data
void LoadLaserData(const char *dataName,std::vector<std::vector<double> > &laserData );
void LoadOdomData(const char *dataName,std::vector<cv::Point2d> &odomData );




#endif // MYTOOLKIT_H
