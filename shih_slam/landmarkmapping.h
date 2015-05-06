#ifndef LANDMARKMAPPING_H
#define LANDMARKMAPPING_H

#include "opencv2/opencv.hpp"
#include "mytoolkit.h"
#include <vector>
#include <iostream>

class LandmarkMapping
{
public:
    LandmarkMapping(const int rows=800,const int cols=800,const double a=(double)CV_PI,const double scale=0.05 )
    :landmarkMap_rows(rows),landmarkMap_cols(cols),aperture(a),pixel_meterFactor(scale)
    {
        landmarkMap=cv::Mat(landmarkMap_rows,landmarkMap_cols,CV_8UC3); // initialize size of gridmap and set type to CV_64F
        landmarkMap_originalPoint =cv::Point2d(150,500);  // unit: m
        mapColor=cv::Scalar(0,0,255); // Red
    }
    void RangesDataToPointsData(const std::vector<double>& laserRangesData,std::vector<cv::Point2d>& laserPointsData);
    void DrawLine(const Line& line,const RobotState &robot=RobotState(),const cv::Scalar& s1=cv::Scalar(0,255,0),const int lineWidth=1,const cv::Point2d& center=cv::Point2d(0,0),double scale=1,cv::Mat& showImg=cv::Mat());
    void DrawLine(const Feature& line,const RobotState &robot=RobotState(),const cv::Scalar& s1=cv::Scalar(0,255,0),const int lineWidth=1,const cv::Point2d& center=cv::Point2d(0,0),double scale=1,cv::Mat& showImg=cv::Mat());
    void InsertLocalLandmarkMap(std::vector<cv::Point2d>& laserPointsData,const RobotState& robot);
    cv::Mat GetLocalLandmarkMap(const std::vector<cv::Point2d>& localMapPointSets,const std::vector<Line>& lineSets,const std::vector<Corner>& cornerSets);
    cv::Mat GetLandmarkMap(){   return landmarkMap.clone();   }
    void DrawRobotPoseWithErrorEllipse(const RobotState robot,cv::Mat& img,bool showEllipse=true);
private:
     cv::Mat landmarkMap;

     int landmarkMap_rows;// unit: meter(m)*pixel_meterFactor
     int landmarkMap_cols;// unit: meter(m)*pixel_meterFactor
     double aperture; // unit:rad laser scan angle
     double pixel_meterFactor;  // meter per pixel
     cv::Point2d landmarkMap_originalPoint; //grid map original point
     cv::Scalar mapColor;

};

#endif // LANDMARKMAPPING_H
