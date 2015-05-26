#ifndef LINEEXTRACTION_H
#define LINEEXTRACTION_H
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "mytoolkit.h"

#include "pcl/point_cloud.h"    // for pcl::PointCloud
#include "pcl/point_types.h"    // for pcl::PointXYZ

class LineExtraction
{
public:
    LineExtraction(double d=100.0):distanceThreshold(d)
    {
        lineImg=cv::Mat::zeros(500,500,CV_8UC3);
    }

    cv::Mat lineImg;
    void SplitAndMerge(const std::vector<cv::Point2d> &PointSite,std::vector<Line>& lineFeature);
    void SetDistanceThreshold(double num){  distanceThreshold=num;  }
    // para.x: rho, para.y: theta
    static void LineRhoThetaExtraction(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, cv::Point2d& para);
private:
    double distanceThreshold;  // up to your PointSite
    Line _BuildLine(const cv::Point2d& side1,const cv::Point2d& side2);
    bool _FindTheFarthestPoint(const Line &baseLine, const std::vector<cv::Point2d> &PointSite,cv::Point2d& farthestPoint);
    void _PointsetsCalLinePolarParameter(const std::vector<cv::Point2d> &PointSite,Line &singleLineFeature);
    void _SeparatePoints(const Line& baseLine,const cv::Point2d& farthestPoint,const std::vector<cv::Point2d>& PointSite,std::vector<cv::Point2d>& LeltPointSite,std::vector<cv::Point2d>& RightPointSite);


};


#endif // LINEEXTRACTION_H


