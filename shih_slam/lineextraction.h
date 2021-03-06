#ifndef LINEEXTRACTION_H
#define LINEEXTRACTION_H
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "mytoolkit.h"

#include "pcl/point_cloud.h"    // for pcl::PointCloud
#include "pcl/point_types.h"    // for pcl::PointXYZ

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

class LineExtraction
{
public:
    LineExtraction(double d=100.0, std::size_t _size = 100)
        : distanceThreshold(d), sizethreshold(_size)
    {
        lineImg=cv::Mat::zeros(500,500,CV_8UC3);
    }

    cv::Mat lineImg;

    void SetSizeThreshold(const std::size_t _size){
        std::cout << "SetSizeThreshold = " << _size << std::endl;
        sizethreshold = _size;}

    void SplitAndMerge(const std::vector<cv::Point2d> &PointSite, std::vector<Line>& lineFeature);
    void SetDistanceThreshold(double num){  distanceThreshold=num;  }
    // para.x: rho, para.y: theta
    static void LineRhoThetaExtraction(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, cv::Point2d& para);

    static void GetExtractionLinePara(const std::vector<cv::Point2d>& LaserCatesianPoints, cv::Point2d& para, bool IsShow);

    void SplitAndMerge(const std::vector<cv::Point2d> &PointSite, std::vector<Line> &lineFeature, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
private:
    // distance threshold for straight line
    double distanceThreshold;  // up to your PointSite
    Line _BuildLine(const cv::Point2d& side1,const cv::Point2d& side2);
    bool _FindTheFarthestPoint(const Line &baseLine, const std::vector<cv::Point2d> &PointSite,cv::Point2d& farthestPoint);
    void _PointsetsCalLinePolarParameter(const std::vector<cv::Point2d> &PointSite,Line &singleLineFeature);
    void _SeparatePoints(const Line& baseLine,const cv::Point2d& farthestPoint,const std::vector<cv::Point2d>& PointSite,std::vector<cv::Point2d>& LeltPointSite,std::vector<cv::Point2d>& RightPointSite);

    std::size_t sizethreshold;

};


#endif // LINEEXTRACTION_H


