#ifndef ICP_H
#define ICP_H
//#include "mrpt/poses/CPosePDF.h"
#include "mrpt/poses/CPosePDFGaussian.h"
#include "mrpt/poses/CPose2D.h"
#include "mrpt/slam/CSimplePointsMap.h"
#include "mrpt/slam.h"
#include "mrpt/maps.h"
#include <iostream>
#include "opencv2/opencv.hpp"
class ICP
{
public:
    ICP(int num=100,double dist=5.0,double angle=CV_PI*(40.0)/180.0,double sDist=0.001,bool ransac=true);
    void Intial();
    cv::Point3d Align(const std::vector<cv::Point2d>& referenceMap,const std::vector<cv::Point2d>& targetMap,double referenceNumPercent );
private:

    mrpt::slam::CICP icp; //icp object
    mrpt::slam:: CICP::TReturnInfo info; //icp information
    int maxIteration;
    double thresholdDist;
    double thresholdAng;
    double smallestThresholdDist;
    bool doRANSAC;

};

#endif // ICP_H
