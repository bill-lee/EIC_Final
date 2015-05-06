#ifndef CORNEREXTRACTION_H
#define CORNEREXTRACTION_H
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "mytoolkit.h"

class CornerExtraction
{
public:
    //constructor
    CornerExtraction(int num=4,double a=CV_PI*(3.0/2.0),int s=5,double cm=5,cv::Point2d center=cv::Point2d(500,800)):
        cornerNum(num),aperture(a),blurMaskSize(s),pixel_meterFactor(cm),imgCenter(center)
    {
        cornerImg=cv::Mat::zeros(1500,1500,CV_8UC3);
    }

    void ExtractCorners(const std::vector<double> &laserPointSite,std::vector<Corner>& cornerFeature);
    cv::Mat cornerImg;  //�e�X���G

private:
    int cornerNum;  //�Q�n���X�Ө��S�x
    double aperture; //unit:rad �p�g�����y����
    int blurMaskSize;  //�ҽk�B�n�j�p
    double pixel_meterFactor;  //�@��pixel�N��X����
    cv::Point2d imgCenter;
};

#endif // CORNEREXTRACTION_H
