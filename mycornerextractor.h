#ifndef MYCORNEREXTRACTOR_H
#define MYCORNEREXTRACTOR_H
#include "pcl/point_cloud.h"    // for pcl::PointCloud
#include "pcl/point_types.h"    // for pcl::PointXYZ
#include <fstream>              // for std::ifstream
#include "opencv2/opencv.hpp"   // opencv
#include "pcl/visualization/cloud_viewer.h"     // for pcl::visualization::CloudViewer
#include "pcl/common/transforms.h"  // for pcl::transformPointCloud
#include <limits>

namespace lab405{
class MyCornerExtractor
{
public:
    MyCornerExtractor();
    ~MyCornerExtractor();

    void GetCornerFeature(std::vector<double> _range_data);
private:
    bool CornerCheckXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::size_t windows, std::size_t start, cv::Point2d f_tor, double d_tor, double c_tor, cv::Point2d dis_var_tor);
};
}


#endif // MYCORNEREXTRACTOR_H
