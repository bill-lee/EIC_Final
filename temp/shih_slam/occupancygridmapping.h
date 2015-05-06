#ifndef OCCUPANCYGRIDMAPPING_H
#define OCCUPANCYGRIDMAPPING_H
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "mytoolkit.h"
#include <pcl/io/pcd_io.h>  //讀取或輸出點雲檔案的class
#include <pcl/point_types.h>  //pcl的資料型態
#include <pcl/point_representation.h>
#include <pcl/visualization/cloud_viewer.h>  //simple viwer

///////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////
const double MAX_RANGE=2996;// cm

class OccupancyGridMapping
{
public:
    ///////////////////////////////////////////////////////////////////////////////////
    // constructor
    ///////////////////////////////////////////////////////////////////////////////////
    OccupancyGridMapping(const int rows=800,const int cols=800,const int obstacle_width=1,const  int laserBeam_width=1,double z=2500,double a=CV_PI*(3.0/2.0),double m=0.25)
        : girdMap_rows(rows),girdMap_cols(cols),alpha(obstacle_width),beta(laserBeam_width),maxZ(z),aperture(a),pixel_meterFactor(m)
    {
        gridMap_originalPoint =cv::Point2d(150,500);  //unit:m

        girdMap=cv::Mat(girdMap_rows,girdMap_cols,CV_64F); //初始化gridmap大小 且設定為單通道CV_64F

    } //unit:公尺(m)

    void InitGridMap(const double lo=0){ girdMap=cv::Scalar::all(lo);}//lo為初始化的數值0~1
    void GetLocalGridMap(const std::vector<double>& laserRangeData,cv::Mat& localMap);
    void InvLaserModel(const double laserBeamRange, const double laserBeamAngle, const cv::Point3d &robotPose2D,cv::Mat& gridMap,bool isGlobal=true);
    void InvLaserAvoidMaxDistModel(const double laserBeamRange, const double laserBeamAngle, const cv::Point3d &robotPose2D,cv::Mat& gridMap,bool isGlobal=true);
    void InsertLocalGridMap(const std::vector<double>& laserRangeData,const cv::Point3d &robotPose2D);
    void InsertLocalGridMap(const std::vector<double> &laserRangeData, const RobotState& robot);
    void GetOccupancyGridMap(cv::Mat& bigGridMap);
    inline void SetGridMapOriginalPoint(const cv::Point2d& point){  gridMap_originalPoint.x=point.x; gridMap_originalPoint.y=point.y;   }
    inline cv::Point2d GetGridMapOriginalPoint(){return cv::Point2d(gridMap_originalPoint.x,gridMap_originalPoint.y);}
    template<class PointT> void InsertPointCloudGridMap(const pcl::PointCloud<PointT> &input,const cv::Point3d &robotPose2D,double upperbound,double lowerbound);
    template<class PointT> void GetLocalPointCloudGridMap(const pcl::PointCloud<PointT> &input,cv::Mat& localMap,double upperbound,double lowerbound);
    double GetPixel_meterFactor(){return pixel_meterFactor; }
    cv::Point2d GetRobotCenter(const cv::Point3d& pose );
    cv::Point2d GetRobotCenter(const RobotState& pose );

private:

    cv::Mat girdMap;
    int girdMap_rows;//unit:公尺(m)*pixel_meterFactor
    int girdMap_cols;//unit:公尺(m)*pixel_meterFactor
    int alpha;  //obstacle width  unit:(cm)
    int beta;  //laser beam width unit:(cm)
    double maxZ;  //雷射最大的量測
    double aperture; //unit:rad 雷射的掃描角度
    double pixel_meterFactor;  //一個pixel代表幾公尺
    cv::Point2d gridMap_originalPoint; //grid map原點

    void _LogToBel(const cv::Mat &localMap,cv::Mat& gridMap8U);

};


template<class PointT>
void OccupancyGridMapping::InsertPointCloudGridMap(const pcl::PointCloud<PointT> &input,const cv::Point3d &robotPose2D,double upperbound=0,double lowerbound=-80)
{
//robotPose2D 是否限制  -CV_PI~CV_PI


    for(int i=0;i!=input.points.size();++i)
    {
        double tempx=input.points[i].x;
        double tempy=input.points[i].y;
        double tempz=input.points[i].z;

        if(tempz<upperbound&&tempz>lowerbound)
        {
            double r=sqrt(pow(tempx,2)+pow(tempy,2));
            double angle=atan2(tempy,tempx);
           // cout<<tempx<<" "<<tempy<<" "<<r<<" "<<angle*180.0/CV_PI<<endl;
            //cv::circle(img, cv::Point(tempx*0.1+500,tempy*0.1+500), 1, cv::Scalar(0,0,255), -1  );
            if(r>35)
                InvLaserModel(r,angle,robotPose2D,girdMap,true);
        }

    }

}

template<class PointT>
void OccupancyGridMapping::GetLocalPointCloudGridMap(const pcl::PointCloud<PointT> &input,cv::Mat& localMap,double upperbound=0,double lowerbound=-80)
{
     cv::Point3d robotPose2D(0,0,0);
     cv::Mat localMapTemp=cv::Mat::zeros(500,500,CV_64F);

     for(int i=0;i!=input.points.size();++i)
     {
         double tempx=input.points[i].x;
         double tempy=input.points[i].y;
         double tempz=input.points[i].z;

         if(tempz<upperbound&&tempz>lowerbound)
         {
             double r=sqrt(pow(tempx,2)+pow(tempy,2));
             double angle=atan2(tempy,tempx);

             InvLaserModel(r,angle,robotPose2D,localMapTemp,false);
         }

     }

     cv::Mat tempMap;
     _LogToBel(localMapTemp,tempMap);
     localMap=tempMap.clone();


}


#endif // OCCUPANCYGRIDMAPPING_H



