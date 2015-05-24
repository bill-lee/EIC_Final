#include "occupancygridmapping.h"

using namespace std;


void OccupancyGridMapping::GetLocalGridMap(const std::vector<double> &laserRangeData, cv::Mat &localMap)
{

    cv::Mat localMapTemp=cv::Mat::zeros(300,300,CV_64F);
    cv::Point3d robotPose2D(0,0,0);
    double resolution =aperture/(laserRangeData.size()-1);


    for(int i=10;i!=laserRangeData.size();++i)
    {
       // if((laserRangeData[i]>=MAX_RANGE||laserRangeData[i]<=10))

        if((laserRangeData[i]>=MAX_RANGE||laserRangeData[i]<=10)&&(i<(laserRangeData.size()*0.5-5)&&i>(laserRangeData.size()*0.5+5)))
            continue;

        double angle=(i*resolution)-aperture/2.0;

        if(laserRangeData[i]<2000)
            InvLaserModel(laserRangeData[i],angle,robotPose2D,localMapTemp,false);
        else
            InvLaserAvoidMaxDistModel(laserRangeData[i],angle,robotPose2D,localMapTemp,false);

    }
    cv::Mat tempMap;
    _LogToBel(localMapTemp,tempMap);

    localMap=tempMap.clone();



}

void OccupancyGridMapping::InvLaserModel(const double laserBeamRange, const double laserBeamAngle,
                                         const cv::Point3d& robotPose2D, cv::Mat& gridMap,
                                         bool isGlobal)
{
    // z: laserBeamRange(unit:cm)   follow robot pose to fill the value
    double lo = 0;
    double lfree = -5 ;
    double locc=5;

    cv::Point2d originalPoint(gridMap.cols/2,gridMap.rows/2);

    if(isGlobal){ originalPoint.x=gridMap_originalPoint.x; originalPoint.y=gridMap_originalPoint.y; }

    for(int i=-beta;i!=beta;++i)
    {


        for(int r=1;r<(laserBeamRange+(double)alpha/2.0);++r)
        {

            double phi=(double)(i*CV_PI/180.0)/1.0+(robotPose2D.z)+laserBeamAngle;
            double mx=r*cos(phi)+robotPose2D.x;
            double my=r*sin(phi)+robotPose2D.y;  //cm

          //  cout<<r<<" "<<(laserBeamRange+(double)alpha/2.0)<<endl;
            mx=mx/100.0*(1.0/pixel_meterFactor)+originalPoint.x;   //cm to m
            my=my/100.0*(1.0/pixel_meterFactor)+originalPoint.y;
            //cout<<mx<<" "<<my<<endl;
            if(gridMap.cols>mx && mx>0 && gridMap.rows>my &&my>0)
            {
                if(r>(laserBeamRange-(double)alpha/2.0))
                    gridMap.ptr<double>((int)my)[(int)mx]= locc;
                else
                    gridMap.ptr<double>((int)my)[(int)mx]= lfree;
            }

         }



    }

}

void OccupancyGridMapping::_LogToBel(const cv::Mat &localMap,cv::Mat& gridMap8U)
{
     gridMap8U=cv::Mat::zeros(localMap.size(),CV_8U);

    for(int i = 0; i < localMap.rows; ++i)
       for(int j = 0; j < localMap.cols; ++j)
       {
          double temp= 1 - (1/(1+exp(localMap.ptr<double>(i)[j])));
          gridMap8U.ptr<uchar>(i)[j]=255-255*temp;  //255-255*temp;

       }
}

void OccupancyGridMapping::InsertLocalGridMap(const std::vector<double> &laserRangeData, const cv::Point3d &robotPose2D)
{
    double resolution = aperture/(laserRangeData.size()-1);

//    std::cout << "aperture = " << aperture << std::endl;
//    std::cout << "resolution = " << resolution << std::endl;
//    std::cout << "laserRangeData.size() = " << laserRangeData.size() << std::endl;

    for(int i = 10; i != laserRangeData.size(); ++i)
    {

        if((laserRangeData[i] >= MAX_RANGE|| laserRangeData[i] <= 10)
                && (i < (laserRangeData.size()*0.5 - 5)
                   && i > (laserRangeData.size()*0.5 + 5)))
            continue;

        double angle = (i*resolution) - aperture/2;

        if(laserRangeData[i] < 2000)
            InvLaserModel(laserRangeData[i], angle, robotPose2D, girdMap);
        else
            InvLaserAvoidMaxDistModel(laserRangeData[i], angle, robotPose2D, girdMap);


    }


}
void OccupancyGridMapping::InsertLocalGridMap(const std::vector<double> &laserRangeData, const RobotState& robot)
{
    double resolution = aperture/(laserRangeData.size() - 1);

    cv::Point3d robotPose2D;
    robotPose2D.x = robot.robotPositionMean.ptr<double>(0)[0];
    robotPose2D.y = robot.robotPositionMean.ptr<double>(1)[0];
    robotPose2D.z = robot.robotPositionMean.ptr<double>(2)[0];

    //
    for(int i = 10; i != laserRangeData.size(); ++i)
    {

        if((laserRangeData[i] >= MAX_RANGE||laserRangeData[i] <= 10)
                && (i < (laserRangeData.size()*0.5 - 5)
                    && i > (laserRangeData.size()*0.5 + 5)))
            continue;
        double angle = (i*resolution) - aperture/2;

        // beam model
        if(laserRangeData[i]<2000)
            InvLaserModel(laserRangeData[i],angle,robotPose2D,girdMap);
        else
            InvLaserAvoidMaxDistModel(laserRangeData[i],angle,robotPose2D,girdMap);
    }


}
void OccupancyGridMapping::GetOccupancyGridMap(cv::Mat &bigGridMap)
{
        cv::Mat tempMap;
        _LogToBel(girdMap,tempMap);
        bigGridMap=tempMap.clone();

}

cv::Point2d OccupancyGridMapping::GetRobotCenter(const cv::Point3d &pose)
{
    //world robot pose to grid map
    double mx=pose.x/100.0*(1.0/pixel_meterFactor)+gridMap_originalPoint.x;   //cm to m
    double my=pose.y/100.0*(1.0/pixel_meterFactor)+gridMap_originalPoint.y;


    return cv::Point2d(mx,my);
}

cv::Point2d OccupancyGridMapping::GetRobotCenter(const RobotState& pose)
{
    //world robot pose to grid map
    double mx=pose.robotPositionMean.ptr<double>(0)[0]/100.0*(1.0/pixel_meterFactor) + gridMap_originalPoint.x;   //cm to m
    double my=pose.robotPositionMean.ptr<double>(1)[0]/100.0*(1.0/pixel_meterFactor) + gridMap_originalPoint.y;


    return cv::Point2d(mx,my);
}

void OccupancyGridMapping::InvLaserAvoidMaxDistModel(const double laserBeamRange, const double laserBeamAngle, const cv::Point3d &robotPose2D, cv::Mat &gridMap, bool isGlobal)
{

       double lo = 0;
       double lfree = -5 ;
       double locc = 5;

       double validRange = laserBeamRange*0.3;
       cv::Point2d originalPoint(gridMap.cols/2,gridMap.rows/2);

       if(isGlobal){ originalPoint.x=gridMap_originalPoint.x; originalPoint.y=gridMap_originalPoint.y; }

       for(int i=-beta;i!=beta;++i)
       {


           for(int r=1;r<(validRange+(double)alpha/2.0);++r)
           {

               double phi=(double)(i*CV_PI/180.0)/1.0+(robotPose2D.z)+laserBeamAngle;
               double mx=r*cos(phi)+robotPose2D.x;
               double my=r*sin(phi)+robotPose2D.y;  //cm

             //  cout<<r<<" "<<(laserBeamRange+(double)alpha/2.0)<<endl;
               mx=mx/100.0*(1.0/pixel_meterFactor)+originalPoint.x;   //cm to m
               my=my/100.0*(1.0/pixel_meterFactor)+originalPoint.y;
               //cout<<mx<<" "<<my<<endl;
               if(gridMap.cols>mx && mx>0 && gridMap.rows>my &&my>0)
               {
                   //if(r>(validRange-(double)alpha/2.0))
                    //   gridMap.ptr<double>((int)my)[(int)mx]= locc;
                  // else
                       gridMap.ptr<double>((int)my)[(int)mx]= lfree;
               }

            }



       }


}

