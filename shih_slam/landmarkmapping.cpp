#include "landmarkmapping.h"

void LandmarkMapping::RangesDataToPointsData(const std::vector<double> &laserRangesData, std::vector<cv::Point2d> &laserPointsData)
{   // robot coordinate

    // pi/360
    double resolution = aperture/(laserRangesData.size() - 1);
    laserPointsData.clear();
    laserPointsData.reserve(laserRangesData.size());

    for(int i = 0; i != laserRangesData.size(); ++i)
    {
        double angle = (i*resolution) /*- aperture/2.0*/;

        if(laserRangesData[i] < 2000 && laserRangesData[i] > 30)
        {
            double r = laserRangesData[i];
            double x = r*cos(angle);
            double y = r*sin(angle);
            laserPointsData.push_back(cv::Point2d(x, y));
        }
    }

}

cv::Mat LandmarkMapping::GetLocalLandmarkMap(const std::vector<cv::Point2d>& localMapPointSets,const std::vector<Line>& lineSets,const std::vector<Corner>& cornerSets)
{
    cv::Mat img=cv::Mat::zeros(700,700,CV_8UC3);
    double scale=2.5;
    //cv::circle(img, cv::Point(img.cols/2.0,img.rows/2.0), 5, cv::Scalar(255,0,255), 4 );

    for(int i=0;i!=localMapPointSets.size();++i)
    {
        double x=localMapPointSets[i].x/100.0*(1/pixel_meterFactor*scale)+img.cols/2.0;
        double y=localMapPointSets[i].y/100.0*(1/pixel_meterFactor*scale)+img.rows/2.0;

        cv::circle(img, cv::Point(x,y), 1, cv::Scalar(0,0,255), -1  );
    }


    for(int i=0;i!=lineSets.size();++i)
    {

        RobotState temp;
        temp.robotPositionMean.ptr<double>(0)[0]=0;
        temp.robotPositionMean.ptr<double>(1)[0]=0;
        temp.robotPositionMean.ptr<double>(2)[0]=0;
        DrawLine(lineSets[i],temp,cv::Scalar(0,255,0),1,cv::Point2d(img.cols/2.0,img.rows/2.0),scale,img);

    }


    return img.clone();
}

void LandmarkMapping::DrawLine(const Line &line, const RobotState &robot, const cv::Scalar &s1, const int lineWidth, const cv::Point2d &center,double scale, cv::Mat &showImg)
{
// robot(0,0,0) draw in world coordinate
        cv::Point2d polarPoint,temp1,temp2;
        cv::Point pt1,pt2;
        polarPoint.x=line.lineMean.ptr<double>(0)[0];
        polarPoint.y=line.lineMean.ptr<double>(1)[0];

        PolarToCartesian(polarPoint,temp1);

        double a = cos(polarPoint.y+robot.robotPositionMean.ptr<double>(2)[0]), b = sin(polarPoint.y+robot.robotPositionMean.ptr<double>(2)[0]);


       temp2.x=(cos(robot.robotPositionMean.ptr<double>(2)[0])*temp1.x-sin(robot.robotPositionMean.ptr<double>(2)[0])*temp1.y+robot.robotPositionMean.ptr<double>(0)[0]);
       temp2.y=(sin(robot.robotPositionMean.ptr<double>(2)[0])*temp1.x+cos(robot.robotPositionMean.ptr<double>(2)[0])*temp1.y+robot.robotPositionMean.ptr<double>(1)[0]);

       temp2.x=temp2.x/100.0*(1/pixel_meterFactor*scale)+center.x;
       temp2.y=temp2.y/100.0*(1/pixel_meterFactor*scale)+center.y;


       pt1.x = cvRound(temp2.x + 1000*(-b)); //cvRound ��double�ܦ�int
       pt1.y = cvRound(temp2.y + 1000*(a));
       pt2.y = cvRound(temp2.y - 1000*(a));
       pt2.x = cvRound(temp2.x - 1000*(-b));


       cv::line( showImg, pt1, pt2, s1, lineWidth, CV_AA);




}
void LandmarkMapping::DrawLine(const Feature &line, const RobotState &robot, const cv::Scalar &s1, const int lineWidth, const cv::Point2d &center,double scale, cv::Mat &showImg)
{       //parameter1: polar coordinate

    cv::Point2d polarPoint,temp1,temp2;
    cv::Point pt1,pt2;
    polarPoint.x=line.featureMean.ptr<double>(0)[0];
    polarPoint.y=line.featureMean.ptr<double>(1)[0];

    PolarToCartesian(polarPoint,temp1);


    //////////////////////////////////////////////////////////////
    // combine with robot state
    // theta
    double a = cos(polarPoint.y + robot.robotPositionMean.ptr<double>(2)[0]);
    double b = sin(polarPoint.y + robot.robotPositionMean.ptr<double>(2)[0]);
    // robot center
    temp2.x = cos(robot.robotPositionMean.ptr<double>(2)[0])*temp1.x
            - sin(robot.robotPositionMean.ptr<double>(2)[0])*temp1.y
            + robot.robotPositionMean.ptr<double>(0)[0];
    temp2.y = sin(robot.robotPositionMean.ptr<double>(2)[0])*temp1.x
            + cos(robot.robotPositionMean.ptr<double>(2)[0])*temp1.y
            + robot.robotPositionMean.ptr<double>(1)[0];
    // rescaling
    temp2.x = temp2.x/100.0*(1/pixel_meterFactor*scale)+center.x;
    temp2.y = temp2.y/100.0*(1/pixel_meterFactor*scale)+center.y;


    pt1.x = cvRound(temp2.x + 1000*(-b)); //cvRound convert double into int
    pt1.y = cvRound(temp2.y + 1000*(a));
    pt2.x = cvRound(temp2.x - 1000*(-b));
    pt2.y = cvRound(temp2.y - 1000*(a));


    cv::line( showImg, pt1, pt2, s1, lineWidth, CV_AA);

}
// used
void LandmarkMapping::InsertLocalLandmarkMap(std::vector<cv::Point2d> &laserPointsData, const RobotState &robot)
{

    double x=robot.robotPositionMean.ptr<double>(0)[0];
    double y=robot.robotPositionMean.ptr<double>(1)[0];
    double phi=robot.robotPositionMean.ptr<double>(2)[0];

    for(int i=0;i!=laserPointsData.size();++i)
    {
        double mx=cos(phi)*laserPointsData[i].x - sin(phi)*laserPointsData[i].y + x;
        double my=sin(phi)*laserPointsData[i].x + cos(phi)*laserPointsData[i].y + y;
        mx=mx/100.0*(1.0/pixel_meterFactor) + landmarkMap_originalPoint.x;
        my=my/100.0*(1.0/pixel_meterFactor) + landmarkMap_originalPoint.y;

        cv::circle(landmarkMap , cv::Point(mx,my), 1, cv::Scalar(0,0,255), -1  );
    }




}

void LandmarkMapping::DrawRobotPoseWithErrorEllipse(const RobotState robot, cv::Mat &img, bool showEllipse)
{


    cv::Mat xyCovariance=robot.robotPositionCovariance(cv::Rect(0,0,2,2));
    double x=robot.robotPositionMean.ptr<double>(0)[0];
    double y=robot.robotPositionMean.ptr<double>(1)[0];

    x=x/100.0*(1.0/pixel_meterFactor)+landmarkMap_originalPoint.x;
    y=y/100.0*(1.0/pixel_meterFactor)+landmarkMap_originalPoint.y;


    if(showEllipse==true)
    {
        cv::Mat EigenValue(robot.robotPositionMean.rows,robot.robotPositionMean.cols,CV_64F),EigenVector(robot.robotPositionCovariance.rows,robot.robotPositionCovariance.cols,CV_64F);  //�j�p�n�i�H�ܰ� �T�w�u���e�����H����
        cv::eigen(xyCovariance, EigenValue, EigenVector); //row
        double angle=atan2(EigenVector.ptr<double>(1)[0],EigenVector.ptr<double>(0)[0]);

        double a1=10*sqrt(abs(EigenValue.ptr<double>(0)[0]));
        double a2=10*sqrt(abs(EigenValue.ptr<double>(1)[0]));
        cv::ellipse(img,cv::Point(x,y),cv::Size(a1,a2),angle*180.0/3.1415926535897,0,360,cv::Scalar(255,0 ,0 ),1);

    }

    cv::circle(img, cv::Point(x,y),4, cv::Scalar(0,255,0), -1 );


}

void LandmarkMapping::Reset()
{
    landmarkMap=cv::Mat(landmarkMap_rows,landmarkMap_cols,CV_8UC3); // initialize size of gridmap and set type to CV_64F
    landmarkMap_originalPoint =cv::Point2d(150,500);  // unit: m
    mapColor=cv::Scalar(0,0,255); // Red
}
