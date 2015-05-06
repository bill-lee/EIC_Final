#include "cornerextraction.h"

void CornerExtraction::ExtractCorners(const std::vector<double> &laserPointSite,std::vector<Corner>& cornerFeature)
{
    cornerImg=cv::Scalar::all(0);
    cornerFeature.clear();
    cornerFeature.reserve(cornerNum);
    double resolution =aperture/(laserPointSite.size()-1);
    int Lcnt=0;
    //To draw cornerImg using laser beam
    for (int i=0 ; i<laserPointSite.size() ; i++)
    {


        if( laserPointSite[i]> 0)
        {

            for(int theta = -5; theta <= 5; theta++)
            {
                double x= cos(((double)Lcnt*resolution-aperture/2.0)+(double)(theta*0.1*CV_PI)/180.0) ;
                double y= sin(((double)Lcnt*resolution-aperture/2.0)+(double)(theta*0.1*CV_PI)/180.0) ;
                for(int r = 1; r <= laserPointSite[i]/pixel_meterFactor ; r++)
                {
                    int xx = cvRound(x*r)+imgCenter.x;
                    int yy = cvRound(y*r)+imgCenter.y;
                    if(xx < cornerImg.cols && xx > 0 &&
                            yy < cornerImg.rows && yy > 0)
                    {
                        cv::circle(cornerImg, cv::Point(xx,yy), 1, cv::Scalar(255,255,255), -1  );


                    }
                }
            }

        }
        else
        {

            for(int theta = -5; theta <= 5; theta++)
            {
                double x= cos(((double)Lcnt*resolution-aperture/2.0)+(double)(theta*0.1*CV_PI)/180.0) ;
                double y= sin(((double)Lcnt*resolution-aperture/2.0)+(double)(theta*0.1*CV_PI)/180.0) ;
                for(int r = 1; r <= 5500.0/pixel_meterFactor; r++)
                {
                    int xx = cvRound(x*r)+imgCenter.x;
                    int yy = cvRound(y*r)+imgCenter.y;
                    if(xx < cornerImg.cols && xx > 0 &&
                            yy < cornerImg.rows && yy > 0)
                    {
                        cv::circle(cornerImg, cv::Point(xx,yy), 1, cv::Scalar(255,255,255), -1  );


                    }

                }
            }
        }


        Lcnt++;
    }

    cv::Mat cornerGray;
    cv::cvtColor(cornerImg, cornerGray, CV_BGR2GRAY);
    std::vector<cv::Point2f> cornerTemp;  //corner temp
    cv::blur( cornerGray, cornerGray, cv::Size(blurMaskSize, blurMaskSize ), cv::Point(-1,-1));

    goodFeaturesToTrack(cornerGray,cornerTemp,cornerNum,0.001,5);//AFTER EDIT

    for(int j=0;j!=cornerTemp.size();++j)
    {

        cv::Point2d cartesian,polar;
        Corner oneCorner;
        cartesian.x=(cornerTemp[j].x-imgCenter.x)*pixel_meterFactor;
        cartesian.y=(cornerTemp[j].y-imgCenter.x)*pixel_meterFactor;

        CartesianToPolar(cartesian,polar);

        if(polar.x>=1)
        {
            cv::circle(cornerImg, cv::Point(cornerTemp[j].x,cornerTemp[j].y), 4, cv::Scalar(0,0,255), 2  );
            //corner mean
            oneCorner.cornerMean.ptr<double>(0)[0]=polar.x;   //r
            oneCorner.cornerMean.ptr<double>(1)[0]=polar.y;   //phi
            //corner Covariance
            oneCorner.cornerCovariance.ptr<double>(0)[0]=2.0;
            oneCorner.cornerCovariance.ptr<double>(0)[1]=0;
            oneCorner.cornerCovariance.ptr<double>(1)[0]=0;
            oneCorner.cornerCovariance.ptr<double>(1)[1]=0.08;
            cornerFeature.push_back(oneCorner);
        }


    }


}
