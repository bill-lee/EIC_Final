#include "lineextraction.h"

using namespace std;
//PointSite:[x  y]
void LineExtraction::SplitAndMerge(const std::vector<cv::Point2d> &PointSite,std::vector<Line>& lineFeature)
{
    cv::Point2d side1(0,0),side2(0,0);
    double maxDistance=0;

    if(PointSite.size() < 10)  // ending condition, with condition of small than 10
        return ;

    for (int i=0 ; i<PointSite.size() ; i++) //finding the furthest points
        for (int j=i+1; j<PointSite.size() ; j++){
           double D = sqrt( pow(PointSite[i].x-PointSite[j].x,2) + pow(PointSite[i].y-PointSite[j].y,2) );
           if (D>maxDistance){
                 side1=cv::Point2d(PointSite[i].x,PointSite[i].y);
                 side2=cv::Point2d(PointSite[j].x,PointSite[j].y);
                 maxDistance=D;

           }
        }

    Line  LineA=_BuildLine(side1,side2);  // using farthest two points compute line

    cv::Point2d farthestPoint;


    bool findPoint=_FindTheFarthestPoint(LineA,PointSite,farthestPoint);




    if(findPoint==false){

        Line temp;
        _PointsetsCalLinePolarParameter(PointSite,temp);
        lineFeature.push_back(temp);

        return;
    }
    std::vector<cv::Point2d> RightPointSite,LeftPointSite;
    RightPointSite.reserve(1000);
    LeftPointSite.reserve(1000);

    _SeparatePoints(LineA,farthestPoint,PointSite,LeftPointSite,RightPointSite);



    SplitAndMerge(LeftPointSite,lineFeature);
    SplitAndMerge(RightPointSite,lineFeature);

}

Line LineExtraction::_BuildLine(const cv::Point2d &side1, const cv::Point2d &side2)
{
    // y = ax + b
    double a=0,b=0;
    // Slope: a
    a=(side1.y-side2.y)/(side1.x-side2.x);
    // bring slope into equation
    // b = y1 - a*x1
    b=(side1.x*side2.y - side1.y*side2.x) / (side1.x-side2.x);
    return Line(a,b);
}

bool LineExtraction::_FindTheFarthestPoint(const Line &baseLine, const std::vector<cv::Point2d> &PointSite, cv::Point2d &farthestPoint)
{
    double t=0,maxDistance=0;
    cv::Point2d temp(0,0);
    bool flag=false;

    double a=baseLine.GetLineParameter().x;  //a
    double b=baseLine.GetLineParameter().y; //b


    for(int i=0;i!=PointSite.size();++i)
    {
        t=abs(-1*a*(PointSite[i].x)+(PointSite[i].y)-b)/sqrt(pow(a,2)+1);

        if(t>maxDistance && t>this->distanceThreshold){


            temp=cv::Point2d(PointSite[i].x,PointSite[i].y);
            maxDistance=t;
            flag=true;
        }


    }

    farthestPoint=temp;

    return flag;
}

void LineExtraction::_PointsetsCalLinePolarParameter(const std::vector<cv::Point2d> &PointSite,Line &singleLineFeature)
{

    ////////////////////////////////////////////////////
    //  A Group of Catesian Points, to line parameter
    ///////////////////////////////////////////////////
      double temp1=0,temp2=0,temp3=0,temp4=0;

      for(int i=0;i!=PointSite.size();++i){
          temp1+=PointSite[i].x;
          temp2+=PointSite[i].y;
          temp3+=PointSite[i].x*PointSite[i].x;
          temp4+=PointSite[i].x*PointSite[i].y;

       }

      temp1/=PointSite.size();
      temp2/=PointSite.size();
      temp3/=PointSite.size();
      temp4/=PointSite.size();

      //Y=AX  --->X=A^-1*Y
      cv::Mat matrixA(2,2, CV_64F),Y(2,1,CV_64F),X(2,1,CV_64F);

      //row col
      Y.ptr<double>(0)[0]=temp4;
      Y.ptr<double>(1)[0]=temp2;

      matrixA.ptr<double>(0)[0]=temp3;
      matrixA.ptr<double>(0)[1]=temp1;
      matrixA.ptr<double>(1)[0]=temp1;
      matrixA.ptr<double>(1)[1]=1;

      X=matrixA.inv()*Y;

      ///////////////////////////////////////////
      // intersection of origin (0, 0) and  X.ptr<double>(1)[0]
      ///////////////////////////////////////////
      double m2=-1/X.ptr<double>(0)[0];
      double b2=-m2*0+0; // origin (0,0)
      double xx=(b2-X.ptr<double>(1)[0])/(X.ptr<double>(0)[0]-m2);
      double yy=(m2*X.ptr<double>(1)[0]-X.ptr<double>(0)[0]*b2)/(m2-X.ptr<double>(0)[0]);

      cv::Point  pt1,pt2;       // int type point
      cv::Point2d temp; // polar type

      CartesianToPolar(cv::Point2d(xx,yy),temp);


      // a b parameter
      singleLineFeature.SetLineParameter(cv::Point2d(X.ptr<double>(0)[0],X.ptr<double>(1)[0])); //a  b
      // line mean
      singleLineFeature.lineMean.ptr<double>(0)[0]=temp.x ;//r phi
      singleLineFeature.lineMean.ptr<double>(1)[0]=temp.y ;

      // line Covariance, diagonal matrix
      singleLineFeature.lineCovariance.ptr<double>(0)[0]=2.0;//2
      singleLineFeature.lineCovariance.ptr<double>(1)[1]=0.025;//0.025
      singleLineFeature.lineCovariance.ptr<double>(0)[1]=0.0;
      singleLineFeature.lineCovariance.ptr<double>(1)[0]=0.0;


      //////////////////////////////////////////////////////
      // Draw line
      ///////////////////////////////////////////////////
      double a = cos(temp.y), b = sin(temp.y);
      xx=xx*0.3+lineImg.cols/2.0;
      yy=yy*0.3+lineImg.rows/2.0;  // for show purpose



      pt1.x = cvRound(xx + 1000*(-b)); //cvRound double to int
      pt1.y = cvRound(yy + 1000*(a));
      pt2.x = cvRound(xx - 1000*(-b));
      pt2.y = cvRound(yy - 1000*(a));

      cv::line( lineImg, pt1, pt2, cv::Scalar(0,255,0), 0.5, CV_AA);



}

void LineExtraction::_SeparatePoints(const Line &baseLine, const cv::Point2d &farthestPoint, const std::vector<cv::Point2d> &PointSite, std::vector<cv::Point2d> &LeltPointSite, std::vector<cv::Point2d> &RightPointSite)
{
    RightPointSite.clear();
    LeltPointSite.clear();
    double temp=0;
    double m2=-1/baseLine.GetLineParameter().x; //a
   double b2=-m2*farthestPoint.x+farthestPoint.y;

    for(int i=0;i!=PointSite.size();++i){
        temp=(PointSite[i].x)*m2+b2-(PointSite[i].y);

        if(temp>0)//分開兩群點的演算法  感謝曹育智提供的講義
            RightPointSite.push_back(PointSite[i]);
        else
            LeltPointSite.push_back(PointSite[i]);

    }

}
