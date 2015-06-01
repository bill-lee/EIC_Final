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

void LineExtraction::SplitAndMerge(const std::vector<cv::Point2d> &PointSite,std::vector<Line>& lineFeature, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // temp member for max distance points
    cv::Point2d side1(0,0),side2(0,0);
    double maxDistance=0;

    if(PointSite.size() < 50) // ending condition, with condition of small than 10
    {
        return ;
    }


    for (int i=0 ; i<PointSite.size() ; i++) //finding the furthest points
        for (int j=i+1; j<PointSite.size() ; j++){
           double D = sqrt( pow(PointSite[i].x-PointSite[j].x,2) + pow(PointSite[i].y-PointSite[j].y,2) );
           if (D>maxDistance){
                 side1=cv::Point2d(PointSite[i].x,PointSite[i].y);
                 side2=cv::Point2d(PointSite[j].x,PointSite[j].y);
                 maxDistance=D;

           }
        }

    Line LineA=_BuildLine(side1,side2);  // using farthest two points compute line

    cv::Point2d farthestPoint;


    // find farthest point
    bool findPoint=_FindTheFarthestPoint(LineA,PointSite,farthestPoint);


    if(findPoint==false){

        Line temp;
        _PointsetsCalLinePolarParameter(PointSite,temp);
        lineFeature.push_back(temp);

        std::cout << "PointSite.size() = " << PointSite.size() << std::endl;
        for (int i = 0; i < PointSite.size(); i++)
        {
            pcl::PointXYZRGB point;
            point.x = PointSite.at(i).x;
            point.y = PointSite.at(i).y;
            point.z = 0;

            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            // pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
            cloud->points.push_back(point);
        }
        return;
    }
    std::vector<cv::Point2d> RightPointSite,LeftPointSite;
    RightPointSite.reserve(1000);
    LeftPointSite.reserve(1000);

    _SeparatePoints(LineA,farthestPoint,PointSite,LeftPointSite,RightPointSite);



    SplitAndMerge(LeftPointSite, lineFeature, cloud);
    SplitAndMerge(RightPointSite, lineFeature, cloud);

}

void LineExtraction::LineRhoThetaExtraction(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, cv::Point2d &para)
{
    std::cout << "cloud.size() = " << cloud->points.size() << std::endl;

    // mean
    double mean_1_x = 0.0;
    double mean_1_y = 0.0;
    int count = 1;

    double varxy_1 = 0.0;
    double varxx_1 = 0.0;



    for (int i = 0; i < cloud->points.size(); i++)
    {
        mean_1_x = mean_1_x + (cloud->points.at(i).x - mean_1_x)/count;
        mean_1_y = mean_1_y + (cloud->points.at(i).y - mean_1_y)/count;
        count++;



    }
    count = 1;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        varxy_1 = varxy_1 + (cloud->points.at(i).x - mean_1_x)*(cloud->points.at(i).y - mean_1_y)/count;
        varxx_1 = varxx_1 + (cloud->points.at(i).x - mean_1_x)*(cloud->points.at(i).x - mean_1_x)/count;
    }

    // y - y_bar = m(x - x_bar)
    // to ax + by = c
    double m1 = varxy_1/varxx_1;
//    double a =  (-1)*m1;
//    double b = 1;
//    double c = mean_1_y - m1*mean_1_x;

    // rho = c/(a*a + b*b)
    // theta = acos(a/(a*a + b*b))
    // use the mean_x, mean_y, varxy, varxx bring into equation
    // costheta = -varxy/sqrt(varxy*varxy + varxx*varxx)
    // rho = abs(varxx*mean_y - varxy*mean_x)/sqrt(varxy*varxy + varxx*varxx)

//    std::cout << "varxy = " << varxy_1 << std::endl;
//    std::cout << "meanx = " << mean_1_x << ", meany = " << mean_1_y << std::endl;
    std::cout << "m = " << m1 << std::endl;

    para.x = abs(varxx_1*mean_1_y - varxy_1*mean_1_x)/sqrt(varxy_1*varxy_1 + varxx_1*varxx_1);
//    std::cout << "cos value = " << -varxy_1/sqrt(varxy_1*varxy_1 + varxx_1*varxx_1) << std::endl;
    double cosvalue = -varxy_1/sqrt(varxy_1*varxy_1 + varxx_1*varxx_1);
    para.y = acos(cosvalue);

    std::cout << "para.y = " << para.y << std::endl;
    // ??
//    if (cosvalue < 0)
//    {
//        std::cout << "cos < 0, pre_para.y = " << para.y << std::endl;
//        para.y = para.y - CV_PI;
//    }

//    if ()
//    para.x = abs(c)/sqrt(a*a + b*b);
//    para.y = acos(a/sqrt(a*a + b*b));
}

void LineExtraction::GetExtractionLinePara(const std::vector<cv::Point2d> &LaserCatesianPoints, cv::Point2d &para, bool IsShow)
{
    // Line Ransac
    const double ransacDistanceThreshold = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    // point cloud for show
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (IsShow)
        show_cloud->points.push_back(pcl::PointXYZRGB(0, 255, 0));

    for (int i = 0; i < LaserCatesianPoints.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = LaserCatesianPoints.at(i).x;
        point.y = LaserCatesianPoints.at(i).y;
        point.z = 0;
        cloud_copy->points.push_back(point);

        if (IsShow)
        {
            pcl::PointXYZRGB point2;
            point2.x = LaserCatesianPoints.at(i).x;
            point2.y = LaserCatesianPoints.at(i).y;
            point2.z = 0;

            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            // pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point2.rgb = *reinterpret_cast<float*>(&rgb);
            show_cloud->points.push_back(point2);
        }
    }
    //  created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
            model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_copy));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    std::vector<int> inliers;
    ransac.setDistanceThreshold (ransacDistanceThreshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_copy, inliers, *final);

    if (IsShow)
    {
        for (int i = 0; i < final->points.size(); i++)
        {
            pcl::PointXYZRGB point2;
            point2.x = final->points.at(i).x;
            point2.y = final->points.at(i).y;
            point2.z = final->points.at(i).z;

            uint8_t r = 255;
            uint8_t g = 0;
            uint8_t b = 0;
            // pack r/g/b into rgb
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            point2.rgb = *reinterpret_cast<float*>(&rgb);
            show_cloud->points.push_back(point2);
        }

        pcl::visualization::CloudViewer viewer("Simple Viewer");
        viewer.showCloud(show_cloud);
        while(!viewer.wasStopped())
        {
        }
    }
    LineExtraction::LineRhoThetaExtraction(final, para);
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

        //
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
//    double a = cos(temp.y), b = sin(temp.y);
//    xx=xx*0.3+lineImg.cols/2.0;
//    yy=yy*0.3+lineImg.rows/2.0;  // for show purpose



//    pt1.x = cvRound(xx + 1000*(-b)); //cvRound double to int
//    pt1.y = cvRound(yy + 1000*(a));
//    pt2.x = cvRound(xx - 1000*(-b));
//    pt2.y = cvRound(yy - 1000*(a));

//    cv::line( lineImg, pt1, pt2, cv::Scalar(0,255,0), 0.5, CV_AA);

//    cv::imshow("lineImg", lineImg);
//    cv::waitKey(1);

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
