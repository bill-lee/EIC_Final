#include "shih_slam/shih_myrobot.h"


MyRobot::MyRobot(double width,int velocity,int num,int s,double r,double l,double w,const cv::Point2d& point):
    robotFrameWidth(width)/*unit:cm*/,robotVelocity(velocity),allSteps(num),scenesCnt(s),rightOdeValue(r),leftOdeValue(l)
  ,robotWidth(w),robotMapOrginal(point)
{
    //generate motor pointer
    rightMotor=new MotorController;
    leftMotor=new MotorController;
    stepMotor=new MotorController;
    //set motor type
    rightMotor->SetMotorType(speedMode);
    leftMotor->SetMotorType(speedMode);
    stepMotor->SetMotorType(angleMode);

    //generate laser s200 pointer
    laserS200=new LaserControllerS200;

    //generate laser lms291 pointer
    laserLms291=new LaserController;

    webcam=new cv::VideoCapture;

    gridMapper=OccupancyGridMapping(800,800,2,1,3000,(double)CV_PI*(3.0/2.0),0.05);  //0.08
    gridMapper.InitGridMap();

    /////////////////////////////////////////////////
    //set camera parameters
    /////////////////////////////////////////////////
    cv::FileStorage loadFile("camera_parameters.yml", cv::FileStorage::READ);  //test  test0120

    if(!loadFile.isOpened()){
        std::cerr<<"can't open file"<<std::endl;
        return;
    }
    //load camera matrix
    loadFile["camera_matrix"]>>cameraIntrinsicMatrix;
    loadFile["distortion_coefficients"]>>cameradistortionCoefficients;

    loadFile.release();

    /////////////////////////////////////////////////
    //set rotation and translation matrix
    /////////////////////////////////////////////////
    //intial matrix
    laser_cameraTranslationMatrix=cv::Mat::zeros(3,1,CV_64F);
    laser_cameraRotationMatrix=cv::Mat::eye(3,3,CV_64F);
    laser_cameraHomogeneous=cv::Mat::eye(4,4,CV_64F);

    laser_cameraTranslationMatrix.ptr<double>(0)[0]=  -1.1885  ;  //x  cm
    laser_cameraTranslationMatrix.ptr<double>(1)[0]= 16.8503;  //y  0.6585
    laser_cameraTranslationMatrix.ptr<double>(2)[0]=  2.0909;  //z


    cv::Mat r_vec(3, 1, CV_64F);
    r_vec.ptr<double>(0)[0]= 1.5713;
    r_vec.ptr<double>(1)[0]= 0.0235;
    r_vec.ptr<double>(2)[0]= 0.0269;
    cv::Rodrigues(r_vec, laser_cameraRotationMatrix);


    cv::Mat copy=laser_cameraHomogeneous(cv::Rect(0,0,3,3));
    laser_cameraRotationMatrix.copyTo(copy);  //insert to homogeneous matrix
    copy=laser_cameraHomogeneous(cv::Rect(3,0,1,3));
    laser_cameraTranslationMatrix.copyTo(copy);

    /////////////////////////////////////////////////
    //set parameters
    /////////////////////////////////////////////////

    //robotMapOrginal=cv::Point2d (150,500);
    gridMapper.SetGridMapOriginalPoint(robotMapOrginal);

    robotPose.x=0;
    robotPose.y=0;
    robotPose.z=0;

    deltaT=1;


    lineExtracter=LineExtraction(20);
    refenceMap.reserve(100000);  //預先開設N筆資料點空間

}
MyRobot::~MyRobot()
{
   rightMotor->Stop();
   leftMotor->Stop();
   stepMotor->Stop();




   delete rightMotor;
   delete leftMotor;
   delete stepMotor;
   delete laserS200;
   delete laserLms291;
   delete webcam;

}
void MyRobot::SaveRawScanSets(const std::string &name)
{

    std::ofstream file;
    file.open(name);

    if(!file)
    {
        std::cerr<<"Check .txt file"<<std::endl;

    }


    for(int i=0;i!=rawScanSets.size();++i)
    {
        for(int j=0;j!=rawScanSets[i].size();++j)
        {
            double tempValue=rawScanSets[i][j];
             file<<tempValue<<" ";

        }
        file<<endl;
    }

    file.close();


}

void MyRobot::SetOdeValue(const double r, const double l)
{

    rightOdeValue=r;
    leftOdeValue=l;

}

void MyRobot::TrajectoryGeneration()
{

    if(robotPathSets.size()==0)
        return;


    double preangle=0;
    //前進:1  2:右轉 3:左轉
    for(int i=1;i!=robotPathSets.size();++i)
    {

        double tempx=robotPathSets[i].x-robotPathSets[i-1].x;
        double tempy=robotPathSets[i].y-robotPathSets[i-1].y;

        double r=sqrt(pow(tempx,2.0)+pow(tempy,2.0));
        double angle=atan2(tempy,tempx);


        double commandAngle=angle-preangle;
        std::pair<int,double> temp;
        //command: angle
        if(commandAngle<0)
        { //+ 左轉

            temp.first=2;  //
            temp.second=commandAngle*180.0/CV_PI;
                commandSets.push(temp);
        }
        else if(commandAngle>0)

        { //- 右轉
            temp.first=3;  //
            temp.second=commandAngle*180.0/CV_PI;
            commandSets.push(temp);
        }
        /*
        else
        {  //若無轉動 則假設不動
            temp.first=1;  //
            temp.second=0;
            commandSets.push(temp);
        }
        */

        //command: forward
        temp.first=1;  //grid map 大小決定每格走的距離
        temp.second=r*gridMapper.GetPixel_meterFactor()*100;  //m->cm
        commandSets.push(temp);

        //cout<<r<<" "<<angle-preangle<<endl;
        preangle=angle;

    }





}

void MyRobot::MotionModel()
{
    double phi=robotPose.z;


    double tempX=(leftOdeValue+rightOdeValue)/2.0*cos(phi+(leftOdeValue-rightOdeValue)*deltaT/(2*robotWidth));
    double tempY=(leftOdeValue+rightOdeValue)/2.0*sin(phi+(leftOdeValue-rightOdeValue)*deltaT/(2*robotWidth));
    double tempThtea=((leftOdeValue-rightOdeValue)/robotWidth)*deltaT;

    //robot state
    robotPose.x+=tempX;
    robotPose.y+=tempY;
    robotPose.z+=tempThtea;

    // cout<<"robot pose:"<<robotPose.x<<" "<<robotPose.y<<" "<<robotPose.z<<endl;

}

void MyRobot::FindCurrentNodeEnd(const cv::Mat &gridMap,double intervalDistance, const cv::Point2d &currentStart,const cv::Point2d &goal ,cv::Point2d& currentEnd)
{
    double min=10000;
    cv::Point2d nodeTemp;

    nodeTemp.x=currentStart.x+10;
    nodeTemp.y=currentStart.y;

    cout<<"currentStart"<<currentStart<<" "<<gridMap.cols<<gridMap.rows<<endl;
    cout<<"FinalEnd"<<goal.x<<" "<<goal.y<<endl;
    int box_size=intervalDistance;  //128個pixel  每個pixel*gridSize
     /////////////需要限制圖片超過範圍的限制
     for(int i=0;i!=(box_size+1);++i)
         for(int j=-box_size;j!=(box_size+1);++j)
         {
             int x=i+currentStart.x;
             int y=j+currentStart.y;


             double temp=gridMap.ptr<uchar>(y)[x];
             if(temp==253)  //1:occ 127:unknow 253:free
             {

                 double suby=abs((double)goal.y-y);
                 double subx=abs((double)goal.x-x);
                 double r=sqrt(pow(subx,2.0)+pow(suby,2.0));
                   //cout<<r<<" "<<subx<<" "<<suby<<endl;
                 if(min>r)
                 {


                     nodeTemp.x=x;
                     nodeTemp.y=y;
                     min=r;
                 }

             }

         }
  //  return cv::Point2d(nodeTemp.x,nodeTemp.y);

    currentEnd.x=nodeTemp.x;
    currentEnd.y=nodeTemp.y;
}

void MyRobot::PathSmoothing()
{
    this->robotPathSetsSmoothing.clear();
    this->robotPathSetsSmoothing.reserve(this->robotPathSets.size());


  //  cv::Mat img2(1500,1500,CV_8UC3);
    //img2=cv::Scalar::all(255);

    double weight_data=0.5;
    double weight_smooth=0.5;
    double tolerance=0.001;
    std::vector<cv::Point2d> newPath;
    newPath.reserve(this->robotPathSets.size());


    for(int i=0;i!=this->robotPathSets.size();++i)
    {
        newPath.push_back(this->robotPathSets[i]);
        //cout<<this->robotPathSets[i]<<endl;
    }

/*
    cv::circle(img2, cv::Point(this->robotPathSets[0].x+100,this->robotPathSets[0].y+400), 4, cv::Scalar(0,0,255), 2  );

    for(int i=1;i!=this->robotPathSets.size();++i)
    {
    cv::line( img2, cv::Point(this->robotPathSets[i-1].x+100,this->robotPathSets[i-1].y+400), cv::Point(this->robotPathSets[i].x+100,this->robotPathSets[i].y+400), cv::Scalar(0,0,255), 4,CV_AA);
    cv::circle(img2, cv::Point(this->robotPathSets[i].x+100,this->robotPathSets[i].y+400), 4, cv::Scalar(0,0,255), 2  );
     }
    cout<<"======================"<<endl;
*/

    double change =tolerance;



    while(change>=tolerance)
    {
        change=0;
        for(int i=1;i!=newPath.size()-1;++i)
        {
            double  x=newPath[i].x;
            double  y=newPath[i].y;

            newPath[i].x+=weight_data*(this->robotPathSets[i].x-newPath[i].x);
            newPath[i].x+=weight_smooth*(newPath[i-1].x+newPath[i+1].x-(2*newPath[i].x));
            change+=abs(x-newPath[i].x);

            newPath[i].y+=weight_data*(this->robotPathSets[i].y-newPath[i].y);
            newPath[i].y+=weight_smooth*(newPath[i-1].y+newPath[i+1].y-(2*newPath[i].y));
            change+=abs(y-newPath[i].y);

        }

    }

    for(int i=0;i!=newPath.size();++i)
    {
       // cout<<newPath[i]<<endl;
        this->robotPathSetsSmoothing.push_back(newPath[i]);
    }
/*
    cv::circle(img2, cv::Point(newPath[0].x+100,newPath[0].y+400), 4, cv::Scalar(0,0,255), 2  );

    for(int i=1;i!=newPath.size();++i)
    {
        cv::line( img2, cv::Point(newPath[i-1].x+100,newPath[i-1].y+400), cv::Point(newPath[i].x+100,newPath[i].y+400), cv::Scalar(0,255,0), 4,CV_AA);
        cv::circle(img2, cv::Point(newPath[i].x+100,newPath[i].y+400), 7, cv::Scalar(0,255,0), 2  );
        cv::circle(img2, cv::Point(robotPathSetsSmoothing[i].x+100,robotPathSetsSmoothing[i].y+400), 4, cv::Scalar(0,255,0), 2  );
     }

    cv::imshow(":123123",img2);
    cv::waitKey(1);

    */
}

void MyRobot::TrajectoryGenerationSmoothing()
{

    if(robotPathSetsSmoothing.size()==0)
        return;


    double preangle=0;
    //前進:1  2:右轉 3:左轉
    for(int i=1;i!=robotPathSetsSmoothing.size();++i)
    {

        double tempx=robotPathSetsSmoothing[i].x-robotPathSetsSmoothing[i-1].x;
        double tempy=robotPathSetsSmoothing[i].y-robotPathSetsSmoothing[i-1].y;

        double r=sqrt(pow(tempx,2.0)+pow(tempy,2.0));
        double angle=atan2(tempy,tempx);


        double commandAngle=angle-preangle;
        std::pair<int,double> temp;
        //command: angle
        if(commandAngle<-0.0175)  //1度~-1度內不考慮
        { //+ 左轉

            temp.first=2;  //
            temp.second=commandAngle*180.0/CV_PI;
                commandSets.push(temp);
        }
        else if(commandAngle>0.0175)

        { //- 右轉
            temp.first=3;  //
            temp.second=commandAngle*180.0/CV_PI;
            commandSets.push(temp);
        }
        /*
        else
        {  //若無轉動 則假設不動
            temp.first=1;  //
            temp.second=0;
            commandSets.push(temp);
        }
        */

        //command: forward
        temp.first=1;  //grid map 大小決定每格走的距離
        temp.second=(int)(r*gridMapper.GetPixel_meterFactor()*100);  //m->cm
        commandSets.push(temp);

        //cout<<r<<" "<<angle-preangle<<endl;
        preangle=angle;

    }






}


