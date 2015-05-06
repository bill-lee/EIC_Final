#include "myrobot.h"

lab405::MyRobot::MyRobot(double w, int velocity, const cv::Point2d &point) :
    step_motor(new StepMotor_Controller()),
    laser_lms291(new Laser_LMS291_Controller),
    laser_slam(new Laser_LMS291_Controller),
    lasercount(0), data_start(false), is_tigger(false), with_color(true),
    completeimage(false), completelaser(false),
    right_dcmotor(new DCMotor_Controller), left_dcmotor(new DCMotor_Controller),
    offlineSLAM_count(0), odo_cmd_count(0),
    robotWidth(w), robotVelocity(velocity), robotMapOrginal(point)
{
    // establish opencv file stream
    cv::FileStorage loadFile("./camera_calibration.yml", cv::FileStorage::READ);
    if(!loadFile.isOpened()){
        std::cerr << "can't open camera calibration file" << std::endl;
    }
    // load camera matrix
    loadFile["camera_matrix"] >> cameraIntrinsicMatrix;
    loadFile["distortion_coefficients"] >> distortionCoefficients;
    loadFile.release();
    //--
//    webcam = new cv::VideoCapture;

    gridMapper=OccupancyGridMapping(800,800,2,1,3000,(double)CV_PI*(3.0/2.0),0.05);  //0.08
    gridMapper.InitGridMap();

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

    int a = 0;

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

//    robotPose.x=0;
//    robotPose.y=0;
//    robotPose.z=0;

//    deltaT=1;


    lineExtracter=LineExtraction(20);
    refenceMap.reserve(100000);  // reserve N data space
}

lab405::MyRobot::~MyRobot()
{
    if (laser_lms291->isOpen())
        laser_lms291->close();
    if (step_motor->isOpen())
        step_motor->close();
    if (right_dcmotor->isOpen())
        right_dcmotor->close();
    if (left_dcmotor->isOpen())
        left_dcmotor->close();
}

void lab405::MyRobot::DataAcquisition(double start_angle, int _scan_count, double end_angle, QString filename, const bool _with_color)
{
    if (this->laser_lms291->isOpen() && this->step_motor->isOpen())
    {
        lasercount = 0;
        // open notify command
        this->step_motor->OpenANSWMode();
        // go to the initial position, emit one position attained
        this->step_motor->RotateAbsoluteAngle(start_angle);
        // enable flag of data acquisition
        data_start = true;
        std::cout << "laser count = " << _scan_count << std::endl;
        // assign member: scancount, rot_ang value
        scancount = _scan_count;
        rot_ang = (end_angle - start_angle) /(_scan_count - 1);
        std::cout << "rot_ang = " << rot_ang << std::endl;
        // assian member: filename
        data_filename = QString("./") + filename + QString(".txt");
        std::cout << "filename = " << data_filename.toStdString() << std::endl;



//        connect(this->step_motor, SIGNAL(PositionAttained()),
//                this, SLOT(ContiTigger()));
//        connect(this->laser_lms291, SIGNAL(SuccessTrigger()),
//                this, SLOT(StopContiTrigger()));

        // assign with color flag
        with_color = _with_color;
        // create image folder
        image_foldername = QString("./") + filename;
        if(!QDir(image_foldername).exists()) //check folder exist
            QDir().mkdir(image_foldername);
        // open camera
        if(!capture.isOpened())
        {
            capture.open(0);
        }
        if (capture.isOpened())
            std::cout << "Camera is opened!" << std::endl;
        //        connect(this->step_motor, SIGNAL(PositionAttained()),
        //                this->laser_lms291, SLOT(TriggerOneScan()));

        // connect position attained to laser nad camera
        connect(this->step_motor, SIGNAL(PositionAttained()),
                this, SLOT(RangeAndColorDataAcquisition()));

        // for allowing boost::shared_ptr legal for connect
        qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
        connect(this->laser_lms291, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
                this, SLOT(PushDataToBufferThread(boost::shared_ptr<QByteArray>)));
        connect(this, SIGNAL(CompleteLaser()),
                this, SLOT(CheckFinishedDataAcquisition()));
        if (with_color)
            connect(this, SIGNAL(CompleteTakingImage()),
                    this, SLOT(CheckFinishedDataAcquisition()));
//        connect(this, SIGNAL(CompleteOneScan()),
//                this, SLOT(RotateToNextScan()));

//        this->step_motor->response.clear();
//        connect(this->step_motor, SIGNAL(readyRead()), this->step_motor, SLOT(Response()));

        // connect step motor response
//        this->laser_lms291->TriggerOneScan();

//        for (int i = 0; i < laser_count; i++)
//        {

//            while (!this->laser_lms291->GetOneScan())
//            {
//            }
//            //*** return step motor stop event
//            //


//            this->step_motor->RotateRelativeAngle(rot_ang);
////            Sleep(500);
//            std::cout << i << std::endl;

//        }
    }
}

void lab405::MyRobot::DataAcquisitionConti(double _start_angle,
                                   int _scan_count,
                                   double _end_angle,
                                   QString filename,
                                   const bool _with_color,
                                   const int cam_num)
{
    if (this->laser_lms291->isOpen() && this->step_motor->isOpen())
    {
        completeimage = false;
        completelaser = false;
        // whole time
        wholetime.start();

        databuffer.clear();
        lasercount = 0;
        // enable flag of data acquisition
        data_start = true;
        std::cout << "laser count = " << _scan_count << std::endl;
        // assign member: scancount, start_angle, and end_angle value
        scancount = _scan_count;
        start_angle = _start_angle;
        end_angle = _end_angle;

        rot_ang = (_end_angle - _start_angle) /(_scan_count - 1);
        std::cout << "rot_ang = " << rot_ang << std::endl;
        // assian member: filename
        data_filename = QString("./") + filename + QString(".txt");
        std::cout << "filename = " << data_filename.toStdString() << std::endl;

        // assign with color flag
        with_color = _with_color;
        // create image folder
        image_foldername = QString("./") + filename;
        if(!QDir(image_foldername).exists()) //check folder exist
            QDir().mkdir(image_foldername);
        // open camera

        capture.open(cam_num);

        if (capture.isOpened())
        {
            std::cout << "Camera is opened!" << std::endl;
            emit CameraOpened();
        }

        // open notify command
        this->step_motor->OpenANSWMode();
        // go to the initial position, emit one position attained
        this->step_motor->RotateAbsoluteAngle(_start_angle);
        // trigger continuous laser
        this->laser_lms291->TriggerContinuousMode();
    }
}

void lab405::MyRobot::ConnectDataAcquisition(bool _with_color)
{
    if (this->laser_lms291->isOpen() && this->step_motor->isOpen())
    {
        // connect position attained to laser nad camera
        connect(this->step_motor, SIGNAL(PositionAttained()),
                this, SLOT(RangeAndColorDataAcquisitionConti()));

        // for allowing boost::shared_ptr legal for connect
        qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
        connect(this->laser_lms291, SIGNAL(GetContiOneScan(boost::shared_ptr<QByteArray>)),
                this, SLOT(PushDataToBufferThread(boost::shared_ptr<QByteArray>)));

        // for allowing boost::shared_ptr legal for connect
        qRegisterMetaType<boost::shared_ptr<cv::Mat>> ("boost::shared_ptr<cv::Mat>");
        connect(this, SIGNAL(ImageReadyToSave(boost::shared_ptr<cv::Mat>)),
                this, SLOT(SaveImageThread(boost::shared_ptr<cv::Mat>)));

        // for connect laser and camera events
        connect(this, SIGNAL(CompleteLaser()),
                this, SLOT(CheckFinishedDataAcquisition()));
        if (_with_color)
            connect(this, SIGNAL(CompleteTakingImage()),
                    this, SLOT(CheckFinishedDataAcquisition()));
    }
}

void lab405:: MyRobot::DisconnectDataAcquisition(bool _with_color)
{
    if (this->laser_lms291->isOpen() && this->step_motor->isOpen())
    {
        // connect position attained to laser nad camera
        disconnect(this->step_motor, SIGNAL(PositionAttained()),
                this, SLOT(RangeAndColorDataAcquisitionConti()));

        // for allowing boost::shared_ptr legal for connect
        qRegisterMetaType<boost::shared_ptr<QByteArray>> ("boost::shared_ptr<QByteArray>");
        disconnect(this->laser_lms291, SIGNAL(GetContiOneScan(boost::shared_ptr<QByteArray>)),
                this, SLOT(PushDataToBufferThread(boost::shared_ptr<QByteArray>)));

        // for allowing boost::shared_ptr legal for connect
        qRegisterMetaType<boost::shared_ptr<cv::Mat>> ("boost::shared_ptr<cv::Mat>");
        disconnect(this, SIGNAL(ImageReadyToSave(boost::shared_ptr<cv::Mat>)),
                this, SLOT(SaveImageThread(boost::shared_ptr<cv::Mat>)));

        // for connect laser and camera events
        disconnect(this, SIGNAL(CompleteLaser()),
                this, SLOT(CheckFinishedDataAcquisition()));
        if (_with_color)
            disconnect(this, SIGNAL(CompleteTakingImage()),
                    this, SLOT(CheckFinishedDataAcquisition()));
    }
}

void lab405::MyRobot::StopDataAcquisition()
{   
    disconnect(this->step_motor, SIGNAL(PositionAttained()),
            this, SLOT(RangeAndColorDataAcquisition()));
    disconnect(this->laser_lms291, SIGNAL(GetOneLaserScan(boost::shared_ptr<QByteArray>)),
            this, SLOT(PushDataToBufferThread(boost::shared_ptr<QByteArray>)));
    disconnect(this, SIGNAL(CompleteOneScan()),
               this, SLOT(CheckFinishedDataAcquisition()));
    if (with_color)
        disconnect(this, SIGNAL(CompleteTakingImage()),
                this, SLOT(CheckFinishedDataAcquisition()));
}

void lab405::MyRobot::StopDataAcquisitionConti()
{
//    completeimage = false;
//    completelaser = false;
    this->laser_lms291->StopContinuousMode();
    this->laser_lms291->DisDataSegment();
}

void lab405::MyRobot::GoForward(double distance_m, double speed)
{
    right_dcmotor->SetMaxVelocity(speed);
    left_dcmotor->SetMaxVelocity(-speed);

    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // calibration coeffient: y = 0.9487x
    int value = static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi)/0.9487);
    std::cout << "value = " << value << std::endl;
    right_dcmotor->RotateRelativeDistancce(value);
    left_dcmotor->RotateRelativeDistancce(-value);

    emit EmitOdoMeter(Odotype::Forward, value);
}

void lab405::MyRobot::GoBackward(double distance_m, double speed)
{
    right_dcmotor->SetMaxVelocity(-speed);
    left_dcmotor->SetMaxVelocity(speed);

    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // calibration coeffient: y = 0.9487x
    int value = static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi)/0.9487);
    std::cout << "value = " << value << std::endl;
    right_dcmotor->RotateRelativeDistancce(-value);
    left_dcmotor->RotateRelativeDistancce(value);

    emit EmitOdoMeter(Odotype::Backward, value);
}

void lab405::MyRobot::TurnLeft(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->SetMaxVelocity(speed);
        left_dcmotor->SetMaxVelocity(speed);

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487);
        right_dcmotor->RotateRelativeDistancce(value);
        left_dcmotor->RotateRelativeDistancce(value);

        emit EmitOdoMeter(Odotype::Leftward, value);
    }
}

void lab405::MyRobot::TurnRight(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->SetMaxVelocity(-speed);
        left_dcmotor->SetMaxVelocity(-speed);

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487);
        right_dcmotor->RotateRelativeDistancce(-value);
        left_dcmotor->RotateRelativeDistancce(-value);

        emit EmitOdoMeter(Odotype::Rightward, value);
    }
}

void lab405::MyRobot::test()
{

    this->capture.open(0);
    if (capture.isOpened())
        std::cout << "Camera is opened!" << std::endl;
    lasercount = 0;

//    this->rotatethread();
//    this->rotatethread();

//    cv::VideoCapture capture(0);
//    std::vector<cv::Mat> imgs;
//    cv::Mat temp;
//    const int size = 1000;
//    for (int i = 0; i < size; i++)
//    {
//        capture >> temp;
//        imgs.push_back(temp);
//    }

//    for (int i = 0; i < size; i++)
//    {
//        std::string filename = "./0317test/" + std::to_string(i) + ".jpg";
//        cv::imwrite(filename, imgs.at(i));
//    }
//    std::cout << "done" << std::endl;
}

void lab405::MyRobot::rotatethread()
{
    QFuture<void> future = QtConcurrent::run(this, &MyRobot::rotate);
    future.waitForFinished();
    //    while (future.isRunning()) {}
}

void lab405::MyRobot::SaveOfflineSLAMThread(boost::shared_ptr<QByteArray> scan)
{
    QFuture<void> future = QtConcurrent::run(this, &MyRobot::SaveOfflineSLAM, scan);
    future.waitForFinished();
}

void lab405::MyRobot::SaveOdoMeter(Odotype type, int value)
{
    // first increment, for matching with laser
    odo_cmd_count++;
    std::string filename;
    std::ofstream ofile;
    switch (type) {
    case Odotype::Forward:
        filename = "./" + odo_filename + "_" + std::to_string(odo_cmd_count) + ".txt";
        ofile.open(filename);
        ofile << "for" << std::endl;
        ofile << "right: " << value << " left: " << -value;
        break;
    case Odotype::Backward:
        filename = "./" + odo_filename + "_" + std::to_string(odo_cmd_count) + ".txt";
        ofile.open(filename);
        ofile << "bac" << std::endl;
        ofile << "right: " << -value << " left: " << value;
        break;
    case Odotype::Leftward:
        filename = "./" + odo_filename + "_" + std::to_string(odo_cmd_count) + ".txt";
        ofile.open(filename);
        ofile << "left" << std::endl;
        ofile << "right: " << value << " left: " << value;
        break;
    case Odotype::Rightward:
        filename = "./" + odo_filename + "_" + std::to_string(odo_cmd_count) + ".txt";
        ofile.open(filename);
        ofile << "rig" << std::endl;
        ofile << "right: " << -value << " left: " << -value;
        break;
    }

}

void lab405::MyRobot::ProcessSingleTriggerSLAM(boost::shared_ptr<QByteArray> scan)
{
//    CornerExtraction cornerEx;

//    vector<double> temp;
//    std::vector<cv::Point2d> temp1;
//    std::vector<Line> temp2;
//    std::vector<Corner> cor;

//    for (int i = 0; i < scan->size(); i += 2)
//    {
//        temp.push_back(double(uint16_t((scan->at(i) & 0xff) | ((scan->at(i + 1) & 0x1f) << 8))));
//    }

//    if(temp.size()==0/*||i==0*/)
//        return;
//    this->mapper.RangesDataToPointsData(temp, temp1);

//    this->lineExtracter.SplitAndMerge(temp1, temp2);
//    cornerEx.ExtractCorners(temp,cor);

//    std::cout << "laser points:" << temp.size() << "  line num:" << temp2.size() << "  cor num:" << cor.size() << std::endl;
//    cv::Mat img = this->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

//    cv::Mat img1 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
//    cv::Mat img2 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


//    lines.push_back(temp2);
//    this->gridMapper.InsertLocalGridMap(temp, cv::Point3d(0,0,0));

//    //////////////////////////////////////////////
//    //save file
//    saveFileIndex++;
//    robotOutputFile << 0 << " " << 0 << endl;  //encoder
//    for(int i = 0; i != temp.size(); ++i)
//        laserOutputFile << temp[i] << " ";

//    laserOutputFile << endl;//´«¦æ


//    //////////////////////////////////////////////
//    //for(int i=0;i!=temp.size();++i)
//    // {
//    //  if(temp[i]<=2996)
//    //      cout<<temp[i]<<endl;
//    //}

//    //  cv::Mat tt;
//    // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
//    // cv::imshow("tt",tt);
//    cv::imshow("1341",img);
//    cv::imshow("13411",img1);
//    cv::imshow("13411222 ",cornerEx.cornerImg);
//    cv::waitKey(10);

}


void lab405::MyRobot::SaveOfflineSLAMFile(const std::string &filename)
{
    std::ofstream ofile (filename);

    std::cout << "temp_SLAM size = " << temp_SLAM.size() << std::endl;
    int count = 0;
    for (int i = 0; i < temp_SLAM.size(); i += 2)
    {
        if (i % 722 != 0)
            ofile << std::dec << uint16_t((temp_SLAM.at(i) & 0xff) | ((temp_SLAM.at(i + 1) & 0x1f) << 8)) << " ";
        else
        {
            count++;
            std::cout << "ok" << std::endl;
            if (i != 0)
                ofile << std::endl;
            ofile << count << ": ";
        }
    }


}

void lab405::MyRobot::FindCurrentNodeEnd(const cv::Mat &gridMap, double intervalDistance, const cv::Point2d &currentStart, const cv::Point2d &goal, cv::Point2d &currentEnd)
{
    double min=10000;
    cv::Point2d nodeTemp;

    nodeTemp.x=currentStart.x+10;
    nodeTemp.y=currentStart.y;

    cout<<"currentStart"<<currentStart<<" "<<gridMap.cols<<gridMap.rows<<endl;
    cout<<"FinalEnd"<<goal.x<<" "<<goal.y<<endl;
    int box_size=intervalDistance;  //128 pixels, each pixel*gridsize
     ///////////// picture oversize constraint
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

void lab405::MyRobot::PathSmoothing()
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

void lab405::MyRobot::TrajectoryGenerationSmoothing()
{
    if(robotPathSetsSmoothing.size()==0)
        return;


    double preangle=0;
    // 1: go forward, 2: turn right, 3: turn left
    for(int i=1;i!=robotPathSetsSmoothing.size();++i)
    {

        double tempx=robotPathSetsSmoothing[i].x-robotPathSetsSmoothing[i-1].x;
        double tempy=robotPathSetsSmoothing[i].y-robotPathSetsSmoothing[i-1].y;

        double r=sqrt(pow(tempx,2.0)+pow(tempy,2.0));
        double angle=atan2(tempy,tempx);


        double commandAngle=angle-preangle;
        std::pair<int,double> temp;
        //command: angle
        if(commandAngle<-0.0175)  // not consider -1 degree ~ 1 degree
        { //+ turn left

            temp.first=2;  //
            temp.second=commandAngle*180.0/CV_PI;
                commandSets.push(temp);
        }
        else if(commandAngle>0.0175)

        { //- turn right
            temp.first=3;  //
            temp.second=commandAngle*180.0/CV_PI;
            commandSets.push(temp);
        }
        /*
        else
        {  // if no rotate, assume no move
            temp.first=1;  //
            temp.second=0;
            commandSets.push(temp);
        }
        */

        //command: forward
        temp.first=1;  //grid map size determine the distance of each grid
        temp.second=(int)(r*gridMapper.GetPixel_meterFactor()*100);  //m->cm
        commandSets.push(temp);

        //cout<<r<<" "<<angle-preangle<<endl;
        preangle=angle;

    }
}

void lab405::MyRobot::rotate()
{
    this->step_motor->RotateRelativeAngle(360);
}

void lab405::MyRobot::SpinData()
{
    capture.open(0);
    if (capture.isOpened())
        std::cout << "Camera is opened" << std::endl;

    this->laser_lms291->DataSegment();
    this->laser_lms291->TriggerContinuousMode();

}

void lab405::MyRobot::ConnectSpinData()
{
    if (this->laser_lms291->isOpen() && this->step_motor->isOpen())
    {
        connect(laser_lms291, SIGNAL(HeaderOK()), this, SLOT(TakePictureThread()));
        qRegisterMetaType<boost::shared_ptr<cv::Mat>> ("boost::shared_ptr<cv::Mat>");
        connect(this, SIGNAL(ImageReadyToSave(boost::shared_ptr<cv::Mat>)),
                this, SLOT(SaveImageThread(boost::shared_ptr<cv::Mat>)));
    }
}


void lab405::MyRobot::PushDataToBuffer(boost::shared_ptr<QByteArray> scan)
{
    int temp_count = lasercount;
    completelaser = true;
    emit CompleteLaser();


    QTime time;
    time.start();

    // for sake of progress bar display
    emit PushBufferSignal(temp_count);
    std::cout << std::dec << "temp_count: " << temp_count << std::endl;
    // append scan data to databuffer
    for (int i = 0; i < scan->size(); i++)
    {
        databuffer.append(scan->at(i));
    }

    std::cout << temp_count << " test size = " << databuffer.size() << std::endl;
    std::cout << "scancount = " << scancount - 1 << std::endl;
    // sum(0:scancount-1) = scancount
    if (temp_count == (scancount - 1))
    {
        StopDataAcquisitionConti();
        WriteData();
        emit FinishedDataAquisition(databuffer.size());
        std::cout << "whole time = " << wholetime.elapsed() << std::endl;
    }


    std::cout << "push to buffer time = " << time.elapsed() << std::endl;

}

void lab405::MyRobot::PushDataToBufferThread(boost::shared_ptr<QByteArray> scan)
{
    QFuture<void> future = QtConcurrent::run(this, &MyRobot::PushDataToBuffer, scan);
    future.waitForFinished();
}

void lab405::MyRobot::PushDataToBufferP(QByteArray *scan)
{
    std::cout << "lasercount: " << lasercount << std::endl;
    lasercount++;
//    buffer = boost::shared_ptr<QByteArray> (new QByteArray(buffer->append(*scan)));
//    if (lasercount == scancount)
//        StopDataReceive();
    delete scan;
//    emit CompleteOneScan();
}

void lab405::MyRobot::RotateToNextScan()
{
    is_tigger = false;
    completeimage = false;
    completelaser = false;
    std::cout << "rotate to next scan" << std::endl;
    this->step_motor->RotateRelativeAngle(rot_ang);
}

void lab405::MyRobot::StepMotorStatus()
{
    if (data_start)
    {
        this->laser_lms291->TriggerOneScan();
    }
    else
    {
    }
}

void lab405::MyRobot::StopContiTrigger()
{
    is_tigger = true;
}

void lab405::MyRobot::ContiTigger()
{
    while (!is_tigger)
    {
        Sleep(1000);
        this->laser_lms291->TriggerOneScan();
    }
}

void lab405::MyRobot::RangeAndColorDataAcquisition()
{
    if (with_color)
        TakePictureThread();
    this->laser_lms291->TriggerOneScan();
}

void lab405::MyRobot::RangeAndColorDataAcquisitionConti()
{
    emit DatasegmentSignal(lasercount);
    if (with_color)
        TakePictureThread();
    this->laser_lms291->DataSegment();
}

void lab405::MyRobot::CheckFinishedDataAcquisition()
{
    std::cout << "with color = " << with_color << ", laser = " << completelaser
              << ", cam = " << completeimage << std::endl;
    emit CheckFinishOneScan(lasercount, completelaser, completeimage);
    if ((!with_color || completeimage) && completelaser)
    {
        emit ProgressBarSignal(lasercount); 
        lasercount++;
        std::cout << "after lasercnt++" << lasercount << std::endl;
        RotateToNextScan();
    }
    //        emit CompleteOneScan();
}

void lab405::MyRobot::SaveImageThread(boost::shared_ptr<cv::Mat> ptr)
{
    QFuture<void> future = QtConcurrent::run(this, &MyRobot::SaveImage, ptr);
    future.waitForFinished();
}

void lab405::MyRobot::Test()
{
    std::cout << "Test" << std::endl;
}

void lab405::MyRobot::WriteData()
{
    std::cout << "buffer final size = " << databuffer.size() << std::endl;
    std::ofstream ofile(data_filename.toStdString().c_str(), std::ios_base::out);
    ofile << lasercount << std::endl;
    ofile << start_angle << std::endl;
    ofile << end_angle << std::endl;
    for (int i = 0; i < databuffer.size(); i += 2)
    {
        ofile << std::dec << uint16_t((databuffer.at(i) & 0xff) | ((databuffer.at(i + 1) & 0x1f) << 8)) << " ";
    }
}

void lab405::MyRobot::TakePictureThread()
{
    QFuture<void> future = QtConcurrent::run(this, &MyRobot::TakePicture);
    future.waitForFinished();
}

void lab405::MyRobot::TakePicture()
{
    boost::shared_ptr<cv::Mat> img_ptr (new cv::Mat());
    capture >> *img_ptr;  // capture image from camera
    if(img_ptr->empty())
    {
        std::cout << "Image Empty!" << std::endl;
        return;
    }
    completeimage = true;
    emit ImageReadyToSave(img_ptr);
    emit CompleteTakingImage();
}

void lab405::MyRobot::SaveImage(boost::shared_ptr<cv::Mat> ptr)
{
    int temp_count = lasercount;
//    if (temp_count < scancount)
//    {
        cv::Mat frame;
        QString imgName = image_foldername + QString("/%1.jpg").arg(lasercount);
        cv::undistort(*ptr, frame, cameraIntrinsicMatrix, distortionCoefficients, cameraIntrinsicMatrix );
        cv::imwrite(imgName.toStdString(), frame);
//    }
//    std::cout << imgName.toStdString() << std::endl;
    // 20150318 可刪
        //    lasercount++;
}

void lab405::MyRobot::SaveOfflineSLAM(boost::shared_ptr<QByteArray> scan)
{
    // append scan data to databuffer
//    for (int i = 0; i < scan->size(); i++)
//    {
//        temp_SLAM.append(scan->at(i));
//    }
//    emit OfflineSLAM_ScanSize(temp_SLAM.size());
//    std::cout << std::dec << "test size = " << temp_SLAM.size() << std::endl;

    const std::string filename = "./" + offlineSLAM_filename + "_" + std::to_string(offlineSLAM_count) + ".txt";

    std::ofstream ofile(filename);
    if (!ofile.is_open())
        return;
    ofile << odo_cmd_count << std::endl;
    for (int i = 0; i < scan->size(); i += 2)
    {
        ofile << std::dec << uint16_t((scan->at(i) & 0xff) | ((scan->at(i + 1) & 0x1f) << 8)) << " ";
    }

    offlineSLAM_count++;
}
