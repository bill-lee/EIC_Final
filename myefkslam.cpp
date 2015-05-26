#include "myefkslam.h"

lab405::MyEFKSLAM::MyEFKSLAM(double w, int velocity, const cv::Point2d &point, int num, double gl, double gc, double t, double n, int weight, double _Kr, double _Kl)
    : myrobot(new MyRobot()), EKFTimer(new QTimer),
      robotWidth(w), robotVelocity(velocity), robotMapOrginal(point),
      landmarkNum(num), gatingLine(gl), gatingCorner(gc),
      deltaT(t), newFeatureRadius(n), weighting(weight),
      Kr(_Kr), Kl(_Kl)
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

//    gridMapper = new OccupancyGridMapping(800, 800, 2, 1, 3000, CV_PI ,0.05);  //0.08
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

    connect(this->EKFTimer, SIGNAL(timeout()), this, SLOT(EKFStepExamine()));

    //n=20
    robotState.robotPositionMean=cv::Scalar::all(0);

    robotState.robotPositionCovariance=cv::Mat::eye(robotState.robotPositionCovariance.size(),robotState.robotPositionCovariance.type());
    robotState.robotPositionCovariance.ptr<double>(0)[0]=0.001;
    robotState.robotPositionCovariance.ptr<double>(1)[1]=0.001;
    robotState.robotPositionCovariance.ptr<double>(2)[2]=0.5;


    robotCombinedState=cv::Mat::zeros(STATE_SIZE+OBS_SIZE*landmarkNum,1,CV_64F);
    robotCombinedCovariance=cv::Mat::eye(STATE_SIZE+OBS_SIZE*landmarkNum,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);
    robotCombinedCovariance.ptr<double>(0)[0]=0.001;
    robotCombinedCovariance.ptr<double>(1)[1]=0.001;
    robotCombinedCovariance.ptr<double>(2)[2]=0.5;


    landmarkSets.clear();
    landmarkSets.reserve(5000);  //�w���}�]5000��landmark�Ŷ�
    candidateFeatureSets.clear();
    candidateWeighting.clear();

    PcontrolTimer = new QTimer;
}

lab405::MyEFKSLAM::~MyEFKSLAM()
{

}

void lab405::MyEFKSLAM::Initial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold, const std::string& filename_tPoints)
{
//    odoValueCurrent = cv::Point2d(0.0, 0.0); // x:right odo y:left odo
    // shih's EKF slam
    // EKFtimer flag
    checkBit = true;
    motionMode = false;
    sceneCnt = 0;
    saveFileIndex = 0;
    // number of scenes
    sceneNum = _sceneNum;

    thresh_count = threshold;


    tP_count = 0;
    std::string filename = "./" + filename_tPoints + ".txt";
    std::ifstream infile_tP (filename);
    if (infile_tP.is_open())
    {
        infile_tP >> num_tP;
        for (int i = 0; i < num_tP; i++)
        {
           int temp_x, temp_y;
           infile_tP >> temp_x >> temp_y;
           cv::Point temp (temp_x, temp_y);
           tP_set.push_back(temp);
        }
    }

    slam_x = _slam_x;
    slam_x0 = _slam_x0;
    slam_y = _slam_y;

    gridDistance = abs(slam_x - slam_x0)/(sceneNum - 1);
    gridDistance = gridDistance*this->gridMapper.GetPixel_meterFactor()*100;
    std::cout << "gridDistance:" << gridDistance << std::endl;

    // intial motor
    this->myrobot->left_dcmotor->SetHome();
    this->myrobot->right_dcmotor->SetHome();
    this->refenceMap.clear();

    this->myrobot->laser_slam->DisconnectReadyRead();

    // first,  extract features from environment


//    std::cout << laserdata.size() << std::endl;
//    for (int i = 0; i < laserdata.size(); i++)
//    {
//        std::cout << laserdata.at(i) << " ";
//    }
//    std::cout << std::endl;

    robotOutputFile.open("20150503_EKF_odoFile.txt");
    laserOutputFile.open("20150503_EKF_laserFile.txt");
    robotSceneFile.open("20150503_EKF_sceneData.txt");

    vector<vector<Line>> lines;
    CornerExtraction cornerEx;
    // repeat features
    for(int i = 0; i != 4; ++i)  // 6
    {
        std::vector<cv::Point2d> temp1;
        std::vector<Line> temp2;
        std::vector<Corner> cor;
        // triiger one scan

        vector<double> laserdata;
        this->myrobot->laser_slam->GetOneLaserData(laserdata);
//        std::cout << "temp size = " << temp.size() << std::endl;
//        for (int j = 0; j < temp.size(); j++)
//        {
//            std::cout << temp.at(j) << " ";
//        }
//        std::cout <<std::endl;

//        SLAM_Robot->laserS200->ReadData(temp);
        if(laserdata.size()==0||i==0)
            continue;
        this->mapper.RangesDataToPointsData(laserdata, temp1);

        this->lineExtracter.SplitAndMerge(temp1, temp2);
        cornerEx.ExtractCorners(laserdata,cor);

        std::cout << "laser points:" << laserdata.size() << "  line num:" << temp2.size() << "  cor num:" << cor.size() << std::endl;
        cv::Mat img = this->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

        cv::Mat img1 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
        cv::Mat img2 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


        lines.push_back(temp2);
        this->gridMapper.InsertLocalGridMap(laserdata, cv::Point3d(0,0,0));

        //////////////////////////////////////////////
        //save file
        saveFileIndex++;
        robotOutputFile << 0 << " " << 0 << endl;  //encoder
        for(int i = 0; i != laserdata.size(); ++i)
            laserOutputFile << laserdata[i] << " ";

        laserOutputFile << endl;


        //////////////////////////////////////////////
        //for(int i=0;i!=temp.size();++i)
        // {
        //  if(temp[i]<=2996)
        //      cout<<temp[i]<<endl;
        //}

        //  cv::Mat tt;
        // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
        // cv::imshow("tt",tt);
        cv::imshow("1341",img);
        cv::imshow("13411",img1);
        cv::imshow("13411222 ",cornerEx.cornerImg);
//        while(cv::waitKey(10) != 27)
//        {
//        }
    }

    this->MapInitial(lines);
    std::cout << "EKF Intial" <<endl;
    this->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
    this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
    this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
    //set start end points
    this->currentStart.x = this->gridMapper.GetGridMapOriginalPoint().x;
    this->currentStart.y = this->gridMapper.GetGridMapOriginalPoint().y;

    this->FinalEnd.x = this->gridMapper.GetGridMapOriginalPoint().x + 200;
    this->FinalEnd.y = this->gridMapper.GetGridMapOriginalPoint().y;


    this->GoalEnd.x = _slam_x;
    this->GoalEnd.y = _slam_y;

    // initial
    this->odoValuePrevious = cv::Point2d(0.0, 0.0);

    this->EKFTimer->start(1000);

    cv::destroyAllWindows();
}

void lab405::MyEFKSLAM::Connect()
{
//    QObject::connect(myrobot->left_dcmotor, SIGNAL(PositionAttained()),
//                     myrobot->left_dcmotor, SLOT(GetPose()));
    QObject::connect(myrobot->left_dcmotor, SIGNAL(ReturnPosition(int)),
                     this, SLOT(Test(int)));
//    QObject::connect(myrobot->right_dcmotor, SIGNAL(PositionAttained()),
//                     myrobot->right_dcmotor, SLOT(GetPose()));
//    QObject::connect(myrobot->right_dcmotor, SIGNAL(ReturnPosition(int)),
//                     this, SLOT(Test(int)));
}

void lab405::MyEFKSLAM::Prediction()
{
//    if(checkBit != true)
//    {

//        myekfslam->myrobot->right_dcmotor->Stop();
//        myekfslam->myrobot->left_dcmotor->Stop();
//        return;

//    }

//    clock_t startTime=clock();
//    //img=cv::Scalar::all(0);

//    saveFileIndex++;
//    readFlag=true;
//    double DeltaRight=((double)((odoValueCurrent.x-odoValuePrevious.x)*32.5*CV_PI)/(4096*14.0*3.33333));
//    double DeltaLeft=((double)((odoValueCurrent.y-odoValuePrevious.y)*32.5*CV_PI)/(4096*14.0*3.33333));



//    ui->textBrowser->setFontWeight( QFont::Normal );
//    ui->textBrowser->setTextColor( QColor( "blue" ) );
//    ui->textBrowser->append("[encoder]:("+QString::number(DeltaRight)+","+QString::number(DeltaLeft)+")");
//    cout<<"3:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<odoValuePrevious.x<<" , "<<odoValuePrevious.y<<endl;
//    cout<<"3:"<<DeltaRight<<" "<<DeltaLeft<<" time:"<<endl;

//    odoValuePrevious.x=odoValueCurrent.x;
//    odoValuePrevious.y=odoValueCurrent.y;

//    std::vector<cv::Point2d> points;
//    std::vector<Line> line;
//    std::vector<Corner> cor;

//    ////////////////////////////////////////////////////////////////////////
//    //feature extraction
//    myekfslam->myrobot->mapper.RangesDataToPointsData(myekfslam->myrobot->rawLaserScanData,points);
//    myekfslam->myrobot->lineExtracter.SplitAndMerge(points,line);

//    ////////////////////////////////////////////////////////////////////////
//    //motion estimation
//    //cout<<DeltaRight<<" "<<odoValueCurrent.x<<" "<<odoValuePrevious.x<<endl;DeltaLeft
//    myekfslam->myrobot->EKFRuner.MotionPrediction(cv::Point2d(DeltaLeft,DeltaRight));

//    //SLAM_Robot->gridMapper.InsertLocalGridMap();

//    myekfslam->myrobot->EKFRuner.DataAssociation(line,cor);
//    myekfslam->myrobot->robotPosition=myekfslam->myrobot->EKFRuner.GetRobotPose();



//    // QMessageBox::information(this, "Error!", "ok1!");

//    ////////////////////////////////////////////////////////////////////////
//    //ICP correction
//    vector<cv::Point2d> temp;
//    dataConvertRobotToWorld(points,myekfslam->myrobot->robotPosition,temp);

//    double percent=0.5;
//    //cv::Point3d adjustPose=SLAM_Robot->icper.Align(SLAM_Robot->refenceMap,temp,percent);
//    cv::Point3d adjustPose;
//    adjustPose.x=0;
//    adjustPose.y=0;
//    adjustPose.z=0;
//    ////////////////////////////////////////////////////////////////////////
//    //­×¥¿«áªº¾÷¾¹¤H¦ì¸m

//    RobotState robotpath1;

//    robotpath1.robotPositionMean.ptr<double>(0)[0]=adjustPose.x+myekfslam->myrobot->robotPosition.robotPositionMean.ptr<double>(0)[0];
//    robotpath1.robotPositionMean.ptr<double>(1)[0]=adjustPose.y+myekfslam->myrobot->robotPosition.robotPositionMean.ptr<double>(1)[0];
//    robotpath1.robotPositionMean.ptr<double>(2)[0]=adjustPose.z+myekfslam->myrobot->robotPosition.robotPositionMean.ptr<double>(2)[0];
//    robotpath1.robotPositionCovariance=myekfslam->myrobot->robotPosition.robotPositionCovariance.clone();

//    //xyz¹ê»Ú§ë¼v¦Ü¥­­±ªü
//    cv::Point2d imagePose=myekfslam->myrobot->gridMapper.GetRobotCenter(robotpath1);

//    myekfslam->myrobot->currentStart.x=imagePose.x;
//    myekfslam->myrobot->currentStart.y=imagePose.y;

//    myekfslam->myrobot->EKFRuner.SetRobotPose(robotpath1);
//    ///////////////////////////////////// ///////////////////////////////////

//    //add new map
//    for(int k=0;k!=points.size();++k)
//    {

//        double x=cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x-sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(0)[0];
//        double y=sin(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].x+cos(robotpath1.robotPositionMean.ptr<double>(2)[0])*points[k].y+robotpath1.robotPositionMean.ptr<double>(1)[0];
//        //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
//        myekfslam->myrobot->refenceMap.push_back(cv::Point2d(x,y));
//        //  cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

//    }

//    ////////////////////////////////////////////////////////////////////////
//    //mapping process

//    myekfslam->myrobot->mapper.InsertLocalLandmarkMap(points,robotpath1);
//    landMarkImg = myekfslam->myrobot->mapper.GetLandmarkMap();
//    myekfslam->myrobot->gridMapper.InsertLocalGridMap(myekfslam->myrobot->rawLaserScanData,robotpath1);
//    myekfslam->myrobot->gridMapper.GetOccupancyGridMap(gridImg);

//    myekfslam->myrobot->mapper.DrawRobotPoseWithErrorEllipse(robotpath1,landMarkImg,true);


//    ////////////////////////////////////////////////////////////////////////
//    //path planning
//    cv::Point2d tempEnd;
//    cv::Mat plannignGridMap;
//    double search_rect=80;
//    myekfslam->myrobot->planner.SetGridMap(gridImg);
//    myekfslam->myrobot->planner.GetPathPlanningMap(plannignGridMap);
//    cv::circle(plannignGridMap,myekfslam->myrobot->currentStart, 2, cv::Scalar(255,255,255), -1 );
//    myekfslam->myrobot->FindCurrentNodeEnd(plannignGridMap,search_rect,myekfslam->myrobot->currentStart,myekfslam->myrobot->currentEnd,tempEnd);


//    double rsrart=sqrt(pow(myekfslam->myrobot->currentStart.x,2.0) +pow(myekfslam->myrobot->currentStart.y,2.0)  );
//    double rend=sqrt(pow(myekfslam->myrobot->currentEnd.x,2.0) +pow(myekfslam->myrobot->currentEnd.y,2.0)  );
//    double rFinal=sqrt(pow(myekfslam->myrobot->FinalEnd.x,2.0) +pow(myekfslam->myrobot->FinalEnd.y,2.0)  );
//    double rTempEnd=sqrt(pow(tempEnd.x,2.0) +pow(tempEnd.y,2.0)  );

//    cv::Mat color_plannignGridMap;
//    cv::cvtColor(plannignGridMap,color_plannignGridMap,CV_GRAY2BGR);


//    //////////////////////////////////////////////
//    //save file
//    robotOutputFile<<DeltaRight<<" "<<DeltaLeft<<endl;
//    for(int i=0;i!=myekfslam->myrobot->rawLaserScanData.size();++i)
//        laserOutputFile<<myekfslam->myrobot->rawLaserScanData[i]<<" ";

//    laserOutputFile<<endl;//´«¦æ


//    //////////////////////////////////////////////

//    if(sceneCnt==sceneNum)  //»`¶°¨ì³õ´º¼Æ
//    {
//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);
//        testEKFTimer->stop();
//        scenesTimer->stop();
//        ui->textBrowser->setFontWeight( QFont::Normal );
//        ui->textBrowser->setTextColor( QColor( "blue" ) );
//        ui->textBrowser->append("[System Message]:Motor stop!");
//        return;
//    }

//    if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=gridDistance*sceneCnt)
//        //if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=sceneCnt*5)
//    {

//        checkBit=false;

//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);

//        motionMode=true;
//        ui->pushButton_2->click();

//        testEKFTimer->stop();
//        sceneCnt++;
//        //QMessageBox::information(this,"dd","ff");
//        cout<<"scen:"<<sceneCnt<<"  robotpose:"<<robotpath1.robotPositionMean<<"   index:"<<saveFileIndex<<endl;
//        robotSceneFile<<sceneCnt<<" "<<robotpath1.robotPositionMean.ptr<double>(0)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(1)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(2)[0]<<" "<<saveFileIndex<<endl;
//        return;
//    }



//    //if( (rend<=rsrart))  //¥b®|¤j©ó©Îµ¥©ó2*pixelFactor;

//    if( rsrart <= rTempEnd && threcont - ui->doubleSpinBox->value() >= myekfslam->myrobot->commandSets.size())
//    {
//        myekfslam->myrobot->commandSets = std::queue<std::pair<int,double>> ();
//        myekfslam->myrobot->planner.SetStartNode(myekfslam->myrobot->currentStart);

//        do
//        {

//            myekfslam->myrobot->planner.SetEndNode(tempEnd);
//            myekfslam->myrobot->planner.AStartPlanning(myekfslam->myrobot->robotPathSets);
//            //  QMessageBox::information(this, "Error!", "2_2!!"+QString::number(SLAM_Robot->robotPathSets.size()));
//            //  cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

//            //  cout<<"tempEnd1:"<<tempEnd<<endl;
//            //  cv::imshow("show",color_plannignGridMap);
//            //  cv::waitKey(1);


//            // QMessageBox::information(this, "Error!", "2_1_1");


//            if(myekfslam->myrobot->robotPathSets.size()!=0)
//            {
//                myekfslam->myrobot->PathSmoothing();

//                myekfslam->myrobot->TrajectoryGenerationSmoothing();
//                threcont=myekfslam->myrobot->commandSets.size();


//            }
//            else
//            {


//                tempEnd.x=myekfslam->myrobot->currentStart.x+10;
//                tempEnd.y=myekfslam->myrobot->currentStart.y;


//            }
//            //cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,0,255), 2  );
//            // cout<<"tempEnd2:"<<tempEnd<<endl;

//            //  cv::imshow("show",color_plannignGridMap);
//            //  cv::waitKey(1);
//            // QMessageBox::information(this, "Error!", "2_1_2");


//        }while(myekfslam->myrobot->robotPathSets.size()==0);

//        this->SetMotionCommand();

//        /*
//        if(SLAM_Robot->robotPathSets.size()!=0)
//        {
//            SLAM_Robot->PathSmoothing();

//            SLAM_Robot->TrajectoryGenerationSmoothing();
//            threcont=SLAM_Robot->commandSets.size();

//            this->SetMotionCommand();
//        }
//*/
//        ui->textBrowser->append("=====================");
//        ui->textBrowser->append("[robotPathSets size]:"+QString::number(myekfslam->myrobot->robotPathSets.size()));
//        for(int i=0;i!=myekfslam->myrobot->robotPathSets.size();++i)
//        {

//            cv::circle(color_plannignGridMap, cv::Point(myekfslam->myrobot->robotPathSets[i].x,myekfslam->myrobot->robotPathSets[i].y), 4 , cv::Scalar(0,0 ,255), -1  );
//            ui->textBrowser->setFontWeight( QFont::DemiBold );
//            ui->textBrowser->setTextColor( QColor( "red" ) );
//            ui->textBrowser->append("[robotPathSets]:"+QString::number(myekfslam->myrobot->robotPathSets[i].x)+" "+QString::number(myekfslam->myrobot->robotPathSets[i].y));

//        }
//        ui->textBrowser->append("=====================");
//    }






//    // else
//    // ui->textBrowser->append("=====no path fuck========");


//    cv::line( color_plannignGridMap, tempEnd, myekfslam->myrobot->currentEnd, cv::Scalar(0,0,255), 7,CV_AA);

//    cv::circle(color_plannignGridMap, myekfslam->myrobot->currentStart, 4, cv::Scalar(0,255,0), 2  );
//    cv::circle(color_plannignGridMap, myekfslam->myrobot->currentEnd, 4, cv::Scalar(0,255,0), 2  );
//    cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

//    clock_t endTime=clock();
//    double total=(double)(endTime-startTime)/CLK_TCK;

//    ui->textBrowser_slam->setFontWeight( QFont::DemiBold );
//    ui->textBrowser_slam->setTextColor( QColor( "red" ) );
//    ui->textBrowser_slam->append("[pose]:" + QString::number(robotpath1.robotPositionMean.ptr<double>(0)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(1)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(2)[0]));

//    std::cout << "[Robot pose]:" << robotpath1.robotPositionMean << std::endl;

//    std::cout << "single step Time:" << total << std::endl;
//    cv::imshow("gridImg",gridImg);
//    cv::imshow("landMarkImg",landMarkImg);
//    cv::imshow("color_plannignGridMap",color_plannignGridMap);
//    cv::imshow("plannignGridMap",plannignGridMap);
//    static int count=0;
//    QString name="Img/"+QString::number(count)+".jpg";
//    cv::imwrite(name.toStdString(),color_plannignGridMap);
//    cv::imwrite("Img/plannignGridMap.jpg",plannignGridMap);
//    cv::imwrite("Img/gridImg.jpg",gridImg);
//    cv::imwrite("Img/landMarkImg.jpg",landMarkImg);
//    count++;

//    cv::waitKey(1);




    //    //if(motionMode==true)
}


void lab405::MyEFKSLAM::Test(int pos)
{
    std::cout << "pos = " << pos << std::endl;
}

void lab405::MyEFKSLAM::EKFStepExamine()
{


    myrobot->right_dcmotor->Stop();
//    std::cout << "right stopped!" << std::endl;
    myrobot->left_dcmotor->Stop();
//    std::cout << "left stopped!" << std::endl;
//    emit MotorStop();


    int Pos_Stop_error = 1;
    int fir_pos = myrobot->right_dcmotor->GetPose();
    int sec_pos = myrobot->right_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
//        std::cout << sec_pos << " " << fir_pos << std::endl;
        fir_pos = sec_pos;
        sec_pos = myrobot->right_dcmotor->GetPose();
    }
//    double ReadingR = sec_pos;
    odoValueCurrent.x = sec_pos;
//    std::cout << "odoValueCurrent.x = " << odoValueCurrent.x << std::endl;

    fir_pos = myrobot->left_dcmotor->GetPose();
    sec_pos = myrobot->left_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
//        std::cout << sec_pos << " " << fir_pos << std::endl;
        fir_pos = sec_pos;
        sec_pos = myrobot->left_dcmotor->GetPose();
    }
//    double ReadingL = sec_pos;
    odoValueCurrent.y = sec_pos;
//    std::cout << "odoValueCurrent.y = " << odoValueCurrent.y << std::endl;


    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // reverse of static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi));
    double DeltaR = abs(odoValueCurrent.x - odoValuePrevious.x)*CV_PI*32.5*0.9487/(4096*3.333*14);  // unit cm
    double DeltaL = abs(odoValueCurrent.y - odoValuePrevious.y)*CV_PI*32.5*0.9487/(4096*3.333*14);
    std::cout << "DeltaR = " << DeltaR << ", DeltaL = " << DeltaL << std::endl;

    odoValuePrevious.x = odoValueCurrent.x;
    odoValuePrevious.y = odoValueCurrent.y;

//    saveFileIndex++;
//    readFlag=true;
//    double DeltaRight=((double)((odoValueCurrent.x-odoValuePrevious.x)*32.5*CV_PI)/(4096*14.0*3.33333));
//    double DeltaLeft=((double)((odoValueCurrent.y-odoValuePrevious.y)*32.5*CV_PI)/(4096*14.0*3.33333));



//    ui->textBrowser->setFontWeight( QFont::Normal );
//    ui->textBrowser->setTextColor( QColor( "blue" ) );
//    ui->textBrowser->append("[encoder]:("+QString::number(DeltaRight)+","+QString::number(DeltaLeft)+")");
//    cout<<"3:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<odoValuePrevious.x<<" , "<<odoValuePrevious.y<<endl;
//    cout<<"3:"<<DeltaRight<<" "<<DeltaLeft<<" time:"<<endl;

//    odoValuePrevious.x=odoValueCurrent.x;
//    odoValuePrevious.y=odoValueCurrent.y;



    ////////////////////////////////////////////////////////////////////////
    //feature extraction
//    std::vector<cv::Point2d> points;
//    std::vector<Line> line;
//    std::vector<Corner> cor;
    std::vector<double> LaserRawData;
    std::vector<cv::Point2d> LaserCatesianPoints;
    std::vector<Line> line;
    std::vector<Corner> cor;
    myrobot->laser_slam->GetOneLaserData(LaserRawData);

    this->mapper.RangesDataToPointsData(LaserRawData,LaserCatesianPoints);
    this->lineExtracter.SplitAndMerge(LaserCatesianPoints,line);


    ////////////////////////////////////////////////////////////////////////
    //motion estimation
    //cout<<DeltaRight<<" "<<odoValueCurrent.x<<" "<<odoValuePrevious.x<<endl;DeltaLeft
    // Prediction
    this->MotionPrediction(DeltaR, DeltaL);

    //SLAM_Robot->gridMapper.InsertLocalGridMap();

    this->DataAssociationAndUpdate(line,cor);
    // Get Robot State
    this->robotPosition = this->robotState;
//    myekfslam->myrobot->robotPosition=myekfslam->myrobot->EKFRuner.GetRobotPose();


//    std::cout << "after data ass" << std::endl;
//    // QMessageBox::information(this, "Error!", "ok1!");

//    ////////////////////////////////////////////////////////////////////////
//    //ICP correction
//    vector<cv::Point2d> temp;
//    dataConvertRobotToWorld(points, this->robotPosition, temp);
//    double percent=0.5;
//    cv::Point3d adjustPose=SLAM_Robot->icper.Align(SLAM_Robot->refenceMap,temp,percent);

    cv::Point3d adjustPose;
    adjustPose.x=0;
    adjustPose.y=0;
    adjustPose.z=0;
    ////////////////////////////////////////////////////////////////////////
    // robot pose after adjust
    RobotState RobotStateAfterAdjust;

    RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0] = adjustPose.x + this->robotPosition.robotPositionMean.ptr<double>(0)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0] = adjustPose.y + this->robotPosition.robotPositionMean.ptr<double>(1)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0] = adjustPose.z + this->robotPosition.robotPositionMean.ptr<double>(2)[0];
    RobotStateAfterAdjust.robotPositionCovariance = this->robotPosition.robotPositionCovariance.clone();

    // xyz project to plane
    cv::Point2d imagePose = this->gridMapper.GetRobotCenter(RobotStateAfterAdjust);

    // convert to unit: m
    this->currentStart.x = imagePose.x;
    this->currentStart.y = imagePose.y;

    // set into combine state
    this->SetRobotPose(RobotStateAfterAdjust);
    ///////////////////////////////////// ///////////////////////////////////


    // add new map
    // Rotate Laser Points
    for(int k = 0; k != LaserCatesianPoints.size(); ++k)
    {
        double x = LaserCatesianPoints[k].x*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                - LaserCatesianPoints[k].y*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0];
        double y = LaserCatesianPoints[k].x*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + LaserCatesianPoints[k].y*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0];
        //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
        this->refenceMap.push_back(cv::Point2d(x,y));
        //  cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

    }

    ////////////////////////////////////////////////////////////////////////
    //mapping process
    // Raw Catesian Map
    this->mapper.InsertLocalLandmarkMap(LaserCatesianPoints, RobotStateAfterAdjust);
    landMarkImg = this->mapper.GetLandmarkMap();

    // Grid Map
    this->gridMapper.InsertLocalGridMap(LaserRawData, RobotStateAfterAdjust);
    this->gridMapper.GetOccupancyGridMap(OccupancyGridMapImg);
    // draw on landmarkimg
    this->mapper.DrawRobotPoseWithErrorEllipse(RobotStateAfterAdjust, landMarkImg, true);

    cv::imshow("Grid Img", OccupancyGridMapImg);
    cv::waitKey(10);

    ////////////////////////////////////////////////////////////////////////
    //path planning
    cv::Point2d tempEnd;
    cv::Mat plannignGridMap;
    double search_rect = 80;
    this->planner.SetGridMap(OccupancyGridMapImg, 1);
    this->planner.GetPathPlanningMap(plannignGridMap);
    cv::circle(plannignGridMap, this->currentStart, 2, cv::Scalar(255,255,255), -1 );
    this->FindCurrentNodeEnd(plannignGridMap, search_rect, this->currentStart, this->GoalEnd, tempEnd);

    std::cout << "temp end = " << tempEnd << std::endl;
//    double rstart=sqrt(pow(this->currentStart.x,2.0) +pow(this->currentStart.y,2.0)  );
//    double rend=sqrt(pow(this->GoalEnd.x,2.0) +pow(this->GoalEnd.y,2.0)  );
//    double rFinal=sqrt(pow(this->FinalEnd.x,2.0) +pow(this->FinalEnd.y,2.0)  );
//    double rTempEnd=sqrt(pow(tempEnd.x,2.0) +pow(tempEnd.y,2.0)  );

    cv::Mat color_plannignGridMap;
    cv::cvtColor(plannignGridMap,color_plannignGridMap,CV_GRAY2BGR);


//    //////////////////////////////////////////////
//    //save file
//    robotOutputFile<<DeltaRight<<" "<<DeltaLeft<<endl;
//    for(int i=0;i!=myekfslam->myrobot->rawLaserScanData.size();++i)
//        laserOutputFile<<myekfslam->myrobot->rawLaserScanData[i]<<" ";

//    laserOutputFile<<endl;//´«¦æ


//    //////////////////////////////////////////////

//    if(sceneCnt==sceneNum)  //»`¶°¨ì³õ´º¼Æ
//    {
//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);
//        testEKFTimer->stop();
//        scenesTimer->stop();
//        ui->textBrowser->setFontWeight( QFont::Normal );
//        ui->textBrowser->setTextColor( QColor( "blue" ) );
//        ui->textBrowser->append("[System Message]:Motor stop!");
//        return;
//    }

//    if(robotpath1.robotPositionMean.ptr<double>(0)[0] >= gridDistance*sceneCnt)
//        //if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=sceneCnt*5)
//    {

//        checkBit=false;

//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);

//        motionMode=true;
//        ui->pushButton_2->click();

//        testEKFTimer->stop();
//        sceneCnt++;
//        //QMessageBox::information(this,"dd","ff");
//        cout<<"scen:"<<sceneCnt<<"  robotpose:"<<robotpath1.robotPositionMean<<"   index:"<<saveFileIndex<<endl;
//        robotSceneFile<<sceneCnt<<" "<<robotpath1.robotPositionMean.ptr<double>(0)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(1)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(2)[0]<<" "<<saveFileIndex<<endl;
//        return;
//    }





    //if( (rend<=rsrart))  //¥b®|¤j©ó©Îµ¥©ó2*pixelFactor;


//    if( rstart <= rTempEnd && temp_threscont - thresh_count >= this->commandSets.size())
//    {
        this->commandSets = std::queue<std::pair<int,double>> ();
//        this->planner.SetStartNode(this->currentStart);

//        do
//        {
    cv::Point2d rs_end(155, 500);

//            this->planner.SetEndNode(tempEnd);
//            this->planner.AStartPlanning(this->robotPathSets);
//            planner.run(this->currentStart, tempEnd, this->robotPathSets);
    planner.run(this->currentStart, rs_end, this->robotPathSets);

            //  QMessageBox::information(this, "Error!", "2_2!!"+QString::number(SLAM_Robot->robotPathSets.size()));
            //  cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

            //  cout<<"tempEnd1:"<<tempEnd<<endl;
            //  cv::imshow("show",color_plannignGridMap);
            //  cv::waitKey(1);


            // QMessageBox::information(this, "Error!", "2_1_1");


            if(this->robotPathSets.size() != 0)
            {
                this->PathSmoothing();

                this->TrajectoryGenerationSmoothing();
                temp_threscont = this->commandSets.size();


            }
            else
            {
                tempEnd.x = this->currentStart.x + 10;
                tempEnd.y = this->currentStart.y;
            }
            //cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,0,255), 2  );
            // cout<<"tempEnd2:"<<tempEnd<<endl;

            //  cv::imshow("show",color_plannignGridMap);
            //  cv::waitKey(1);
            // QMessageBox::information(this, "Error!", "2_1_2");


//        } while (this->robotPathSets.size() == 0);

        // control
//        this->SetMotionCommand();
        ControlMotion();

        /*
        if(SLAM_Robot->robotPathSets.size()!=0)
        {
            SLAM_Robot->PathSmoothing();

            SLAM_Robot->TrajectoryGenerationSmoothing();
            threcont=SLAM_Robot->commandSets.size();

            this->SetMotionCommand();
        }
*/
//        ui->textBrowser->append("=====================");
//        ui->textBrowser->append("[robotPathSets size]:"+QString::number(myekfslam->myrobot->robotPathSets.size()));
        for(int i = 0;i != this->robotPathSets.size();++i)
        {

            cv::circle(color_plannignGridMap, cv::Point(this->robotPathSets[i].x, this->robotPathSets[i].y), 4 , cv::Scalar(0,0 ,255), -1  );
//            ui->textBrowser->setFontWeight( QFont::DemiBold );
//            ui->textBrowser->setTextColor( QColor( "red" ) );
//            ui->textBrowser->append("[robotPathSets]:"+QString::number(myekfslam->myrobot->robotPathSets[i].x)+" "+QString::number(myekfslam->myrobot->robotPathSets[i].y));

        }
//        ui->textBrowser->append("=====================");
//    }







    // else
    // ui->textBrowser->append("=====no path fuck========");


        for (int i = 0; i < this->robotPathSets.size(); i++)
        {
            std::cout << "x = " << this->robotPathSets.at(i).x
                      << ", y = " << this->robotPathSets.at(i).y << std::endl;
        }
        std::cout << "this->currentStart " << this->currentStart << std::endl;
        cv::Mat show = color_plannignGridMap.clone();
        cv::circle(show, this->currentStart, 1, cv::Scalar(0, 0, 255));
        cv::circle(show, rs_end, 1, cv::Scalar(255, 0, 0));
        cv::imshow("show", show);
        cv::waitKey(1);

    cv::line(color_plannignGridMap, tempEnd, this->GoalEnd, cv::Scalar(0,0,255), 7,CV_AA);

//    std::cout << "this->currentStart = " << this->currentStart << std::endl;
//    std::cout << "this->currentEnd = " << this->currentEnd << std::endl;
//    std::cout << "tempEnd = " << tempEnd << std::endl;

//    ControlMotion(this->currentStart, tempEnd);

    cv::circle(color_plannignGridMap, this->currentStart, 4, cv::Scalar(0,255,0), 2  );
    cv::circle(color_plannignGridMap, this->GoalEnd, 4, cv::Scalar(0,255,0), 2  );
    cv::circle(color_plannignGridMap, tempEnd, 4, cv::Scalar(0,255,0), 2  );

//    clock_t endTime=clock();
//    double total=(double)(endTime-startTime)/CLK_TCK;

//    ui->textBrowser_slam->setFontWeight( QFont::DemiBold );
//    ui->textBrowser_slam->setTextColor( QColor( "red" ) );
//    ui->textBrowser_slam->append("[pose]:" + QString::number(robotpath1.robotPositionMean.ptr<double>(0)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(1)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(2)[0]));

    std::cout << "[Robot pose]:" << RobotStateAfterAdjust.robotPositionMean << std::endl;

//    std::cout << "single step Time:" << total << std::endl;
    cv::imshow("gridImg",OccupancyGridMapImg);
    cv::imshow("landMarkImg",landMarkImg);
    cv::imshow("color_plannignGridMap",color_plannignGridMap);
    cv::imshow("plannignGridMap",plannignGridMap);
    static int count=0;
    QString name="Img/"+QString::number(count)+".jpg";
    cv::imwrite(name.toStdString(),color_plannignGridMap);
    cv::imwrite("Img/plannignGridMap.jpg",plannignGridMap);
    cv::imwrite("Img/gridImg.jpg",OccupancyGridMapImg);
    cv::imwrite("Img/landMarkImg.jpg",landMarkImg);
    count++;

    cv::waitKey(1);




    //    //if(motionMode==true)
}

void lab405::MyEFKSLAM::PControlTest()
{
    myrobot->right_dcmotor->Stop();
    myrobot->left_dcmotor->Stop();

    int Pos_Stop_error = 1;
    int fir_pos = myrobot->right_dcmotor->GetPose();
    int sec_pos = myrobot->right_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
        fir_pos = sec_pos;
        sec_pos = myrobot->right_dcmotor->GetPose();
    }
    odoValueCurrent.x = sec_pos;

    fir_pos = myrobot->left_dcmotor->GetPose();
    sec_pos = myrobot->left_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
        fir_pos = sec_pos;
        sec_pos = myrobot->left_dcmotor->GetPose();
    }
    odoValueCurrent.y = sec_pos;


    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // reverse of static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi));
    double DeltaR = abs(odoValueCurrent.x - odoValuePrevious.x)*CV_PI*32.5*0.9487/(4096*3.333*14);  // unit cm
    double DeltaL = abs(odoValueCurrent.y - odoValuePrevious.y)*CV_PI*32.5*0.9487/(4096*3.333*14);
    std::cout << "DeltaR = " << DeltaR << ", DeltaL = " << DeltaL << std::endl;

    odoValuePrevious.x = odoValueCurrent.x;
    odoValuePrevious.y = odoValueCurrent.y;

    ////////////////////////////////////////////////////////////////////////
    //feature extraction
    std::vector<double> LaserRawData;
    std::vector<cv::Point2d> LaserCatesianPoints;
    std::vector<Line> line;
    std::vector<Corner> cor;
    myrobot->laser_slam->GetOneLaserData(LaserRawData);

    this->mapper.RangesDataToPointsData(LaserRawData,LaserCatesianPoints);
    this->lineExtracter.SplitAndMerge(LaserCatesianPoints,line);



    std::cout << "LaserCatesianPoints size = " << LaserCatesianPoints.size() << std::endl;
    // Line Ransac
    const double ransacDistanceThreshold = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    show_cloud->points.push_back(pcl::PointXYZRGB(0, 255, 0));

    for (int i = 0; i < LaserCatesianPoints.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = LaserCatesianPoints.at(i).x;
        point.y = LaserCatesianPoints.at(i).y;
        point.z = 0;
        cloud_copy->points.push_back(point);

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
    //  created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
            model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_copy));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
    std::vector<int> inliers;
    ransac.setDistanceThreshold (ransacDistanceThreshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
    std::cout << "inliers size = " << inliers.size() << std::endl;
    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud_copy, inliers, *final);
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
    cv::Point2d para;
    LineExtraction::LineRhoThetaExtraction(final, para);
    std::cout << "rho = " << para.x <<", theta = " << para.y << std::endl;

    MotionControl(para);

//    pcl::visualization::CloudViewer viewer("Simple Viewer");
//    viewer.showCloud(show_cloud);
//    while(!viewer.wasStopped())
//    {
//    }

    ////////////////////////////////////////////////////////////////////////
    // motion estimation
    // Prediction
    this->MotionPrediction(DeltaR, DeltaL);

    //SLAM_Robot->gridMapper.InsertLocalGridMap();

    this->DataAssociationAndUpdate(line, cor);

    cv::Mat show = cv::Mat::zeros(800, 800, CV_8UC3);
    for (int i = 0; i < line.size(); i++)
    {
        double r = line.at(i).lineMean.ptr<double>(0)[0];
        double mytheta = line.at(i).lineMean.ptr<double>(1)[0];
//        double cosvalue = (-1)*a/(a*a + 1);
//        double r = abs(b)/(a*a + 1);

//        double mytheta = acos(cosvalue);
        std::cout << "[Debug] Line No." << i << " r = " << r << ", theta = " << mytheta << std::endl;
        if (mytheta > 2.8 && mytheta < CV_PI)
        {
            cv::circle(show, cv::Point(400, 400), 1, cv::Scalar(0, 255, 0));
            cv::line(show, cv::Point(400 + r/cos(mytheta), 400), cv::Point(400 + r*cos(mytheta) , 400 - r*sin(mytheta)), cv::Scalar(0, 0, 255));
            cv::imshow("Line Pic", show);
            cv::waitKey(1);
        }
    }
    // Get Robot State
    this->robotPosition = this->robotState;


    cv::Point3d adjustPose;
    adjustPose.x=0;
    adjustPose.y=0;
    adjustPose.z=0;
    ////////////////////////////////////////////////////////////////////////
    // robot pose after adjust
    RobotState RobotStateAfterAdjust;

    RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0] = adjustPose.x + this->robotPosition.robotPositionMean.ptr<double>(0)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0] = adjustPose.y + this->robotPosition.robotPositionMean.ptr<double>(1)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0] = adjustPose.z + this->robotPosition.robotPositionMean.ptr<double>(2)[0];
    RobotStateAfterAdjust.robotPositionCovariance = this->robotPosition.robotPositionCovariance.clone();

    // xyz project to plane
    cv::Point2d imagePose = this->gridMapper.GetRobotCenter(RobotStateAfterAdjust);

    // convert to unit: m
    this->currentStart.x = imagePose.x;
    this->currentStart.y = imagePose.y;

    // set into combine state
    this->SetRobotPose(RobotStateAfterAdjust);
    ///////////////////////////////////// ///////////////////////////////////


    // add new map
    // Rotate Laser Points
    for(int k = 0; k != LaserCatesianPoints.size(); ++k)
    {
        double x = LaserCatesianPoints[k].x*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                - LaserCatesianPoints[k].y*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0];
        double y = LaserCatesianPoints[k].x*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + LaserCatesianPoints[k].y*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0];
        //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
        this->refenceMap.push_back(cv::Point2d(x,y));
        //  cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

    }

    std::cout << "[Robot pose]:" << RobotStateAfterAdjust.robotPositionMean << std::endl;

    ////////////////////////////////////////////////////////////////////////
    //mapping process
    // Raw Catesian Map
    this->mapper.InsertLocalLandmarkMap(LaserCatesianPoints, RobotStateAfterAdjust);
    landMarkImg = this->mapper.GetLandmarkMap();
    // draw on landmarkimg
    this->mapper.DrawRobotPoseWithErrorEllipse(RobotStateAfterAdjust, landMarkImg, true);
    cv::imshow("landMarkImg",landMarkImg);
    cv::waitKey(1);
}

// Propagation the time k state to k + 1 state via motion model
void lab405::MyEFKSLAM::MotionPrediction(const double DeltaR, const double DeltaL)
{
    // DeltaR, DeltaL
//    double DeltaR = _DeltaR;
//    double DeltaL = _DeltaL;

    ////////////////////////////////////////////////////////////////////////////////
    // Get Time k postition
//    double x = robotState.robotPositionMean.ptr<double>(0)[0];
//    double y = robotState.robotPositionMean.ptr<double>(1)[0];
    double phi = robotState.robotPositionMean.ptr<double>(2)[0];  //rad

    // DeltaC = (DeltaR + DeltaL)/2, the trajectory of center of mass
    double DeltaC = (DeltaR + DeltaL)/2.0;
    double DeltaTheta = (DeltaR - DeltaL)/robotWidth;

    cv::Mat Fx = cv::Mat::eye(STATE_SIZE, STATE_SIZE + OBS_SIZE*landmarkNum, CV_64F);  //3*(3+3*N)


    ////////////////////////////////////////////////////////////////////////////////
    //------±a¤J±À¾Éªº¾÷¾¹¤Hmotion modelªºmean­È  Ut=f(command,Ut-1)-------------------------------------------------------
    // bring means into motion model, Ut=f(command, Ut-1)

    //*** deltaT?
    // X' = X + DeltaC*cos(theta + delta_theta/2)
    // Y' = Y + DeltaC*cos(theta + delta_theta/2)
    // theta' = theta + delta_theta;
    // which delta_theta = (DeltaR - DeltaL)/b
    double tempX = DeltaC*cos(phi + DeltaTheta/2.0*deltaT);
    double tempY = DeltaC*sin(phi + DeltaTheta/2.0*deltaT);
    double tempTheta = DeltaTheta*deltaT;

    // take displacement into current robot state
    robotState.robotPositionMean.ptr<double>(0)[0] += tempX;
    robotState.robotPositionMean.ptr<double>(1)[0] += tempY;
    robotState.robotPositionMean.ptr<double>(2)[0] += tempTheta;

    //***y vector ??
    robotCombinedState.ptr<double>(0)[0] += tempX;
    robotCombinedState.ptr<double>(1)[0] += tempY;
    robotCombinedState.ptr<double>(2)[0] += tempTheta;

    // angle normalize
    while(robotState.robotPositionMean.ptr<double>(2)[0] > CV_PI)
    {
        robotState.robotPositionMean.ptr<double>(2)[0] -= 2*CV_PI;
        robotCombinedState.ptr<double>(2)[0] -= 2*CV_PI;
    }
    while(robotState.robotPositionMean.ptr<double>(2)[0] < -CV_PI)
    {
        robotState.robotPositionMean.ptr<double>(2)[0] += 2*CV_PI;
        robotCombinedState.ptr<double>(2)[0] += 2*CV_PI;
    }


    ////////////////////////////////////////////////////////////////////////////////
    //------±a¤J±À¾Éªº¾÷¾¹¤Hmotion modelªºcovariance  -------------------------------------------------------
    //(a):·ímotion model§ïÅÜ®É motion model Jacobians­n§ï³oÃä


    // (a): Jacobians
    // Fp, Jacobain of robot state (p), derivative of p' with respect (x, y, theta)
    cv::Mat Gt = cv::Mat::eye(STATE_SIZE + OBS_SIZE*landmarkNum, STATE_SIZE + OBS_SIZE*landmarkNum, CV_64F);
    Gt.ptr<double>(0)[2] = -DeltaC*sin(phi + DeltaTheta/2.0*deltaT);
    Gt.ptr<double>(1)[2] = DeltaC*cos(phi + DeltaTheta/2.0*deltaT);



    // (b): motion
    //(b):·ímotion model§ïÅÜ®É motion error model Jacobians­n§ï³oÃä
    cv::Mat motionErrorJacobian = cv::Mat::zeros(STATE_SIZE, MOT_SIZE, CV_64F);
    cv::Mat R = cv::Mat::zeros(STATE_SIZE, STATE_SIZE, CV_64F);
    motionErrorJacobian.ptr<double>(0)[0] = 0.5*cos(phi + DeltaTheta/2.0*deltaT) - DeltaC/(2.0*robotWidth)*deltaT*sin(phi + DeltaTheta/2.0*deltaT);
    motionErrorJacobian.ptr<double>(0)[1] = 0.5*cos(phi + DeltaTheta/2.0*deltaT) + DeltaC/(2.0*robotWidth)*deltaT*sin(phi + DeltaTheta/2.0*deltaT);
    motionErrorJacobian.ptr<double>(1)[0] = 0.5*sin(phi + DeltaTheta/2.0*deltaT) + DeltaC/(2.0*robotWidth)*deltaT*cos(phi + DeltaTheta/2.0*deltaT);
    motionErrorJacobian.ptr<double>(1)[1] = 0.5*sin(phi + DeltaTheta/2.0*deltaT) - DeltaC/(2.0*robotWidth)*deltaT*cos(phi + DeltaTheta/2.0*deltaT);
    motionErrorJacobian.ptr<double>(2)[0] = deltaT/robotWidth;
    motionErrorJacobian.ptr<double>(2)[1] = -deltaT/robotWidth;

    // (c) motion noise
    cv::Mat motionError = cv::Mat::eye(MOT_SIZE, MOT_SIZE, CV_64F);
    motionError.ptr<double>(0)[0] = Kr*DeltaR;         //Sr^2     5
    motionError.ptr<double>(0)[1] = 0.0;                        //SrSl
    motionError.ptr<double>(1)[0] = 0.0;                        //SlSr
    motionError.ptr<double>(1)[1] = Kl*DeltaL;         //Sl^2     5

    R = motionErrorJacobian*motionError*motionErrorJacobian.t();

    // Sigma_p'
    robotCombinedCovariance = cv::Mat(Gt*robotCombinedCovariance*Gt.t() + Fx.t()*R*Fx).clone();


//    for (int y = 0; y < STATE_SIZE; y++)
//        double *ptr = robotState.robotPositionCovariance.ptr<double>(y);
//        double robotCombinedCovariance.ptr<double>(i)[j];
//        for (int x = 0; x < STATE_SIZE; x++)
//            ptr[x] =

    for(int i = 0;i != STATE_SIZE; ++i)
        for(int j = 0;j != STATE_SIZE; ++j)
            robotState.robotPositionCovariance.ptr<double>(i)[j] = robotCombinedCovariance.ptr<double>(i)[j];


   // cv::imshow("23",robotCombinedCovariance*1000);
    //cv::waitKey(1);

    //////////////////////////////////////////////////////
    //cout<<"//////////////////////////////////////////////////////"<<endl;
    //cout<<"¹w´úªº¾÷¾¹¤H¦ì¸m"<<robotCombinedState<<endl;
    //cout<<"¹w´úªº¾÷¾¹¤HCovariance"<<robotCombinedCovariance<<endl;
    std::cout << "Predicted Robot State" << robotState.robotPositionMean << std::endl;
     //cout<<"¹w´úªº¾÷¾¹¤H"<<robotState.robotPositionCovariance<<endl;
    // cout<<"//////////////////////////////////////////////////////"<<endl;
}

void lab405::MyEFKSLAM::DataAssociationAndUpdate(const std::vector<Line> &obsLineFeature, const std::vector<Corner> &obsCornerFeature)
{
    //obsFeature: match corner and line features in robot coordinate


    LandmarkMapping mapper;
    cv::Mat imgg(500,500,CV_8UC3);
    imgg=cv::Scalar::all(0);
    cv::circle(imgg, cv::Point(0 + 250,0 + 250), 3, cv::Scalar(255, 0, 255), 2 );


    for(int i=0;i!=landmarkSets.size();++i)
    {

        // Green line is landmark Line Features
        mapper.DrawLine(landmarkSets[i],RobotState(),cv::Scalar(0,255,0),1,cv::Point2d(250,250),1,imgg);

    }

              // mapper.DrawLine(temp11,RobotState(),cv::Scalar(255,0,0),10,cv::Point2d(250,250),1,imgg);


    cout<<"=================DataAssociation================================"<<endl;
    // Create Container for match flags
    vector<bool> obsMatchFlag(obsLineFeature.size() + obsCornerFeature.size(), false);  // match: true
    // Create Container for match Num (Observed to Map)
    vector<int> landmarkMatchingNum(obsLineFeature.size() + obsCornerFeature.size(), -1);  // not match, tag as -1
    // Create Container for Means and Covariance of Observed Features
    vector<Feature> obsFeature;
    obsFeature.reserve(obsLineFeature.size() + obsCornerFeature.size());

    for(int i = 0; i != obsLineFeature.size(); ++i)
    {
        Feature temp;
        temp.featureMean = obsLineFeature[i].lineMean.clone();
        temp.featureCovariance = obsLineFeature[i].lineCovariance.clone();
        temp.SetFeatureType(Line_Feature);
        obsFeature.push_back(temp);
    }

    for(int i=0;i!=obsCornerFeature.size();++i)
    {
        Feature temp;
        temp.featureMean=obsCornerFeature[i].cornerMean.clone();
        temp.featureCovariance=obsCornerFeature[i].cornerCovariance.clone();
        temp.SetFeatureType(Corner_Feature);

        obsFeature.push_back(temp);
    }

    //  Create Container for un-match features
    vector<Feature> newFeature;
    newFeature.reserve(obsLineFeature.size() + obsCornerFeature.size());

    cout<<"The Number of Observed Features: "<<obsFeature.size()<<endl;


    // Deal with All observed features
    for(int i = 0; i != obsFeature.size(); ++i)
    {
        //obsFeature: the Observed Features of this Step

        /*
        Feature transferTemp;
      //  Feature gobalLineobs;
        // convert line feature to feature type
        transferTemp.featureMean=obsLineFeature[i].lineMean;
        transferTemp.featureCovariance=obsLineFeature[i].lineCovariance;
        transferTemp.SetFeatureType(Line_Feature);

        //ConvertRobotToWorld(transferTemp,robotState,gobalLineobs);  // from robot coordinate to world coordinate
        */
        cv::Mat H_min, innovation_min, innovationCovariance_min;

        double GatingFeature=0;

        // default gl: 0.25, gc: 0
        if(obsFeature[i].GetFeatureType() == Line_Feature)
            GatingFeature = gatingLine;
        else
            GatingFeature = gatingCorner;


        // All Observed Features Match with Features of Map
        for(int j = 0; j != landmarkSets.size(); ++j)
        {
            // first filter out type different situation
            if(obsFeature[i].GetFeatureType()!=landmarkSets[i].GetFeatureType())
                continue;

            Feature localLandmark;
            ConvertWorldToRobot(landmarkSets[j], robotState, localLandmark); // from world coordinate to robot coordinate
            // measurement estimation
            // Jacobian of coordinate transformation between the world frame and the sensor frame
            cv::Mat H = cv::Mat::zeros(OBS_SIZE, STATE_SIZE + OBS_SIZE*landmarkNum,CV_64F);
            double x = robotState.robotPositionMean.ptr<double>(0)[0];
            double y = robotState.robotPositionMean.ptr<double>(1)[0];
            // landmark space: r alpha
            double alpha = landmarkSets[j].featureMean.ptr<double>(1)[0];

            // covariance
            H.ptr<double>(0)[0]=-cos(alpha);
            H.ptr<double>(0)[1]=-sin(alpha);
            H.ptr<double>(1)[2]=-1;
            H.ptr<double>(0)[STATE_SIZE + OBS_SIZE*(landmarkNum-1)] = 1;
            H.ptr<double>(0)[STATE_SIZE + OBS_SIZE*(landmarkNum-1)+1] = x*sin(alpha) - y*cos(alpha);
            H.ptr<double>(1)[STATE_SIZE + OBS_SIZE*(landmarkNum-1)+1] = 1;


            // innovation: r alpha

            // cv::Mat innovation(obsLineFeature[i].lineMean.size(),obsLineFeature[i].lineMean.type());
            cv::Mat innovation=(obsFeature[i].featureMean-localLandmark.featureMean);

            //normalize

            while(innovation.ptr<double>(1)[0]> CV_PI)
            {
                innovation.ptr<double>(1)[0] -= 2*CV_PI;
            }
            while(innovation.ptr<double>(1)[0]< -CV_PI)
            {
                innovation.ptr<double>(1)[0] += 2*CV_PI;
            }

            cv::Mat innovationCovariance = H*robotCombinedCovariance*H.t() + obsFeature[i].featureCovariance;

            double gating = _MahalanobisDistance(innovation, innovationCovariance);


            // line feature debug message
//            if (obsFeature[i].GetFeatureType() == Line_Feature)
//            {
//                std::cout << i << " Line Gating = " << gating << ", threshold = " << GatingFeature << std::endl;
//            }
            if(gating <= GatingFeature)
            {

                H_min=H.clone();
                innovation_min=innovation.clone();
                innovationCovariance_min=innovationCovariance.clone();

                landmarkMatchingNum[i]=j;
                GatingFeature=gating;
                obsMatchFlag[i]=true;

            }

        }
        if ( obsMatchFlag[i]== true)//match
        {


            Feature temp11;


            ConvertRobotToWorld(obsFeature[i], robotState, temp11);

            // Blue line is landmark of map
            mapper.DrawLine(landmarkSets[landmarkMatchingNum[i]], RobotState(),cv::Scalar(255,0,0),2,cv::Point2d(250,250),1,imgg);
            // Red line is observed feature
            mapper.DrawLine(temp11,RobotState(),cv::Scalar(0,0,255),2,cv::Point2d(250,250),1,imgg);


            cout << "[Matched!]  No." << i << " Observed Feature with No." << landmarkMatchingNum[i]<<" Map Score:"<<endl;
            cout<<"Before Update: "<<robotState.robotPositionMean<<endl;
            Update(H_min,innovation_min,innovationCovariance_min);  //kalman filter framework
            cout<<"After Update"<<robotState.robotPositionMean<<endl;



            //////////////////////////////////////
            //            count++;

        }
        else
        {
            newFeature.push_back(obsFeature[i]);  // take not match feature as new Landmark
            // cout<<"Add into Candidates"<<endl;
            // AddNewLandmark(obsFeature[i]);
        }

    }




    //feature selection
    vector<Feature> map;
    FeatureSelection(newFeature,map);
     //cout<<"Number of landmark = "<<map.size()<<endl;
    for(int i=0;i!=map.size();++i)
    {
        Feature localTemp;
        ConvertWorldToRobot(map[i],robotState,localTemp);
        AddNewLandmark(localTemp);
       // cout<<"localTemp"<<localTemp.featureMean<<endl;

    }




    cv::imshow("imgg",imgg);
    cv::waitKey(1);




    /*
    cout<<"Increase Number: "<<map.size()<<" Number of Feature on Map:"<<landmarkSets.size()<<endl;
    cout<<"Map Features (Robot Coordinate) ==========="<<endl;
    for(int i=0;i!=landmarkSets.size();++i)
    {
        Feature localLandmark;
        ConvertWorldToRobot(landmarkSets[i],robotState,localLandmark);
        cout<<"No."<<i<<"­ "<<localLandmark.featureMean<<endl;
//        cout<<"No."<<i<<"­ "<<landmarkSets[i].featureMean<<endl;
    }
    */


   // cout<<"=========================================================="<<endl;
   // cout<<"Robot State After Update"<<endl;
    //cout<<robotCombinedState<<endl;
    ///cout<<"Robot Update Position Covariance"<<endl;
    //cout<<robotCombinedCovariance<<endl;
}

double lab405::MyEFKSLAM::_MahalanobisDistance(const cv::Mat &mean, const cv::Mat &covariance)
{
    cv::Mat mahalanobisDistance=(mean.t()*(covariance.inv())*mean);

    return mahalanobisDistance.ptr<double>(0)[0];
}

void lab405::MyEFKSLAM::Update(const cv::Mat &H, const cv::Mat &innovation, const cv::Mat &innovationCovariance)
{
    cv::Mat kalmanGain=robotCombinedCovariance*H.t()*(innovationCovariance.inv());
    cv::Mat robotPositonTemp=robotCombinedState+kalmanGain*innovation;
    cv::Mat I=cv::Mat::eye(STATE_SIZE+OBS_SIZE*landmarkNum,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);
    cv::Mat robotCovarianceTemp=(I-kalmanGain*H)*robotCombinedCovariance;



    robotCombinedState=robotPositonTemp.clone();
    robotCombinedCovariance=robotCovarianceTemp.clone();

    robotState.robotPositionMean.ptr<double>(0)[0]=robotCombinedState.ptr<double>(0)[0];
    robotState.robotPositionMean.ptr<double>(1)[0]=robotCombinedState.ptr<double>(1)[0];
    robotState.robotPositionMean.ptr<double>(2)[0]=robotCombinedState.ptr<double>(2)[0];


    for(int i=0;i!=STATE_SIZE;++i)
       for(int j=0;j!=STATE_SIZE;++j)
            robotState.robotPositionCovariance.ptr<double>(i)[j]=robotCombinedCovariance.ptr<double>(i)[j];

}

void lab405::MyEFKSLAM::AddNewLandmark(const Feature &singleFeature)
{
    //ÂX¤j¯x°}
    cv::Mat extendMap=cv::Mat::zeros(STATE_SIZE+OBS_SIZE*(++landmarkNum),1,CV_64F);
    cv::Mat extendMapC=cv::Mat::zeros(extendMap.rows,extendMap.rows,CV_64F);
    //cv::Mat H = cv::Mat::zeros(OBS_SIZE,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);

    Feature worldFeature;
    ConvertRobotToWorld(singleFeature,robotState,worldFeature);  //compounding Ãö«Y
    //»Ý­nÂà¦¨¥@¬É®y¼Ðªºr alpha
    double r=worldFeature.featureMean.ptr<double>(0)[0];
    double alpha=worldFeature.featureMean.ptr<double>(1)[0];

    double x=robotState.robotPositionMean.ptr<double>(0)[0];
    double y=robotState.robotPositionMean.ptr<double>(1)[0];

    //mean
    for(int i=0; i<robotCombinedState.rows;i++)
    {
        extendMap.ptr<double>(i)[0]=robotCombinedState.ptr<double>(i)[0];

    }

    extendMap.ptr<double>(robotCombinedState.rows)[0]=r;
    extendMap.ptr<double>(robotCombinedState.rows+1)[0]=alpha;

    cout<<"addstate"<<extendMap<<endl;


    cv::Mat H1=cv::Mat::zeros(OBS_SIZE,STATE_SIZE,CV_64F);
    cv::Mat H2=cv::Mat::eye(OBS_SIZE,OBS_SIZE,CV_64F);

    H1.ptr<double>(0)[0]=-cos(alpha);
    H1.ptr<double>(0)[1]=-sin(alpha);
    H1.ptr<double>(1)[2]=-1;

    H2.ptr<double>(0)[1]=x*sin(alpha)-y*cos(alpha);

    cv::Mat mapC=H1*robotState.robotPositionCovariance*H1.t()+H2*singleFeature.featureCovariance*H2.t();
    cv::Mat robot_mapCovariance=robotState.robotPositionCovariance*H1.t();
    cv::Mat map_robotCovariance=H1*robotState.robotPositionCovariance;

    //P=[  R  RM; MR M]
    //R
    cv::Mat temp=extendMapC(cv::Rect(0,0,robotCombinedCovariance.cols,robotCombinedCovariance.rows));
    robotCombinedCovariance.copyTo(temp);
    //M
    temp=extendMapC(cv::Rect(robotCombinedCovariance.cols,robotCombinedCovariance.rows,mapC.cols,mapC.rows));
    mapC.copyTo(temp);
    //RM
    temp=extendMapC(cv::Rect(robotCombinedCovariance.cols,0,robot_mapCovariance.cols,robot_mapCovariance.rows));
    robot_mapCovariance.copyTo(temp);
    //MR
    temp=extendMapC(cv::Rect(0,robotCombinedCovariance.rows,map_robotCovariance.cols,map_robotCovariance.rows));
    map_robotCovariance.copyTo(temp);

    //mean
    robotCombinedState=extendMap.clone();
    robotState.robotPositionMean.ptr<double>(0)[0]=robotCombinedState.ptr<double>(0)[0];
    robotState.robotPositionMean.ptr<double>(1)[0]=robotCombinedState.ptr<double>(1)[0];
    robotState.robotPositionMean.ptr<double>(2)[0]=robotCombinedState.ptr<double>(2)[0];

    robotCombinedCovariance=extendMapC.clone();

    for(int i=0;i!=STATE_SIZE;++i)
       for(int j=0;j!=STATE_SIZE;++j)
            robotState.robotPositionCovariance.ptr<double>(i)[j]=robotCombinedCovariance.ptr<double>(i)[j];


    landmarkSets.push_back(worldFeature);


    //cout<<robotCombinedCovariance<<endl;

 //  cv::imshow("23",robotCombinedCovariance*1000);
  // cv::waitKey(1);
    //cout<<"======================="<<endl;
}

void lab405::MyEFKSLAM::FeatureSelection(const std::vector<Feature> &obsFeatureSets, std::vector<Feature> &landmark)
{
    landmark.clear();
    landmark.reserve(obsFeatureSets.size());
    //

    vector<bool> matchFlag;  //true: match  §PÂ_obs¯S¼x¬O§_³Qmatch¨ì ¦pªG¨S¦³³Qmatch¨ì«hµø¬°²Ä¤@¦¸µo²{
    matchFlag.reserve(obsFeatureSets.size());


    for(int i=0;i!=obsFeatureSets.size();++i)
    {
        Feature obsglobalFeature;
        ConvertRobotToWorld(obsFeatureSets[i],robotState,obsglobalFeature);

        std::list<Feature>::const_iterator j=candidateFeatureSets.begin();
        std::list<int>::iterator k=candidateWeighting.begin();

        bool match=false;
        for(int count=0;count!=candidateFeatureSets.size();++count)
        {
            if(obsglobalFeature.GetFeatureType()!=j->GetFeatureType())
                continue;

            double distance=_EuclideanDistance(obsglobalFeature,*j);


            if(distance<newFeatureRadius)
            {
                //cout<<"²Ä"<<i<<"­Ó¯S¼x_»P²Ä"<<count<<"  "<<(*j).featureMean<<"­Ó­Ô¿ï¤H_¤À¼Æ"<<distance<<endl;

                *k=*k+1;  //­p¼Æ+1
                match=true;
               //­n«ä¦Ò§ä¨ì¤@­Ó¬O§_Ä~ÄòÅýfor¶]§¹
            }

            j++;
            k++;  //«ü¼Ð»¼¼W

        }


        //check landmark set ¬O§_¦³¤@¼Ëªº¯S¼x
        /////////////////////////////////

        for(int k=0;k!=landmarkSets.size();++k)
        {
            if(obsglobalFeature.GetFeatureType()!=landmarkSets[k].GetFeatureType())
                continue;


            double distance=_EuclideanDistance(obsglobalFeature,landmarkSets[k]);

            if(distance<(newFeatureRadius)*5)
            {
                 match=true;

            }

        }


         ////////////////////////////////

        //§PÂ_obs¯S¼x¬O§_³Qmatch¨ì ¦pªG¨S¦³³Qmatch¨ì«hµø¬°²Ä¤@¦¸µo²{
        matchFlag.push_back(match);
    }


    /*
    cout<<"add  matcher==========="<<endl;
    for(int i=0;i!=matchFlag.size();++i)
        cout<<matchFlag[i]<<endl;
    cout<<"add  matcher==========="<<endl;

*/


    std::list<Feature>::iterator i=candidateFeatureSets.begin();
    std::list<int>::iterator j;

    for(j=candidateWeighting.begin();j!=candidateWeighting.end();)
    {


        if(*j>=weighting&&j!=candidateWeighting.end()&&i!=candidateFeatureSets.end())
        {
            landmark.push_back(*i);
            j=candidateWeighting.erase(j);
            i=candidateFeatureSets.erase(i);
        }
        else
        {
            ++j;
            ++i;
        }


    }


/*
    for(int c=0;c!=candidateWeighting.size();++c)
    {


        if(*j>=weighting&& j!=candidateWeighting.end()&&i!=candidateFeatureSets.end())
        {
            landmark.push_back(*i);
            candidateWeighting.erase(j);
            candidateFeatureSets.erase(i);
        }


    }

*/
        //²Ä¤@¦¸µo²{ªº¯S¼x
        for(int i=0;i!=matchFlag.size();++i)
        {
            if(matchFlag[i]==false)
            {

                Feature obsglobalFeature;
                ConvertRobotToWorld(obsFeatureSets[i],robotState,obsglobalFeature);

                candidateFeatureSets.push_back(obsglobalFeature);
                candidateWeighting.push_back(1);


            }
        }

}

double lab405::MyEFKSLAM::_EuclideanDistance(const Feature &obsFeautureW, const Feature &candFeautureW)
{
    // Polar to Cartesian
    cv::Point2d obsXY;
    cv::Point2d candXY;
    PolarToCartesian(cv::Point2d(obsFeautureW.featureMean.ptr<double>(0)[0],obsFeautureW.featureMean.ptr<double>(1)[0]),obsXY);
    PolarToCartesian(cv::Point2d(candFeautureW.featureMean.ptr<double>(0)[0],candFeautureW.featureMean.ptr<double>(1)[0]),candXY);

    return sqrt((obsXY.x-candXY.x)*(obsXY.x-candXY.x)+(obsXY.y-candXY.y)*(obsXY.y-candXY.y));
}

void lab405::MyEFKSLAM::MapInitial(const std::vector<std::vector<Line> > &lineFeature)
{
    LandmarkMapping mapper;

    cv::Mat imgg(500,500,CV_8UC3);
    imgg=cv::Scalar::all(0);
    cv::circle(imgg, cv::Point(0+250,0+250), 3, cv::Scalar(255,0,255), 1 );
    //the line features of n scans
    for(int i=0;i!=lineFeature.size();++i)
    {
        vector<Feature> obs;
        vector<Feature> map;
        obs.clear();
        obs.reserve(lineFeature[i].size());

        for(int j=0;j!=lineFeature[i].size();++j)
        {
            // convert line feature to normal feature
            Feature temp;
            temp.featureMean=lineFeature[i][j].lineMean.clone();
            temp.featureCovariance=lineFeature[i][j].lineCovariance.clone();
            temp.SetFeatureType(Line_Feature);
            obs.push_back(temp);


            // convert feature to world space
            Feature temp11;


            ConvertRobotToWorld(temp,robotState,temp11);

           // mapper.DrawLine(temp11,RobotState(),cv::Scalar(255,0,0),10,cv::Point2d(250,250),1,imgg);


        }

        FeatureSelection(obs, map);
        cout<<"²Ä´X­Óloop:"<<i<<" µo²{´X­Ó¯S¼x:"<<map.size()<<endl;

        if(map.size()>0)
        {

            for(int u=0;u!=map.size();++u)
            {

                Feature localTemp;

                RobotState test;



                ConvertWorldToRobot(map[u],robotState,localTemp);

             //   mapper.DrawLine(localTemp,RobotState(),cv::Scalar(0,255,0),3,cv::Point2d(250,250),1,imgg);



                cout<<"²Ä"<<u<<"­Ó¯S¼x"<<endl;
                cout<<"¥[¤J«e"<<endl;
                cout<<"¾÷¾¹¤H¦ì¸m"<<robotCombinedState<<endl;
                cout<<"¾÷¾¹¤H¦@ÅÜ²§¾ð¯x°}"<<robotCombinedCovariance<<endl;

                AddNewLandmark(localTemp);

                cout<<"¥[¤J«á"<<endl;
                cout<<"¾÷¾¹¤H¦ì¸m"<<robotCombinedState<<endl;
                cout<<"¾÷¾¹¤H¦@ÅÜ²§¾ð¯x°}"<<robotCombinedCovariance<<endl;

                //cv::Point2d obsXY;
                ////PolarToCartesian(cv::Point2d(map[u].featureMean.ptr<double>(0)[0],map[u].featureMean.ptr<double>(1)[0]),obsXY);
                //obsXY.x=obsXY.x/100.0*(1/0.15);
                //obsXY.y=obsXY.y/100.0*(1/0.15);
                //cv::circle(imgg, cv::Point(obsXY.x+250,obsXY.y+250), (i), cv::Scalar(255,255,0), -1  );


            }

        }

    }

    cv::imshow("123",imgg);
    cv::waitKey(1);

    cout<<"ªì©l¤Æ§ä¨ìªº¯S¼x¼Æ¥Ø:"<<landmarkNum<<" "<<landmarkSets.size()<<endl;
}

void lab405::MyEFKSLAM::SetRobotPose(const RobotState &robot)
{
    robotState.robotPositionMean = robot.robotPositionMean.clone();
    robotCombinedState.ptr<double>(0)[0] = robot.robotPositionMean.ptr<double>(0)[0];
    robotCombinedState.ptr<double>(1)[0] = robot.robotPositionMean.ptr<double>(1)[0];
    robotCombinedState.ptr<double>(2)[0] = robot.robotPositionMean.ptr<double>(2)[0];
}

void lab405::MyEFKSLAM::SetMotionCommand()
{
    double value = 0;
    int velocity = this->GetVelocity();
    //RightMotor,LeftMotor);

//    if(checkBit!=true)
//    {

//        SLAM_Robot->rightMotor->Stop();
//        SLAM_Robot->leftMotor->Stop();
//        return;

//    }


    if(!this->commandSets.empty())
    {
//        collector->SetCommandFlag(true);
       // cout<<"!!"<<commandSets.front().second<<endl;
        if(this->commandSets.front().first==1)
        {

            //SLAM_Robot->rightMotor->SetVelocity(0);
            //SLAM_Robot->leftMotor->SetVelocity(0);
          //  commandSets.front().second
            //[encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:325]
            value = (4096*3.333*14)*(this->commandSets.front().second/32.5)/(CV_PI);
            std::cout << "forward: " << value << std::endl;
            this->myrobot->left_dcmotor->SetVelocity(velocity);
            this->myrobot->right_dcmotor->SetVelocity(-velocity);



        }
        else
        {
            double arc=2*CV_PI*this->GetRobotWidth()*(this->commandSets.front().second/360.0);
             //double arc=2*CV_PI*SLAM_Robot->GetRobotWidth()*((SLAM_Robot->commandSets.front().second*30.0/45.0)/360.0);
            // cout<<angle<<endl;
            value=(4096*3.333*14)*(arc/32.5)/(CV_PI);

            if(this->commandSets.front().first==2) //right
            {
                //SLAM_Robot->rightMotor->SetVelocity(-velocity);
                this->myrobot->right_dcmotor->SetVelocity(-30);
                this->myrobot->left_dcmotor->SetVelocity(0);

            }
            else if(this->commandSets.front().first==3) //left
            {
                this->myrobot->right_dcmotor->SetVelocity(0);
                //SLAM_Robot->leftMotor->SetVelocity(velocity);
                this->myrobot->left_dcmotor->SetVelocity(30);

            }
            value = abs(value); //test
            std::cout << "angle: " << value << std::endl;

        }
//        ui->textBrowser_2->setFontWeight( QFont::DemiBold );
//        ui->textBrowser_2->setTextColor( QColor( "red" ) );
//        ui->textBrowser_2->append("mode:"+QString::number(SLAM_Robot->commandSets.front().first)+ " value:"+QString::number(SLAM_Robot->commandSets.front().second));


        this->commandSets.front().second=value;
//        collector->SetMotionCommand(this->commandSets.front().first,SLAM_Robot->commandSets.front().second);
        this->commandSets.pop();

    }
    else
    {
//        collector->SetCommandFlag(false);
        this->myrobot->right_dcmotor->Stop();
        this->myrobot->left_dcmotor->Stop();
    }
}

void lab405::MyEFKSLAM::SetMotionCommand2(int type, double value)
{
    motionType=type;

    if(type==1||type==2)
        rightCommand+=value;

    if(type==1||type==3)
        leftCommand+=value;
}

void lab405::MyEFKSLAM::ControlMotion()
{
//    std::cout << "ControlMotion" << std::endl;
//    cv::Point2d diff = current_end - current_start;
//    if (diff.y < 0)
//    {
//        myrobot->left_dcmotor->SetVelocity(-30);
//        myrobot->right_dcmotor->SetVelocity(0);
//    }
//    else if (diff.y > 0)
//    {
//        myrobot->left_dcmotor->SetVelocity(0);
//        myrobot->right_dcmotor->SetVelocity(30);
//    }
//    else
//    {
//        myrobot->left_dcmotor->SetVelocity(-30);
//        myrobot->right_dcmotor->SetVelocity(30);
//    }


//    std::cout << "commandSets.front().first = " << commandSets.front().first << std::endl;
//    std::cout << "commandSets.size() = " << commandSets.size() << std::endl;
//    switch (commandSets.front().first)
//    {
//    // 1: go forward, 2: turn right, 3: turn left
//    case 1:
//        myrobot->right_dcmotor->RotateRelativeDistancce(10000);
//        myrobot->left_dcmotor->RotateRelativeDistancce(-10000);
////        myrobot->right_dcmotor->SetVelocity(100);
////        myrobot->left_dcmotor->SetVelocity(-100);
//        break;
//    case 2:
//        myrobot->right_dcmotor->RotateRelativeDistancce(0);
//        myrobot->left_dcmotor->RotateRelativeDistancce(-10000);
////        myrobot->left_dcmotor->SetVelocity(-100);
////        myrobot->right_dcmotor->SetVelocity(0);
//        break;
//    case 3:
//        myrobot->right_dcmotor->RotateRelativeDistancce(10000);
//        myrobot->left_dcmotor->RotateRelativeDistancce(0);
////        myrobot->left_dcmotor->SetVelocity(0);
////        myrobot->right_dcmotor->SetVelocity(100);
//        break;
//    }
//    commandSets.pop();

    if (robotPathSets.size() > 1)
    {
        cv::Point Dir = robotPathSets.at(1) - this->currentStart;
        std::cout << "Dir = " << Dir << std::endl;

        if (Dir.x == 1 && Dir.y == 0)
        {
            // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
            // calibration coeffient: y = 0.9487x
            int value = static_cast<int>((4096*3.333*14)*(gridMapper.GetPixel_meterFactor()/0.325)/(pi)/0.9487);
            myrobot->right_dcmotor->RotateRelativeDistancce(value);
            myrobot->left_dcmotor->RotateRelativeDistancce((-1)*value);
        }

    }


}


void lab405::MyEFKSLAM::PathSmoothing()
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

void lab405::MyEFKSLAM::TrajectoryGenerationSmoothing()
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
        else
        {
            temp.first=1;  //
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

//        //command: forward
//        temp.first=1;  //grid map size determine the distance of each grid
//        temp.second=(int)(r*gridMapper.GetPixel_meterFactor()*100);  //m->cm
//        commandSets.push(temp);


        //cout<<r<<" "<<angle-preangle<<endl;
        preangle=angle;

    }
}

void lab405::MyEFKSLAM::NavigationUpdate(bool scene)
{

    myrobot->right_dcmotor->Stop();
//    std::cout << "right stopped!" << std::endl;
    myrobot->left_dcmotor->Stop();
//    std::cout << "left stopped!" << std::endl;
//    emit MotorStop();


    int Pos_Stop_error = 1;
    int fir_pos = myrobot->right_dcmotor->GetPose();
    int sec_pos = myrobot->right_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
//        std::cout << sec_pos << " " << fir_pos << std::endl;
        fir_pos = sec_pos;
        sec_pos = myrobot->right_dcmotor->GetPose();
    }
//    double ReadingR = sec_pos;
    odoValueCurrent.x = sec_pos;
//    std::cout << "odoValueCurrent.x = " << odoValueCurrent.x << std::endl;

    fir_pos = myrobot->left_dcmotor->GetPose();
    sec_pos = myrobot->left_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
//        std::cout << sec_pos << " " << fir_pos << std::endl;
        fir_pos = sec_pos;
        sec_pos = myrobot->left_dcmotor->GetPose();
    }
//    double ReadingL = sec_pos;
    odoValueCurrent.y = sec_pos;
//    std::cout << "odoValueCurrent.y = " << odoValueCurrent.y << std::endl;


    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // reverse of static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi));
    double DeltaR = abs(odoValueCurrent.x - odoValuePrevious.x)*CV_PI*32.5*0.9487/(4096*3.333*14);  // unit cm
    double DeltaL = abs(odoValueCurrent.y - odoValuePrevious.y)*CV_PI*32.5*0.9487/(4096*3.333*14);
    std::cout << "DeltaR = " << DeltaR << ", DeltaL = " << DeltaL << std::endl;

    odoValuePrevious.x = odoValueCurrent.x;
    odoValuePrevious.y = odoValueCurrent.y;

//    readFlag=true;
//    double DeltaRight=((double)((odoValueCurrent.x-odoValuePrevious.x)*32.5*CV_PI)/(4096*14.0*3.33333));
//    double DeltaLeft=((double)((odoValueCurrent.y-odoValuePrevious.y)*32.5*CV_PI)/(4096*14.0*3.33333));



//    ui->textBrowser->setFontWeight( QFont::Normal );
//    ui->textBrowser->setTextColor( QColor( "blue" ) );
//    ui->textBrowser->append("[encoder]:("+QString::number(DeltaRight)+","+QString::number(DeltaLeft)+")");
//    cout<<"3:"<<odoValueCurrent.x<<" "<<odoValueCurrent.y<<" time:"<<odoValuePrevious.x<<" , "<<odoValuePrevious.y<<endl;
//    cout<<"3:"<<DeltaRight<<" "<<DeltaLeft<<" time:"<<endl;

//    odoValuePrevious.x=odoValueCurrent.x;
//    odoValuePrevious.y=odoValueCurrent.y;



    ////////////////////////////////////////////////////////////////////////
    //feature extraction
//    std::vector<cv::Point2d> points;
//    std::vector<Line> line;
//    std::vector<Corner> cor;
    std::vector<double> LaserRawData;
    std::vector<cv::Point2d> LaserCatesianPoints;
    std::vector<Line> line;
    std::vector<Corner> cor;
    myrobot->laser_slam->GetOneLaserData(LaserRawData);


    ////////////////////////////////////////////////////////////////////
    // saving
    saveFileIndex++;
    robotOutputFile << saveFileIndex << " " << odoValueCurrent.x << " " << odoValueCurrent.y << endl;  //encoder
    laserOutputFile << saveFileIndex << " ";
    for(int i = 0; i != LaserRawData.size(); ++i)
        laserOutputFile << LaserRawData[i] << " ";
    ////////////////////////////////////////////////////////////////////
    laserOutputFile << std::endl;

    this->mapper.RangesDataToPointsData(LaserRawData,LaserCatesianPoints);
    this->lineExtracter.SplitAndMerge(LaserCatesianPoints,line);


    ////////////////////////////////////////////////////////////////////////
    //motion estimation
    //cout<<DeltaRight<<" "<<odoValueCurrent.x<<" "<<odoValuePrevious.x<<endl;DeltaLeft
    // Prediction
    this->MotionPrediction(DeltaR, DeltaL);

    //SLAM_Robot->gridMapper.InsertLocalGridMap();

    this->DataAssociationAndUpdate(line,cor);
    // Get Robot State
    this->robotPosition = this->robotState;
//    myekfslam->myrobot->robotPosition=myekfslam->myrobot->EKFRuner.GetRobotPose();


//    std::cout << "after data ass" << std::endl;
//    // QMessageBox::information(this, "Error!", "ok1!");

//    ////////////////////////////////////////////////////////////////////////
//    //ICP correction
//    vector<cv::Point2d> temp;
//    dataConvertRobotToWorld(points, this->robotPosition, temp);
//    double percent=0.5;
//    cv::Point3d adjustPose=SLAM_Robot->icper.Align(SLAM_Robot->refenceMap,temp,percent);

    cv::Point3d adjustPose;
    adjustPose.x=0;
    adjustPose.y=0;
    adjustPose.z=0;
    ////////////////////////////////////////////////////////////////////////
    // robot pose after adjust
    RobotState RobotStateAfterAdjust;

    RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0] = adjustPose.x + this->robotPosition.robotPositionMean.ptr<double>(0)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0] = adjustPose.y + this->robotPosition.robotPositionMean.ptr<double>(1)[0];
    RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0] = adjustPose.z + this->robotPosition.robotPositionMean.ptr<double>(2)[0];
    RobotStateAfterAdjust.robotPositionCovariance = this->robotPosition.robotPositionCovariance.clone();

    // xyz project to plane
    cv::Point2d imagePose = this->gridMapper.GetRobotCenter(RobotStateAfterAdjust);

    // convert to unit: m
    this->currentStart.x = imagePose.x;
    this->currentStart.y = imagePose.y;

    // set into combine state
    this->SetRobotPose(RobotStateAfterAdjust);
    ///////////////////////////////////// ///////////////////////////////////


    // add new map
    // Rotate Laser Points
    for(int k = 0; k != LaserCatesianPoints.size(); ++k)
    {
        double x = LaserCatesianPoints[k].x*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                - LaserCatesianPoints[k].y*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0];
        double y = LaserCatesianPoints[k].x*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + LaserCatesianPoints[k].y*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])
                + RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0];
        //   cv::circle(imgICP, cv::Point(5*x+400,5*y+400), 1, cv::Scalar(255,0,0), -1  );
        this->refenceMap.push_back(cv::Point2d(x,y));
        //  cv::circle(imgICP, cv::Point(x/100.0*(1.0/0.05)+500,y/100.0*(1.0/0.05)+500), 1, cv::Scalar(255,0,0), -1  );

    }

    ////////////////////////////////////////////////////////////////////////
    //mapping process
    // Raw Catesian Map
    this->mapper.InsertLocalLandmarkMap(LaserCatesianPoints, RobotStateAfterAdjust);
    landMarkImg = this->mapper.GetLandmarkMap();

    // Grid Map
    this->gridMapper.InsertLocalGridMap(LaserRawData, RobotStateAfterAdjust);
    this->gridMapper.GetOccupancyGridMap(OccupancyGridMapImg);
    // draw on landmarkimg
    this->mapper.DrawRobotPoseWithErrorEllipse(RobotStateAfterAdjust, landMarkImg, true);

    cv::imshow("Grid Img", OccupancyGridMapImg);
    cv::waitKey(10);

    ////////////////////////////////////////////////////////////////////////
    //path planning
    cv::Point2d tempEnd;
    cv::Mat plannignGridMap;
    double search_rect = 80;
    this->planner.SetGridMap(OccupancyGridMapImg, 1);
    this->planner.GetPathPlanningMap(plannignGridMap);
//    cv::circle(plannignGridMap, this->currentStart, 2, cv::Scalar(255,255,255), -1 );
    this->FindCurrentNodeEnd(plannignGridMap, search_rect, this->currentStart, this->GoalEnd, tempEnd);

    std::cout << "temp end = " << tempEnd << std::endl;
//    double rstart=sqrt(pow(this->currentStart.x,2.0) +pow(this->currentStart.y,2.0)  );
//    double rend=sqrt(pow(this->GoalEnd.x,2.0) +pow(this->GoalEnd.y,2.0)  );
//    double rFinal=sqrt(pow(this->FinalEnd.x,2.0) +pow(this->FinalEnd.y,2.0)  );
//    double rTempEnd=sqrt(pow(tempEnd.x,2.0) +pow(tempEnd.y,2.0)  );

    cv::Mat color_plannignGridMap;
    cv::cvtColor(plannignGridMap,color_plannignGridMap,CV_GRAY2BGR);


//    //////////////////////////////////////////////
//    //save file
//    robotOutputFile<<DeltaRight<<" "<<DeltaLeft<<endl;
//    for(int i=0;i!=myekfslam->myrobot->rawLaserScanData.size();++i)
//        laserOutputFile<<myekfslam->myrobot->rawLaserScanData[i]<<" ";

//    laserOutputFile<<endl;//´«¦æ


//    //////////////////////////////////////////////

//    if(sceneCnt==sceneNum)  //»`¶°¨ì³õ´º¼Æ
//    {
//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);
//        testEKFTimer->stop();
//        scenesTimer->stop();
//        ui->textBrowser->setFontWeight( QFont::Normal );
//        ui->textBrowser->setTextColor( QColor( "blue" ) );
//        ui->textBrowser->append("[System Message]:Motor stop!");
//        return;
//    }

//    if(robotpath1.robotPositionMean.ptr<double>(0)[0] >= gridDistance*sceneCnt)
//        //if(robotpath1.robotPositionMean.ptr<double>(0)[0]>=sceneCnt*5)
//    {

//        checkBit=false;

//        myekfslam->myrobot->left_dcmotor->Stop();
//        myekfslam->myrobot->right_dcmotor->Stop();

//        collector->SetStopped(true);

//        motionMode=true;
//        ui->pushButton_2->click();

//        testEKFTimer->stop();
//        sceneCnt++;
//        //QMessageBox::information(this,"dd","ff");
//        cout<<"scen:"<<sceneCnt<<"  robotpose:"<<robotpath1.robotPositionMean<<"   index:"<<saveFileIndex<<endl;
//        robotSceneFile<<sceneCnt<<" "<<robotpath1.robotPositionMean.ptr<double>(0)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(1)[0]<<" "<<robotpath1.robotPositionMean.ptr<double>(2)[0]<<" "<<saveFileIndex<<endl;
//        return;
//    }





    //if( (rend<=rsrart))  //¥b®|¤j©ó©Îµ¥©ó2*pixelFactor;


//    if( rstart <= rTempEnd && temp_threscont - thresh_count >= this->commandSets.size())
//    {
        this->commandSets = std::queue<std::pair<int,double>> ();
//        this->planner.SetStartNode(this->currentStart);

//        do
//        {
    cv::Point2d rs_end(155, 500);

//            this->planner.SetEndNode(tempEnd);
//            this->planner.AStartPlanning(this->robotPathSets);
//            planner.run(this->currentStart, tempEnd, this->robotPathSets);
    planner.run(this->currentStart, rs_end, this->robotPathSets);

            //  QMessageBox::information(this, "Error!", "2_2!!"+QString::number(SLAM_Robot->robotPathSets.size()));
            //  cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,255,0), 2  );

            //  cout<<"tempEnd1:"<<tempEnd<<endl;
            //  cv::imshow("show",color_plannignGridMap);
            //  cv::waitKey(1);


            // QMessageBox::information(this, "Error!", "2_1_1");


            if(this->robotPathSets.size() != 0)
            {
                this->PathSmoothing();

                this->TrajectoryGenerationSmoothing();
                temp_threscont = this->commandSets.size();


            }
            else
            {
                tempEnd.x = this->currentStart.x + 10;
                tempEnd.y = this->currentStart.y;
            }
            //cv::circle(color_plannignGridMap,tempEnd, 4, cv::Scalar(0,0,255), 2  );
            // cout<<"tempEnd2:"<<tempEnd<<endl;

            //  cv::imshow("show",color_plannignGridMap);
            //  cv::waitKey(1);
            // QMessageBox::information(this, "Error!", "2_1_2");


//        } while (this->robotPathSets.size() == 0);

        // control
//        this->SetMotionCommand();
//        ControlMotion();

        /*
        if(SLAM_Robot->robotPathSets.size()!=0)
        {
            SLAM_Robot->PathSmoothing();

            SLAM_Robot->TrajectoryGenerationSmoothing();
            threcont=SLAM_Robot->commandSets.size();

            this->SetMotionCommand();
        }
*/
//        ui->textBrowser->append("=====================");
//        ui->textBrowser->append("[robotPathSets size]:"+QString::number(myekfslam->myrobot->robotPathSets.size()));
//        for(int i = 0;i != this->robotPathSets.size();++i)
//        {

//            cv::circle(color_plannignGridMap, cv::Point(this->robotPathSets[i].x, this->robotPathSets[i].y), 4 , cv::Scalar(0,0 ,255), -1  );
//            ui->textBrowser->setFontWeight( QFont::DemiBold );
//            ui->textBrowser->setTextColor( QColor( "red" ) );
//            ui->textBrowser->append("[robotPathSets]:"+QString::number(myekfslam->myrobot->robotPathSets[i].x)+" "+QString::number(myekfslam->myrobot->robotPathSets[i].y));

//        }
//        ui->textBrowser->append("=====================");
//    }







    // else
    // ui->textBrowser->append("=====no path fuck========");


//        for (int i = 0; i < this->robotPathSets.size(); i++)
//        {
//            std::cout << "x = " << this->robotPathSets.at(i).x
//                      << ", y = " << this->robotPathSets.at(i).y << std::endl;
//        }
        std::cout << "this->currentStart " << this->currentStart << std::endl;
        cv::Mat show = color_plannignGridMap.clone();
        cv::circle(show, this->currentStart, 1, cv::Scalar(0, 0, 255));
        cv::circle(show, rs_end, 1, cv::Scalar(255, 0, 0));
        cv::imshow("show", show);
        cv::waitKey(1);

//    cv::line(color_plannignGridMap, tempEnd, this->GoalEnd, cv::Scalar(0,0,255), 7,CV_AA);



//    std::cout << "this->currentStart = " << this->currentStart << std::endl;
//    std::cout << "this->currentEnd = " << this->currentEnd << std::endl;
//    std::cout << "tempEnd = " << tempEnd << std::endl;

//    ControlMotion(this->currentStart, tempEnd);

        if (saveFileIndex > 4)
        {
            cv::line(color_plannignGridMap, this->pre_start, this->currentStart, cv::Scalar(0,0,255), 7,CV_AA);
            cv::circle(color_plannignGridMap, this->pre_start, 4, cv::Scalar(255,0,0), 2  );
        }
    cv::circle(color_plannignGridMap, this->currentStart, 4, cv::Scalar(0,255,0), 2  );

    cv::line(color_plannignGridMap, this->currentStart,
             cv::Point(this->currentStart.x +
                       10*cos(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0]),
             this->currentStart.y +
             10*sin(RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0])), cv::Scalar(0,255,0), 2,CV_AA);
    if (saveFileIndex > 4)
    {
        cv::line(color_plannignGridMap, this->pre_start, cv::Point(pre_start.x + 10*cos(pre_theta), pre_start.y + 10*sin(pre_theta)), cv::Scalar(255,0,0), 2,CV_AA);
    }

    this->pre_theta = RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0];
    this->pre_start = this->currentStart;

//    cv::circle(color_plannignGridMap, this->GoalEnd, 4, cv::Scalar(0,255,0), 2  );
//    cv::circle(color_plannignGridMap, tempEnd, 4, cv::Scalar(0,255,0), 2  );

//    clock_t endTime=clock();
//    double total=(double)(endTime-startTime)/CLK_TCK;

//    ui->textBrowser_slam->setFontWeight( QFont::DemiBold );
//    ui->textBrowser_slam->setTextColor( QColor( "red" ) );
//    ui->textBrowser_slam->append("[pose]:" + QString::number(robotpath1.robotPositionMean.ptr<double>(0)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(1)[0]) + " , " + QString::number(robotpath1.robotPositionMean.ptr<double>(2)[0]));

    // saving
    robotStateFile
            << saveFileIndex
            << " " << RobotStateAfterAdjust.robotPositionMean.ptr<double>(0)[0]
            << " " << RobotStateAfterAdjust.robotPositionMean.ptr<double>(1)[0]
            << " " << RobotStateAfterAdjust.robotPositionMean.ptr<double>(2)[0]
            << " " << scene << std::endl;

    std::cout << "[Robot pose]:" << RobotStateAfterAdjust.robotPositionMean << std::endl;

//    std::cout << "single step Time:" << total << std::endl;
    cv::imshow("gridImg",OccupancyGridMapImg);
    cv::imshow("landMarkImg",landMarkImg);
    cv::imshow("color_plannignGridMap",color_plannignGridMap);
    cv::imshow("plannignGridMap",plannignGridMap);
//    static int count=0;
    navi_count++;
    QString name="Img/"+QString::number(navi_count)+".jpg";
    cv::imwrite(name.toStdString(),color_plannignGridMap);
    cv::imwrite("Img/plannignGridMap.jpg",plannignGridMap);
    cv::imwrite("Img/gridImg.jpg",OccupancyGridMapImg);
    cv::imwrite("Img/landMarkImg.jpg",landMarkImg);
//    count++;

    cv::waitKey(1);




    //    //if(motionMode==true)
}

void lab405::MyEFKSLAM::NavigationInitial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold, const string &filename_tPoints)
{
    //    odoValueCurrent = cv::Point2d(0.0, 0.0); // x:right odo y:left odo
        // shih's EKF slam
        // EKFtimer flag
        checkBit = true;
        motionMode = false;
        sceneCnt = 0;
        saveFileIndex = 0;
        navi_count = 0;
        // number of scenes
        sceneNum = _sceneNum;

        thresh_count = threshold;


        tP_count = 0;
        std::string filename = "./" + filename_tPoints + ".txt";
        std::ifstream infile_tP (filename);
        if (infile_tP.is_open())
        {
            infile_tP >> num_tP;
            for (int i = 0; i < num_tP; i++)
            {
               int temp_x, temp_y;
               infile_tP >> temp_x >> temp_y;
               cv::Point temp (temp_x, temp_y);
               tP_set.push_back(temp);
            }
        }

        slam_x = _slam_x;
        slam_x0 = _slam_x0;
        slam_y = _slam_y;

        gridDistance = abs(slam_x - slam_x0)/(sceneNum - 1);
        gridDistance = gridDistance*this->gridMapper.GetPixel_meterFactor()*100;
        std::cout << "gridDistance:" << gridDistance << std::endl;

        // intial motor
        this->myrobot->left_dcmotor->SetHome();
        this->myrobot->right_dcmotor->SetHome();
        this->refenceMap.clear();

        this->myrobot->laser_slam->DisconnectReadyRead();

        // first,  extract features from environment


    //    std::cout << laserdata.size() << std::endl;
    //    for (int i = 0; i < laserdata.size(); i++)
    //    {
    //        std::cout << laserdata.at(i) << " ";
    //    }
    //    std::cout << std::endl;

        robotOutputFile.open("20150525_EKF_odoFile.txt");
        laserOutputFile.open("20150525_EKF_laserFile.txt");
        robotSceneFile.open("20150525_EKF_sceneData.txt");
        robotStateFile.open("20150525_EKF_robotState.txt");

        vector<vector<Line>> lines;
        CornerExtraction cornerEx;
        // repeat features
        for(int i = 0; i != 4; ++i)  // 6
        {
            std::vector<cv::Point2d> temp1;
            std::vector<Line> temp2;
            std::vector<Corner> cor;
            // triiger one scan

            vector<double> laserdata;
            this->myrobot->laser_slam->GetOneLaserData(laserdata);
    //        std::cout << "temp size = " << temp.size() << std::endl;
    //        for (int j = 0; j < temp.size(); j++)
    //        {
    //            std::cout << temp.at(j) << " ";
    //        }
    //        std::cout <<std::endl;

    //        SLAM_Robot->laserS200->ReadData(temp);
            if(laserdata.size()==0||i==0)
                continue;
            this->mapper.RangesDataToPointsData(laserdata, temp1);

            this->lineExtracter.SplitAndMerge(temp1, temp2);
            cornerEx.ExtractCorners(laserdata,cor);

            std::cout << "laser points:" << laserdata.size() << "  line num:" << temp2.size() << "  cor num:" << cor.size() << std::endl;
            cv::Mat img = this->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

            cv::Mat img1 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
            cv::Mat img2 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


            lines.push_back(temp2);
            this->gridMapper.InsertLocalGridMap(laserdata, cv::Point3d(0,0,0));

            //////////////////////////////////////////////
            //save file
            saveFileIndex++;
            robotOutputFile << saveFileIndex<< " " << 0 << " " << 0 << endl;  //encoder
            laserOutputFile << saveFileIndex << " ";
            for(int i = 0; i != laserdata.size(); ++i)
                laserOutputFile << laserdata[i] << " ";

            laserOutputFile << endl;


            //////////////////////////////////////////////
            //for(int i=0;i!=temp.size();++i)
            // {
            //  if(temp[i]<=2996)
            //      cout<<temp[i]<<endl;
            //}

            //  cv::Mat tt;
            // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
            // cv::imshow("tt",tt);
            cv::imshow("1341",img);
            cv::imshow("13411",img1);
            cv::imshow("13411222 ",cornerEx.cornerImg);
    //        while(cv::waitKey(10) != 27)
    //        {
    //        }
        }

        this->MapInitial(lines);
        std::cout << "EKF Intial" <<endl;
        this->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
        this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
        this->robotPosition.robotPositionMean.ptr<double>(2)[0]=0;

        robotStateFile << saveFileIndex
                       << " " << this->robotPosition.robotPositionMean.ptr<double>(0)[0]
                       << " " << this->robotPosition.robotPositionMean.ptr<double>(1)[0]
                       << " " << this->robotPosition.robotPositionMean.ptr<double>(2)[0]
                       << " " << 1 << std::endl;

        //set start end points
        this->currentStart.x = this->gridMapper.GetGridMapOriginalPoint().x;
        this->currentStart.y = this->gridMapper.GetGridMapOriginalPoint().y;

        this->FinalEnd.x = this->gridMapper.GetGridMapOriginalPoint().x + 200;
        this->FinalEnd.y = this->gridMapper.GetGridMapOriginalPoint().y;


        this->pre_start = this->gridMapper.GetGridMapOriginalPoint();

        this->GoalEnd.x = _slam_x;
        this->GoalEnd.y = _slam_y;

        // initial
        this->odoValuePrevious = cv::Point2d(0.0, 0.0);

//        this->EKFTimer->start(1000);

        cv::destroyAllWindows();
}

void lab405::MyEFKSLAM::PControlInitial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y, double threshold/*??*/, const string &filename_tPoints)
{
    //    odoValueCurrent = cv::Point2d(0.0, 0.0); // x:right odo y:left odo
        // shih's EKF slam
        // EKFtimer flag
        checkBit = true;
        motionMode = false;
        sceneCnt = 0;
        saveFileIndex = 0;
        // number of scenes
        sceneNum = _sceneNum;

        thresh_count = threshold;


        tP_count = 0;
        std::string filename = "./" + filename_tPoints + ".txt";
        std::ifstream infile_tP (filename);
        if (infile_tP.is_open())
        {
            infile_tP >> num_tP;
            for (int i = 0; i < num_tP; i++)
            {
               int temp_x, temp_y;
               infile_tP >> temp_x >> temp_y;
               cv::Point temp (temp_x, temp_y);
               tP_set.push_back(temp);
            }
        }

        slam_x = _slam_x;
        slam_x0 = _slam_x0;
        slam_y = _slam_y;

        gridDistance = abs(slam_x - slam_x0)/(sceneNum - 1);
        gridDistance = gridDistance*this->gridMapper.GetPixel_meterFactor()*100;
        std::cout << "gridDistance:" << gridDistance << std::endl;

        // intial motor
        this->myrobot->left_dcmotor->SetHome();
        this->myrobot->right_dcmotor->SetHome();
        this->refenceMap.clear();

        this->myrobot->laser_slam->DisconnectReadyRead();

        // first,  extract features from environment


    //    std::cout << laserdata.size() << std::endl;
    //    for (int i = 0; i < laserdata.size(); i++)
    //    {
    //        std::cout << laserdata.at(i) << " ";
    //    }
    //    std::cout << std::endl;

        robotOutputFile.open("20150503_EKF_odoFile.txt");
        laserOutputFile.open("20150503_EKF_laserFile.txt");
        robotSceneFile.open("20150503_EKF_sceneData.txt");

        vector<vector<Line>> lines;
        CornerExtraction cornerEx;
        // repeat features
        for(int i = 0; i != 4; ++i)  // 6
        {
            std::vector<cv::Point2d> temp1;
            std::vector<Line> temp2;
            std::vector<Corner> cor;
            // triiger one scan

            vector<double> laserdata;
            this->myrobot->laser_slam->GetOneLaserData(laserdata);
    //        std::cout << "temp size = " << temp.size() << std::endl;
    //        for (int j = 0; j < temp.size(); j++)
    //        {
    //            std::cout << temp.at(j) << " ";
    //        }
    //        std::cout <<std::endl;

    //        SLAM_Robot->laserS200->ReadData(temp);
            if(laserdata.size()==0||i==0)
                continue;
            this->mapper.RangesDataToPointsData(laserdata, temp1);

            this->lineExtracter.SplitAndMerge(temp1, temp2);
            cornerEx.ExtractCorners(laserdata,cor);

            std::cout << "laser points:" << laserdata.size() << "  line num:" << temp2.size() << "  cor num:" << cor.size() << std::endl;
            cv::Mat img = this->mapper.GetLocalLandmarkMap(temp1,temp2,vector<Corner>());

            cv::Mat img1 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),vector<Corner>());
            cv::Mat img2 = this->mapper.GetLocalLandmarkMap(temp1,std::vector<Line>(),cor);


            lines.push_back(temp2);
            this->gridMapper.InsertLocalGridMap(laserdata, cv::Point3d(0,0,0));

            //////////////////////////////////////////////
            //save file
            saveFileIndex++;
            robotOutputFile << 0 << " " << 0 << endl;  //encoder
            for(int i = 0; i != laserdata.size(); ++i)
                laserOutputFile << laserdata[i] << " ";

            laserOutputFile << endl;


            //////////////////////////////////////////////
            //for(int i=0;i!=temp.size();++i)
            // {
            //  if(temp[i]<=2996)
            //      cout<<temp[i]<<endl;
            //}

            //  cv::Mat tt;
            // SLAM_Robot->gridMapper.GetLocalGridMap(temp,tt);
            // cv::imshow("tt",tt);
            cv::imshow("1341",img);
            cv::imshow("13411",img1);
            cv::imshow("13411222 ",cornerEx.cornerImg);
    //        while(cv::waitKey(10) != 27)
    //        {
    //        }
        }

        this->MapInitial(lines);
        std::cout << "EKF Intial" <<endl;
        this->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
        this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
        this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
        //set start end points
        this->currentStart.x = this->gridMapper.GetGridMapOriginalPoint().x;
        this->currentStart.y = this->gridMapper.GetGridMapOriginalPoint().y;

        this->FinalEnd.x = this->gridMapper.GetGridMapOriginalPoint().x + 200;
        this->FinalEnd.y = this->gridMapper.GetGridMapOriginalPoint().y;


        this->GoalEnd.x = _slam_x;
        this->GoalEnd.y = _slam_y;

        // initial
        this->odoValuePrevious = cv::Point2d(0.0, 0.0);

        this->PcontrolTimer->start(1000);

        cv::destroyAllWindows();
}

void lab405::MyEFKSLAM::MotionControl(const cv::Point2d &para)
{
    double diff_theta = para.y - CV_PI/2;
    std::cout << "diff_theta: " << diff_theta << std::endl;

    double diff_r = para.x - 20;
    std::cout << "diff_r: " << diff_r << std::endl;

    if (diff_r > 2) // toward right move
    {
        double diff_r_theta = para.y - (CV_PI/2 - CV_PI/16);
        if (diff_r_theta > 0.2) // turn right
        {
//            std::cout << "Turn Right" << std::endl;
            myrobot->right_dcmotor->RotateRelativeDistancce(0);
            myrobot->left_dcmotor->RotateRelativeDistancce(-3000);
        }
        else if (diff_r_theta < -0.2)
        {
            myrobot->right_dcmotor->RotateRelativeDistancce(3000);
            myrobot->left_dcmotor->RotateRelativeDistancce(0);
        }
        else
        {
            myrobot->right_dcmotor->RotateRelativeDistancce(5000);
            myrobot->left_dcmotor->RotateRelativeDistancce(-5000);
        }

    }
    else if (diff_r < -2)
    {
        double diff_r_theta = para.y - (CV_PI/2 + CV_PI/16);
        if (diff_r_theta > 0.2) // turn right
        {
//            std::cout << "Turn Right" << std::endl;
            myrobot->right_dcmotor->RotateRelativeDistancce(0);
            myrobot->left_dcmotor->RotateRelativeDistancce(-3000);
        }
        else if (diff_r_theta < -0.2)
        {
            myrobot->right_dcmotor->RotateRelativeDistancce(3000);
            myrobot->left_dcmotor->RotateRelativeDistancce(0);
        }
        else
        {
            myrobot->right_dcmotor->RotateRelativeDistancce(5000);
            myrobot->left_dcmotor->RotateRelativeDistancce(-5000);
        }
    }
    else
    {
        if (diff_theta > 0.2) // turn right
        {
            std::cout << "Turn Right" << std::endl;
            myrobot->right_dcmotor->RotateRelativeDistancce(0);
            myrobot->left_dcmotor->RotateRelativeDistancce(-1000);
        }
        else if (diff_theta < -0.2)   // turn left
        {
            std::cout << "Turn Left" << std::endl;
            myrobot->right_dcmotor->RotateRelativeDistancce(1000);
            myrobot->left_dcmotor->RotateRelativeDistancce(0);
        }
        else
        {
            myrobot->right_dcmotor->RotateRelativeDistancce(5000);
            myrobot->left_dcmotor->RotateRelativeDistancce(-5000);
        }
    }


//    if (robotPathSets.size() > 1)
//    {
//        cv::Point Dir = robotPathSets.at(1) - this->currentStart;
//        std::cout << "Dir = " << Dir << std::endl;

//        if (Dir.x == 1 && Dir.y == 0)
//        {
//            // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
//            // calibration coeffient: y = 0.9487x
//            int value = static_cast<int>((4096*3.333*14)*(gridMapper.GetPixel_meterFactor()/0.325)/(pi)/0.9487);
//            myrobot->right_dcmotor->RotateRelativeDistancce(value);
//            myrobot->left_dcmotor->RotateRelativeDistancce((-1)*value);
//        }

//    }
}

void lab405::MyEFKSLAM::FindCurrentNodeEnd(const cv::Mat &gridMap, double intervalDistance, const cv::Point2d &currentStart, const cv::Point2d &goal, cv::Point2d &currentEnd)
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

