#include "myefkslam.h"

lab405::MyEFKSLAM::MyEFKSLAM(double w, int velocity, const cv::Point2d &point) :
    myrobot(new MyRobot()), EKFTimer(new QTimer),
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

}

void lab405::MyEFKSLAM::Initial(std::size_t _sceneNum, double _slam_x0, double _slam_x, double _slam_y)
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
    gridDistance = abs(_slam_x - _slam_x0)/(sceneNum - 1);
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
        while(cv::waitKey(10) != 27)
        {
        }
    }

    this->EKFRuner.Initial(lines);
    std::cout << "EKF Intial" <<endl;
    this->robotPosition.robotPositionMean.ptr<double>(0)[0]=0;
    this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
    this->robotPosition.robotPositionMean.ptr<double>(1)[0]=0;
    //set start end points
    this->currentStart.x = this->gridMapper.GetGridMapOriginalPoint().x;
    this->currentStart.y = this->gridMapper.GetGridMapOriginalPoint().y;

    this->FinalEnd.x = this->gridMapper.GetGridMapOriginalPoint().x + 200;
    this->FinalEnd.y = this->gridMapper.GetGridMapOriginalPoint().y;


    this->currentEnd.x = _slam_x;
    this->currentEnd.y = _slam_y;


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

void lab405::MyEFKSLAM::Update()
{

}

void lab405::MyEFKSLAM::Test(int pos)
{
    std::cout << "pos = " << pos << std::endl;
}

void lab405::MyEFKSLAM::EKFStepExamine()
{
//    myrobot->left_dcmotor->Stop();
//    myrobot->right_dcmotor->Stop();
    emit MotorStop();

    int Pos_Stop_error = 5;
    int fir_pos = myrobot->left_dcmotor->GetPose();
    int sec_pos = myrobot->left_dcmotor->GetPose();
    while (abs(sec_pos - fir_pos) > Pos_Stop_error)
    {
//        std::cout << sec_pos << " " << fir_pos << std::endl;
        fir_pos = sec_pos;
        sec_pos = myrobot->left_dcmotor->GetPose();
    }


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

lab405::MyEFKSLAM::~MyEFKSLAM()
{

}
