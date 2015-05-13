#include "ekf_slam.h"

using namespace std;

void EKF_SLAM::Initial(const std::vector<std::vector<Line> >& lineFeature)
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
        cout<<"�ĴX��loop:"<<i<<" �o�{�X�ӯS�x:"<<map.size()<<endl;

        if(map.size()>0)
        {

            for(int u=0;u!=map.size();++u)
            {

                Feature localTemp;

                RobotState test;



                ConvertWorldToRobot(map[u],robotState,localTemp);

             //   mapper.DrawLine(localTemp,RobotState(),cv::Scalar(0,255,0),3,cv::Point2d(250,250),1,imgg);



                cout<<"��"<<u<<"�ӯS�x"<<endl;
                cout<<"�[�J�e"<<endl;
                cout<<"�����H��m"<<robotCombinedState<<endl;
                cout<<"�����H�@�ܲ���x�}"<<robotCombinedCovariance<<endl;

                AddNewLandmark(localTemp);

                cout<<"�[�J��"<<endl;
                cout<<"�����H��m"<<robotCombinedState<<endl;
                cout<<"�����H�@�ܲ���x�}"<<robotCombinedCovariance<<endl;

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

    cout<<"��l�Ƨ�쪺�S�x�ƥ�:"<<landmarkNum<<" "<<landmarkSets.size()<<endl;


}

void EKF_SLAM::MotionPrediction(const cv::Point2d& odometryValue) //[x=right y=left]
{


    ////////////////////////////////////////////////////////////////////////////////
    double x=robotState.robotPositionMean.ptr<double>(0)[0];
    double y=robotState.robotPositionMean.ptr<double>(1)[0];
    double phi=robotState.robotPositionMean.ptr<double>(2)[0];  //rad
    double right=odometryValue.x;
    double left=odometryValue.y;
    cv::Mat Fx=cv::Mat::eye(STATE_SIZE,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);  //3*(3+3*N)


    ////////////////////////////////////////////////////////////////////////////////
    //------�a�J���ɪ������Hmotion model��mean��  Ut=f(command,Ut-1)-------------------------------------------------------
    double tempX=(right+left)/2.0*cos(phi+(right-left)*deltaT/(2*robotWidth));
    double tempY=(right+left)/2.0*sin(phi+(right-left)*deltaT/(2*robotWidth));
    double tempThtea=((right-left)/robotWidth)*deltaT;

    //robot state
    robotState.robotPositionMean.ptr<double>(0)[0]+=tempX;
    robotState.robotPositionMean.ptr<double>(1)[0]+=tempY;
    robotState.robotPositionMean.ptr<double>(2)[0]+=tempThtea;
    //y vector
    robotCombinedState.ptr<double>(0)[0]+=tempX;
    robotCombinedState.ptr<double>(1)[0]+=tempY;
    robotCombinedState.ptr<double>(2)[0]+=tempThtea;

    //normalize
    while(robotState.robotPositionMean.ptr<double>(2)[0]> CV_PI)
    {
        robotState.robotPositionMean.ptr<double>(2)[0] -= 2*CV_PI;
        robotCombinedState.ptr<double>(2)[0]-= 2*CV_PI;
    }
    while(robotState.robotPositionMean.ptr<double>(2)[0]< -CV_PI)
    {
        robotState.robotPositionMean.ptr<double>(2)[0] += 2*CV_PI;
        robotCombinedState.ptr<double>(2)[0]+= 2*CV_PI;
    }


    ////////////////////////////////////////////////////////////////////////////////
    //------�a�J���ɪ������Hmotion model��covariance  -------------------------------------------------------
    //(a):��motion model���ܮ� motion model Jacobians�n��o��
    cv::Mat Gt=cv::Mat::eye(STATE_SIZE+OBS_SIZE*landmarkNum,STATE_SIZE+OBS_SIZE*landmarkNum ,CV_64F);
    Gt.ptr<double>(0)[2]=-(right+left)/2.0*sin(phi+(right-left)*deltaT/(2.0*robotWidth));
    Gt.ptr<double>(1)[2]=(right+left)/2.0*cos(phi+(right-left)*deltaT/(2.0*robotWidth));



    //(b):��motion model���ܮ� motion error model Jacobians�n��o��
    cv::Mat motionErrorJacobian=cv::Mat::zeros(STATE_SIZE,MOT_SIZE,CV_64F);
    cv::Mat R=cv::Mat::zeros(STATE_SIZE,STATE_SIZE,CV_64F);
    motionErrorJacobian.ptr<double>(0)[0]=0.5*cos(phi+(right-left)*deltaT/(2*robotWidth))-(right+left)*deltaT/(4*robotWidth)*sin(phi+(right-left)*deltaT/(2*robotWidth));
    motionErrorJacobian.ptr<double>(0)[1]=0.5*cos(phi+(right-left)*deltaT/(2*robotWidth))+(right+left)*deltaT/(4*robotWidth)*sin(phi+(right-left)*deltaT/(2*robotWidth));
    motionErrorJacobian.ptr<double>(1)[0]=0.5*sin(phi+(right-left)*deltaT/(2*robotWidth))+(right+left)*deltaT/(4*robotWidth)*cos(phi+(right-left)*deltaT/(2*robotWidth));
    motionErrorJacobian.ptr<double>(1)[1]=0.5*sin(phi+(right-left)*deltaT/(2*robotWidth))-(right+left)*deltaT/(4*robotWidth)*cos(phi+(right-left)*deltaT/(2*robotWidth));
    motionErrorJacobian.ptr<double>(2)[0]=deltaT/robotWidth;
    motionErrorJacobian.ptr<double>(2)[1]=-deltaT/robotWidth;
    //(c) motion noise
    cv::Mat motionError=cv::Mat::eye(MOT_SIZE,MOT_SIZE,CV_64F);
    motionError.ptr<double>(0)[0]=3.5*odometryValue.x;         //Sr^2     5
    motionError.ptr<double>(0)[1]=0.0;                          //SrSl
    motionError.ptr<double>(1)[0]=0.0;                          //SlSr
    motionError.ptr<double>(1)[1]=3.5*odometryValue.y;         //Sl^2     5

    R=motionErrorJacobian*motionError*motionErrorJacobian.t();

    robotCombinedCovariance=cv::Mat(Gt*robotCombinedCovariance*Gt.t()+Fx.t()*R*Fx).clone();



    for(int i=0;i!=STATE_SIZE;++i)
       for(int j=0;j!=STATE_SIZE;++j)
            robotState.robotPositionCovariance.ptr<double>(i)[j]=robotCombinedCovariance.ptr<double>(i)[j];


   // cv::imshow("23",robotCombinedCovariance*1000);
    //cv::waitKey(1);

    //////////////////////////////////////////////////////
    //cout<<"//////////////////////////////////////////////////////"<<endl;
    //cout<<"�w���������H��m"<<robotCombinedState<<endl;
    //cout<<"�w���������HCovariance"<<robotCombinedCovariance<<endl;
     cout<<"�w���������H"<<robotState.robotPositionMean<<endl;
     //cout<<"�w���������H"<<robotState.robotPositionCovariance<<endl;
   // cout<<"//////////////////////////////////////////////////////"<<endl;

}
void EKF_SLAM::AddNewLandmark(const Feature& singleFeature)  //��J:�����H�y�Ъ��u�S�x
{

        //�X�j�x�}
        cv::Mat extendMap=cv::Mat::zeros(STATE_SIZE+OBS_SIZE*(++landmarkNum),1,CV_64F);
        cv::Mat extendMapC=cv::Mat::zeros(extendMap.rows,extendMap.rows,CV_64F);
        //cv::Mat H = cv::Mat::zeros(OBS_SIZE,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);

        Feature worldFeature;
        ConvertRobotToWorld(singleFeature,robotState,worldFeature);  //compounding ���Y
        //�ݭn�ন�@�ɮy�Ъ�r alpha
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

void EKF_SLAM::FeatureSelection(const std::vector<Feature> &obsFeatureSets, std::vector<Feature> &landmark)
{  //��J���b�����H�y�Ъ��S�x   ��X���b�@�ɮy�ФW��landmark

    landmark.clear();
    landmark.reserve(obsFeatureSets.size());
    //

    vector<bool> matchFlag;  //true: match  �P�_obs�S�x�O�_�Qmatch�� �p�G�S���Qmatch��h�����Ĥ@���o�{
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
                //cout<<"��"<<i<<"�ӯS�x_�P��"<<count<<"  "<<(*j).featureMean<<"�ӭԿ�H_����"<<distance<<endl;

                *k=*k+1;  //�p��+1
                match=true;
               //�n��ҧ��@�ӬO�_�~����for�]��
            }

            j++;
            k++;  //���л��W

        }


        //check landmark set �O�_���@�˪��S�x
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

        //�P�_obs�S�x�O�_�Qmatch�� �p�G�S���Qmatch��h�����Ĥ@���o�{
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
        //�Ĥ@���o�{���S�x
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

double EKF_SLAM::_EuclideanDistance(const Feature& obsFeautureW, const Feature& candFeauturW)
{
    //���y����d���y��
    cv::Point2d obsXY;
    cv::Point2d candXY;
    PolarToCartesian(cv::Point2d(obsFeautureW.featureMean.ptr<double>(0)[0],obsFeautureW.featureMean.ptr<double>(1)[0]),obsXY);
    PolarToCartesian(cv::Point2d(candFeauturW.featureMean.ptr<double>(0)[0],candFeauturW.featureMean.ptr<double>(1)[0]),candXY);

    return sqrt((obsXY.x-candXY.x)*(obsXY.x-candXY.x)+(obsXY.y-candXY.y)*(obsXY.y-candXY.y));
}
double EKF_SLAM::_MahalanobisDistance(const cv::Mat &mean, const cv::Mat &covariance)
{
    cv::Mat mahalanobisDistance=(mean.t()*(covariance.inv())*mean);

    return mahalanobisDistance.ptr<double>(0)[0];
}
void EKF_SLAM::DataAssociation(const std::vector<Line> &obsLineFeature,const std::vector<Corner>& obsCornerFeature )
{
    //obsFeature �Ҧb�����H�y�ФW  ��u�P���S�x�X�_�ӧa!


    LandmarkMapping mapper;
    cv::Mat imgg(500,500,CV_8UC3);
    imgg=cv::Scalar::all(0);
    cv::circle(imgg, cv::Point(0+250,0+250), 3, cv::Scalar(255,0,255), 2 );


    for(int i=0;i!=landmarkSets.size();++i)
    {

        mapper.DrawLine(landmarkSets[i],RobotState(),cv::Scalar(0,255,0),1,cv::Point2d(250,250),1,imgg);

    }

              // mapper.DrawLine(temp11,RobotState(),cv::Scalar(255,0,0),10,cv::Point2d(250,250),1,imgg);


    cout<<"=================DataAssociation================================"<<endl;
    vector<bool> obsMatchFlag(obsLineFeature.size()+obsCornerFeature.size(),false);  //match�쪺��true
    vector<int> landmarkMatchingNum(obsLineFeature.size()+obsCornerFeature.size(),-1);  //�Smatch�쪺�s����-1
    vector<Feature> obsFeature;
    obsFeature.reserve(obsLineFeature.size()+obsCornerFeature.size());

    for(int i=0;i!=obsLineFeature.size();++i)
    {
        Feature temp;
        temp.featureMean=obsLineFeature[i].lineMean.clone();
        temp.featureCovariance=obsLineFeature[i].lineCovariance.clone();
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

    //obsFeature �Ҧb�����H�y�ФW  ��u�P���S�x�X�_�ӧa!
    vector<Feature> newFeature;
    newFeature.reserve(obsLineFeature.size()+obsCornerFeature.size());

   // cout<<"�[��S�x�ƶq:"<<obsFeature.size()<<endl;

    for(int i=0;i!=obsFeature.size();++i)
    {
        //obsFeature �o�@�B���^���쪺�S�x

        /*
        Feature transferTemp;
      //  Feature gobalLineobs;
        //�N�u�S�x�୼�S�x���A
        transferTemp.featureMean=obsLineFeature[i].lineMean;
        transferTemp.featureCovariance=obsLineFeature[i].lineCovariance;
        transferTemp.SetFeatureType(Line_Feature);

        //ConvertRobotToWorld(transferTemp,robotState,gobalLineobs);  //��u�S�x�Ѿ����H�y�����@�ɮy�ФW
        */
        cv::Mat H_min,innovation_min,innovationCovariance_min;

        double GatingFeature=0;

        if(obsFeature[i].GetFeatureType()==Line_Feature)
            GatingFeature =gatingLine;
        else
            GatingFeature =gatingCorner;


        //�N�Ҧ��^����S�x�P�a�϶i��ǰt
        for(int j=0;j!=landmarkSets.size();++j)
        {
           if(obsFeature[i].GetFeatureType()!=landmarkSets[i].GetFeatureType())
               continue;

           Feature localLandmark;
           ConvertWorldToRobot(landmarkSets[j],robotState,localLandmark); //�Nlandmark�ѥ@�ɮy���������H�y�ФW
          //measurement estimation
           cv::Mat H = cv::Mat::zeros(OBS_SIZE,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);
           double x=robotState.robotPositionMean.ptr<double>(0)[0];
           double y=robotState.robotPositionMean.ptr<double>(1)[0];
           //landmark�@�ɮy�Ъ�r alpha
           double alpha=landmarkSets[j].featureMean.ptr<double>(1)[0];

           //covariance
           H.ptr<double>(0)[0]=-cos(alpha);
           H.ptr<double>(0)[1]=-sin(alpha);
           H.ptr<double>(1)[2]=-1;
           H.ptr<double>(0)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)]=1;
           H.ptr<double>(0)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)+1]=x*sin(alpha)-y*cos(alpha);
           H.ptr<double>(1)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)+1]=1;


            //innovation: r alpha

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

           cv::Mat innovationCovariance=H*robotCombinedCovariance*H.t()+obsFeature[i].featureCovariance;

           double gating=_MahalanobisDistance(innovation,innovationCovariance);


           if(gating<=GatingFeature)
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


            ConvertRobotToWorld(obsFeature[i],robotState,temp11);

            mapper.DrawLine(landmarkSets[landmarkMatchingNum[i]],RobotState(),cv::Scalar(255,0,0),2,cv::Point2d(250,250),1,imgg);
            mapper.DrawLine(temp11,RobotState(),cv::Scalar(0,0,255),2,cv::Point2d(250,250),1,imgg);


            cout<<"match��F  ��"<<i<<"�[��P ��"<<landmarkMatchingNum[i]<<"�a��  ����:"<<endl;
            cout<<"��s�e"<<robotState.robotPositionMean<<endl;
            Update(H_min,innovation_min,innovationCovariance_min);  //kalman filter framework
            cout<<"��s��"<<robotState.robotPositionMean<<endl;



            //////////////////////////////////////
            count++;

        }
        else
        {
            newFeature.push_back(obsFeature[i]);  //�S��match�쪺�i�ର�s���S�x
           // cout<<"�[�J�Կ�H�W��"<<endl;
           // AddNewLandmark(obsFeature[i]);
        }

    }



    //feature selection �ϥίS�x�I�z��
    vector<Feature> map;
    FeatureSelection(newFeature,map);
     //cout<<"�s�W�X��landmark�ƶq"<<map.size()<<endl;
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
    cout<<"�W�[���ƥ�:"<<map.size()<<" �a�ϼƶq:"<<landmarkSets.size()<<endl;
    cout<<"�a�ϯS�x(�����H�y��)==========="<<endl;
    for(int i=0;i!=landmarkSets.size();++i)
    {
        Feature localLandmark;
        ConvertWorldToRobot(landmarkSets[i],robotState,localLandmark);
        cout<<"��"<<i<<"��:"<<localLandmark.featureMean<<endl;
        //cout<<"��"<<i<<"�� "<<landmarkSets[i].featureMean<<endl;
    }
    */

   // cout<<"=========================================================="<<endl;
   // cout<<"�����H��s��m"<<endl;
    //cout<<robotCombinedState<<endl;
    ///cout<<"�����H��s��mCovariance"<<endl;
    //cout<<robotCombinedCovariance<<endl;
}

void EKF_SLAM::Update(const cv::Mat &H, const cv::Mat &innovation, const cv::Mat &innovationCovariance)
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

void EKF_SLAM::GetLandmarkSets(std::vector<Feature> &map)
{

    map.clear();
    map.reserve(landmarkSets.size());
    for(int i=0;i!=landmarkSets.size();i++)
        map.push_back(landmarkSets[i]);


}

void EKF_SLAM::SetRobotPose(const RobotState& robot)
{

    robotState.robotPositionMean=robot.robotPositionMean.clone();
    robotCombinedState.ptr<double>(0)[0]=robot.robotPositionMean.ptr<double>(0)[0];
    robotCombinedState.ptr<double>(1)[0]=robot.robotPositionMean.ptr<double>(1)[0];
    robotCombinedState.ptr<double>(2)[0]=robot.robotPositionMean.ptr<double>(2)[0];

}

/*
void EKF_SLAM::AddNewLandmark(const Feature& singleFeature)  //��J:�����H�y�Ъ��u�S�x
{

        //�X�j�x�}
        cv::Mat extendMap=cv::Mat::zeros(STATE_SIZE+OBS_SIZE*(++landmarkNum),1,CV_64F);
        cv::Mat extendMapC=cv::Mat::zeros(extendMap.rows,extendMap.rows,CV_64F);
        cv::Mat H = cv::Mat::zeros(OBS_SIZE,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);

        Feature worldFeature;
        ConvertRobotToWorld(singleFeature,robotState,worldFeature);  //compounding ���Y
        //�ݭn�ন�@�ɮy�Ъ�r alpha
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

        //covariance
        H.ptr<double>(0)[0]=-cos(alpha);
        H.ptr<double>(0)[1]=-sin(alpha);
        H.ptr<double>(1)[2]=-1;
        H.ptr<double>(0)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)]=1;
        H.ptr<double>(0)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)+1]=x*sin(alpha)-y*cos(alpha);
        H.ptr<double>(1)[STATE_SIZE+OBS_SIZE*(landmarkNum-1)+1]=1;


        cout<<"H�x�}"<<H<<endl;
        cout<<"Q�x�}"<<singleFeature.featureCovariance<<endl;




        //insert  matrix
        cv::Mat temp=extendMapC(cv::Rect(0,0,robotCombinedCovariance.cols,robotCombinedCovariance.rows));
        robotCombinedCovariance.copyTo(temp);
        temp=extendMapC(cv::Rect(robotCombinedCovariance.cols,robotCombinedCovariance.rows,worldFeature.featureCovariance.cols,worldFeature.featureCovariance.rows));
        worldFeature.featureCovariance.copyTo(temp);
        cout<<"addmatrix"<<extendMapC<<endl;
        //robotCombinedCovariance=extendMapC.clone();

        cv::Mat KalmanGain=extendMapC*H.t()*((H*extendMapC*H.t()+singleFeature.featureCovariance).inv());

        cv::Mat I=cv::Mat::eye(STATE_SIZE+OBS_SIZE*landmarkNum,STATE_SIZE+OBS_SIZE*landmarkNum,CV_64F);
        cv::Mat tempR=((I-KalmanGain*H)*extendMapC);

        robotCombinedState=extendMap.clone(); //�����D�O�_�n��s?


        robotState.robotPositionMean.ptr<double>(0)[0]=robotCombinedState.ptr<double>(0)[0];
        robotState.robotPositionMean.ptr<double>(1)[0]=robotCombinedState.ptr<double>(1)[0];
        robotState.robotPositionMean.ptr<double>(2)[0]=robotCombinedState.ptr<double>(2)[0];

        robotCombinedCovariance=tempR.clone();

        for(int i=0;i!=STATE_SIZE;++i)
           for(int j=0;j!=STATE_SIZE;++j)
                robotState.robotPositionCovariance.ptr<double>(i)[j]=robotCombinedCovariance.ptr<double>(i)[j];


        landmarkSets.push_back(worldFeature);


        //cout<<robotCombinedCovariance<<endl;

        cout<<"���A�x�}"<<robotCombinedState<<endl;
        cout<<"���A�x�}�ܲ���"<<robotCombinedCovariance<<endl;
       cv::imshow("23",robotCombinedCovariance*1000);
       cv::waitKey(1);
       //cout<<"======================="<<endl;

}
*/
