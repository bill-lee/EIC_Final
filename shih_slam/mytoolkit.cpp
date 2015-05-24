#include"mytoolkit.h"
using namespace std;

/*
�e���榡
polarPoint(rho,theta)
cartesianPoint(x,y)
*/
void PolarToCartesian(const cv::Point2d &polarPoint,cv::Point2d &cartesianPoint){  // rad

    float rho = polarPoint.x, theta = polarPoint.y;
    double a = cos(theta), b = sin(theta);
    cartesianPoint.x= a*rho, cartesianPoint.y= b*rho;

}
/*----------------------------------------------------------------------------*/
void CartesianToPolar(const cv::Point2d &cartesianPoint,cv::Point2d &polarPoint){



     double x=cartesianPoint.x,y=cartesianPoint.y;


     polarPoint.y=atan2(y,x);
     polarPoint.x=sqrt(pow(x,2)+pow(y,2)); //rho

    //QMessageBox::information(0,0,QString::number(y)+"   "+QString::number(x)+"   "+QString::number(y/x));

}
/*----------------------------------------------------------------------------*/
void ConvertRobotToWorld(const Line& localLine,const RobotState& robotPose, Line& globalLine)  //line:�ϥη��y�Ъ��
{
 // Input[�Ѽ�1: �����H�y�Ъ��u �Ѽ�2: �b�@�ɮy�Ъ������H��m ] output[�Ѽ�3: �@�ɮy�Ъ��u(�a��)]
    Line worldTemp;
    //---------------------------------------------------------------------------------------//
    //Mean�ഫ
    //phi
    worldTemp.lineMean.ptr<double>(1)[0]= localLine.lineMean.ptr<double>(1)[0]+robotPose.robotPositionMean.ptr<double>(2)[0];
    //r
    worldTemp.lineMean.ptr<double>(0)[0]=localLine.lineMean.ptr<double>(0)[0]+(robotPose.robotPositionMean.ptr<double>(0)[0]*cos(worldTemp.lineMean.ptr<double>(1)[0])+robotPose.robotPositionMean.ptr<double>(1)[0]*sin(worldTemp.lineMean.ptr<double>(1)[0]));
    //---------------------------------------------------------------------------------------//
    //Covariance�ഫ
    //1.���G�n��compounding��H�����H�y�Ъ�ܽu���~�t �z�L �@�ɮy�о����H��m���~�t  �i���ഫ�i�H�o��  �H�@�ɮy�Ъ�ܽu���~�t
    //2.��� ���B�ҳ]r_error=0.001  phi_error=0.01 (Map��error)   ��άO�NH �x�}�d�b�~����
    worldTemp.lineCovariance=cv::Mat::zeros(localLine.lineCovariance.size(),localLine.lineCovariance.type());
    worldTemp.lineCovariance.ptr<double>(0)[0]=worldTemp.lineCovariance.ptr<double>(1)[1]=0.01;

    globalLine.lineMean=worldTemp.lineMean.clone();
    globalLine.lineCovariance=worldTemp.lineCovariance.clone();

}
void ConvertRobotToWorld(const Feature& localFeature,const RobotState& robotPose, Feature& globalFeature)  //line:�ϥη��y�Ъ��
{
 // Input[�Ѽ�1: �����H�y�Ъ��u �Ѽ�2: �b�@�ɮy�Ъ������H��m ] output[�Ѽ�3: �@�ɮy�Ъ��u(�a��)]
    Feature worldTemp;
    //---------------------------------------------------------------------------------------//
    //Mean�ഫ
    //phi
    worldTemp.featureMean.ptr<double>(1)[0]= localFeature.featureMean.ptr<double>(1)[0]+robotPose.robotPositionMean.ptr<double>(2)[0];
    //r
    worldTemp.featureMean.ptr<double>(0)[0]=localFeature.featureMean.ptr<double>(0)[0]+(robotPose.robotPositionMean.ptr<double>(0)[0]*cos(worldTemp.featureMean.ptr<double>(1)[0])+robotPose.robotPositionMean.ptr<double>(1)[0]*sin(worldTemp.featureMean.ptr<double>(1)[0]));
    //---------------------------------------------------------------------------------------//
    //Covariance�ഫ
    //1.���G�n��compounding��H�����H�y�Ъ�ܽu���~�t �z�L �@�ɮy�о����H��m���~�t  �i���ഫ�i�H�o��  �H�@�ɮy�Ъ�ܽu���~�t
    //2.��� ���B�ҳ]r_error=0.001  phi_error=0.01 (Map��error)   ��άO�NH �x�}�d�b�~����
    worldTemp.featureCovariance=cv::Mat::zeros(localFeature.featureCovariance.size(),localFeature.featureCovariance.type());
    if(localFeature.GetFeatureType()==Line_Feature)
        worldTemp.featureCovariance.ptr<double>(0)[0]=worldTemp.featureCovariance.ptr<double>(1)[1]=0.01;
    else
        worldTemp.featureCovariance.ptr<double>(0)[0]=worldTemp.featureCovariance.ptr<double>(1)[1]=0.03;

    globalFeature.featureMean=worldTemp.featureMean.clone();
    globalFeature.featureCovariance=worldTemp.featureCovariance.clone();

}
void ConvertWorldToRobot(const Feature& globalFeature,const RobotState& robotPose, Feature& localLine)
{

    Feature localTemp;
    //---------------------------------------------------------------------------------------//
    //Mean�ഫ
    //phi
    localTemp.featureMean.ptr<double>(1)[0]= globalFeature.featureMean.ptr<double>(1)[0]-robotPose.robotPositionMean.ptr<double>(2)[0];
    //r
    localTemp.featureMean.ptr<double>(0)[0]=globalFeature.featureMean.ptr<double>(0)[0]-(robotPose.robotPositionMean.ptr<double>(0)[0]*cos(globalFeature.featureMean.ptr<double>(1)[0])+robotPose.robotPositionMean.ptr<double>(1)[0]*sin(globalFeature.featureMean.ptr<double>(1)[0]));
    //---------------------------------------------------------------------------------------//
    //Covariance�ഫ
    localTemp.featureCovariance=cv::Mat::zeros(globalFeature.featureCovariance.size(),globalFeature.featureCovariance.type());

    if(globalFeature.GetFeatureType()==Line_Feature)
    {
        localTemp.featureCovariance.ptr<double>(0)[0]=1.69;
        localTemp.featureCovariance.ptr<double>(1)[1]=0.1;;  //line
        localTemp.featureCovariance.ptr<double>(1)[0]=localTemp.featureCovariance.ptr<double>(0)[1]=0;

    }
    else
    {
        localTemp.featureCovariance.ptr<double>(0)[0]=1.69;
        localTemp.featureCovariance.ptr<double>(1)[1]=2.9;;  //line
        localTemp.featureCovariance.ptr<double>(1)[0]=localTemp.featureCovariance.ptr<double>(0)[1]=0.9;  //corner
    }


    localLine.featureMean=localTemp.featureMean.clone();
    localLine.featureCovariance=localTemp.featureCovariance.clone();

}
void LoadLaserData(const char *dataName,std::vector<std::vector<double> > &laserData )
{

    ifstream file(dataName, ios_base::in);
    double temp;
    laserData.clear();
    laserData.reserve(1000);

    int laserDeep=540;

    if (!file){                       // �Y�L�k�}���ɮ�
            cerr << "�ɮ׶}�ҥ���-�Ь��]�p�H��" << endl;
            return;
    }

    while(!file.eof()){  //���F���n�g��  ��laserDeep�O���n��
            vector<double> vecLaser;
            vecLaser.reserve(laserDeep);

            for(int j=0;j!=laserDeep;++j){
                    file>>temp;
                    vecLaser.push_back(temp);
                  //  laserangle+=resolution;
            }

            if(file.eof())
                break;

            laserData.push_back(vecLaser);

    }
    file.close();

}
void LoadOdomData(const char *dataName,std::vector<cv::Point2d> &odomData )
{
    ifstream file(dataName, ios_base::in);
    double temp=0;

    odomData.clear();   //
    odomData.reserve(1000);
    if (!file){                      // �Y�L�k�}���ɮ�
            cerr << "�ɮ׶}�ҥ���-�Ь��]�p�H��1" << endl;
            return;
    }

    while(!file.eof()){
            double x ,y;
            file>>x>>y;

            if(file.eof())
                break;

            odomData.push_back(cv::Point2d(x,y));

    }

    file.close();



}

cv::Mat CvMatSearchNode::gridMapN=cv::Mat::zeros(480, 640, CV_8U);  //intial

bool CvMatSearchNode::IsSameState(CvMatSearchNode &rhs)
{
    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
            (y == rhs.y) )
    {
            return true;
    }
    else
    {
            return false;
    }

}

float CvMatSearchNode::GoalDistanceEstimate(CvMatSearchNode &nodeGoal)
{
    float xd = abs( ( (float)x - (float)nodeGoal.x ) );
    float yd = abs( ( (float)y - (float)nodeGoal.y) );

    return xd + yd;

}

bool CvMatSearchNode::IsGoal(CvMatSearchNode &nodeGoal)
{
    if( (x == nodeGoal.x) &&
            (y == nodeGoal.y) )
    {
            return true;
    }

    return false;

}

bool CvMatSearchNode::GetSuccessors(AStarSearch<CvMatSearchNode> *astarsearch, CvMatSearchNode *parent_node)
{

    int parent_x = -1;
    int parent_y = -1;

    if( parent_node )
    {
            parent_x = parent_node->x;
            parent_y = parent_node->y;
    }

    //  up-left      up      up-right
    //  left         center  right
    //  down-left    down    down-right
    CvMatSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetGridMapValue( x-1, y ) == 255) //down
            && !((parent_x == x-1) && (parent_y == y))
      )
    {
            NewNode = CvMatSearchNode( x-1, y );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x, y-1 ) == 255) //left
            && !((parent_x == x) && (parent_y == y-1))
      )
    {
            NewNode = CvMatSearchNode( x, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x+1, y )== 255) //right
            && !((parent_x == x+1) && (parent_y == y))
      )
    {
            NewNode = CvMatSearchNode( x+1, y );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x, y+1 ) == 255)  //up
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
            NewNode = CvMatSearchNode( x, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x-1, y-1 )== 255)  //up-left
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
            NewNode = CvMatSearchNode( x-1, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x-1, y+1 )== 255)  //down-left
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
            NewNode = CvMatSearchNode( x-1, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x+1, y-1 )== 255)  // up-right
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
            NewNode = CvMatSearchNode( x+1, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }
    if( (GetGridMapValue( x+1, y+1 )== 255)  // down-right
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
            NewNode = CvMatSearchNode( x+1, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }


    return true;


}

void CvMatSearchNode::PrintNodeInfo()
{
    char str[100];
    //sprintf( str, "Node position : (%d,%d)\n", x,y );
    sprintf( str, "%d   %d \n", x,y );
    cout << str;

}

float CvMatSearchNode::GetCost(CvMatSearchNode &successor)
{
    return 1;
}


double CvMatSearchNode::GetGridMapValue(int x1, int y1)
{

    //free: 0(0~20)  occupy: 255(235~255)  unkown: 127(115~140)

    double num=gridMapN.ptr<uchar>(y1)[x1];
    if( x1 < 0 ||x1 >gridMapN.cols ||y1 < 0 ||y1 > gridMapN.rows||(num>115 && num<140 ))
    {
            return 127;  //127
    }


    if(num>=0 && num<20)
        return 0;

    if(num>235 && num<=255)
         return 255;

}
void dataConvertRobotToWorld(const std::vector<cv::Point2d>& localPointSets,const RobotState& robotPose, std::vector<cv::Point2d>& globalPointSets)
{

    globalPointSets.clear();
    globalPointSets.reserve(localPointSets.size());


    for(int i=0;i!=localPointSets.size();++i)
    {

        double x=cos(robotPose.robotPositionMean.ptr<double>(2)[0])*localPointSets[i].x - sin(robotPose.robotPositionMean.ptr<double>(2)[0])*localPointSets[i].y + robotPose.robotPositionMean.ptr<double>(0)[0];
        double y=sin(robotPose.robotPositionMean.ptr<double>(2)[0])*localPointSets[i].x + cos(robotPose.robotPositionMean.ptr<double>(2)[0])*localPointSets[i].y + robotPose.robotPositionMean.ptr<double>(1)[0];
        globalPointSets.push_back(cv::Point2d(x,y));
    }



}
