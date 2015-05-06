#include "icp.h"

ICP::ICP(int num,double dist,double angle,double sDist,bool ransac):maxIteration(num),thresholdDist(dist),thresholdAng(angle),smallestThresholdDist(sDist),doRANSAC(ransac)
{
}

void ICP::Intial()
{

    icp.options.ICP_algorithm = mrpt::slam::icpLevenbergMarquardt;
    icp.options.maxIterations			= maxIteration;   //迭代次數
    icp.options.thresholdAng			= thresholdAng;
    icp.options.thresholdDist			= thresholdDist;  //10
    icp.options.ALFA			        = 0.5f;  //0.5f;
    icp.options.smallestThresholdDist	= smallestThresholdDist;
    icp.options.doRANSAC = doRANSAC; //是否使用ransac
    icp.options.dumpToConsole();

}

cv::Point3d ICP::Align(const std::vector<cv::Point2d> &referenceMap, const std::vector<cv::Point2d> &targetMap,double referenceNumPercent )
{  //referenceMap 與 targetMap 皆存在於世界座標上
      mrpt::slam::CSimplePointsMap m1,m2;
      mrpt::poses::CPose2D initialPose(0,0,0);

      float runningTime;

      for(int i=0;i!=targetMap.size();++i)
            m2.insertPoint(targetMap[i].x,targetMap[i].y);

      for(int i=referenceMap.size()*referenceNumPercent;i!=referenceMap.size();++i)
           m1.insertPoint(referenceMap[i].x,referenceMap[i].y);


      mrpt::poses::CPosePDFPtr pdf = icp.Align(
                  &m1,  // Reference map
                  &m2,// Map to be aligned
                  initialPose,
                  &runningTime,
                  (void*)&info);



      return cv::Point3d(pdf->getMeanVal().x(),pdf->getMeanVal().y(),pdf->getMeanVal().phi());
}
