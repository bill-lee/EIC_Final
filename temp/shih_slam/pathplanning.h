#ifndef PATHPLANNING_H
#define PATHPLANNING_H
#include "stlastar.h"
#include "mytoolkit.h"
#include "opencv2/opencv.hpp"
#include <vector>

class PathPlanning
{
public:
    PathPlanning(int size=1);
    inline void SetErodeSize(int s){    erodeSize=s;    }
    void SetGridMap(const cv::Mat& map);
    void SetStartNode(const cv::Point2d& node);
    void SetEndNode(const cv::Point2d& node);
    void AStartPlanning(std::vector<cv::Point2d>& pathSets);
    void GetPathPlanningMap(cv::Mat& img){img=pathPlanningMap.clone();}
private:
    int erodeSize;
    cv::Mat pathPlanningMap;
    cv::Mat gridMap;
    cv::Mat drawpathMap;
    CvMatSearchNode nodeStart;
    CvMatSearchNode nodeEnd;
    AStarSearch<CvMatSearchNode> astarsearch;
};

#endif // PATHPLANNING_H
