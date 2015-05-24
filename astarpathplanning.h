#ifndef ASTARPATHPLANNING_H
#define ASTARPATHPLANNING_H
// Implementation of path planning using A* Algorithm by Justin Heyes-Jones
// Modify from example: findpath.cpp
#include "3rdParty/stlastar.h" // See header for copyright and usage information
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>

//// The world map
//const int MAP_WIDTH = 20;
//const int MAP_HEIGHT = 20;

//// map helper functions
//int GetMap( int x, int y )
//{
//    if( x < 0 ||
//        x >= MAP_WIDTH ||
//         y < 0 ||
//         y >= MAP_HEIGHT
//      )
//    {
//        return 9;
//    }

//    return world_map[(y*MAP_WIDTH)+x];
//}

class MapSearchNode
{
public:
    int x;	 // the (x,y) positions of the node
    int y;

    // using static data allowing external access from class AStarPathPlanning
    static cv::Mat PlanningMap;

    MapSearchNode() { x = y = 0; }
    MapSearchNode( int px, int py ) { x=px; y=py; }
    MapSearchNode(const cv::Point2i& node) { x = node.x; y = node.y;}

    // cost function
    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    // Using 255 as free space
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );
    double GetGridMapValue(int x1, int y1);

    void PrintNodeInfo();


};

class AStarPathPlanning
{
public:
    AStarPathPlanning();
    ~AStarPathPlanning();
    void SetGridMap(const cv::Mat& map, const int erodeSize);
    bool run(const cv::Point2i &StartNode, const cv::Point2i &EndNode, std::vector<cv::Point2d> &PathPointSet);
    void GetPathPlanningMap(cv::Mat& img) {img = pathPlanningMap.clone();}

private:
    cv::Mat pathPlanningMap;
    cv::Mat gridMap;
    cv::Mat drawpathMap;
    AStarSearch<MapSearchNode> astarsearch;
};


#endif // ASTARPATHPLANNING_H
