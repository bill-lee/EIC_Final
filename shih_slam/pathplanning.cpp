#include "pathplanning.h"

PathPlanning::PathPlanning(int size):erodeSize(size)
{
}

void PathPlanning::SetStartNode(const cv::Point2d &node)
{
    nodeStart.x=node.x;
    nodeStart.y=node.y;


}

void PathPlanning::SetEndNode(const cv::Point2d &node)
{

    nodeEnd.x=node.x;
    nodeEnd.y=node.y;
}

void PathPlanning::AStartPlanning(std::vector<cv::Point2d> &pathSets)
{
    pathSets.clear();
    pathSets.reserve(1000);
/*
    int dilation_type = cv::MORPH_RECT;
    int dilation_size=erodeSize;

    cv::Mat element = cv::getStructuringElement( dilation_type,
    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    cv::Point( -1, -1 ) );



    cv::erode( gridMap, pathPlanningMap, element );
    drawpathMap=gridMap.clone();
*/
    //cv::imshow("dilation_dst",dilation_dst);

  ///////////////
    CvMatSearchNode::gridMapN = pathPlanningMap.clone();

    std::cout << "nodeStart.x = " << nodeStart.x << ", " << "nodeStart.y = " << nodeStart.y << std::endl;
    std::cout << "nodeEnd.x = " << nodeEnd.x << ", " << "nodeEnd.y = " << nodeEnd.y << std::endl;
    astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do
    {
        SearchState = astarsearch.SearchStep();

        SearchSteps++;

    }
    while( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_SEARCHING ); //判斷是否做完路徑規畫

    std::cout << "SearchSteps = " << SearchSteps << std::endl;
    std::cout << "SearchState = " << SearchState << std::endl;

    if( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_SUCCEEDED )
    {
        std::cout << "Search found goal state" << std::endl;
        // cout << "Search found goal state\n";
        CvMatSearchNode *node = astarsearch.GetSolutionStart();

        int steps = 0;

       // node->PrintNodeInfo();
        //CvMatSearchNode::gridMap.ptr<uchar>(node->y)[node->x]=128;
        cv::circle(drawpathMap, cv::Point(node->x,node->y), 1, cv::Scalar(128,128,128), -1  );

        pathSets.push_back(cv::Point2d(node->x,node->y));

        for( ;; )
        {
                node = astarsearch.GetSolutionNext();

                if( !node )
                {
                        break;
                }

                //CvMatSearchNode::gridMap.ptr<uchar>(node->y)[node->x]=128;
                cv::circle(drawpathMap, cv::Point(node->x,node->y), 1, cv::Scalar(128,128,128), -1  );
                //node->PrintNodeInfo();
                pathSets.push_back(cv::Point2d(node->x,node->y));

                steps ++;

        };
        cout << "Solution steps " << steps << endl;
        // Once you're done with the solution you can free the nodes up
        astarsearch.FreeSolutionNodes();
    }
    else if( SearchState == AStarSearch<CvMatSearchNode>::SEARCH_STATE_FAILED )
    {
        std::cout << "Search terminated. Did not find goal state" << std::endl;
    }

    astarsearch.EnsureMemoryFreed();
    cv::imshow("dfdf",drawpathMap);
    cv::waitKey(1);

}

void PathPlanning::SetGridMap(const cv::Mat &map)
{

       gridMap=map.clone();
       int dilation_type = cv::MORPH_RECT;
       int dilation_size=erodeSize;

       cv::Mat element = cv::getStructuringElement( dilation_type,
       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
       cv::Point( -1, -1 ) );

       cv::erode( gridMap, pathPlanningMap, element );
       drawpathMap=gridMap.clone();

}
