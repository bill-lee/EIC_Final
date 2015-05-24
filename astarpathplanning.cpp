#include "astarpathplanning.h"

cv::Mat MapSearchNode::PlanningMap = cv::Mat::zeros(480, 640, CV_8U);  //intial

void AStarPathPlanning::SetGridMap(const cv::Mat &map, const int erodeSize)
{
    gridMap=map.clone();
    int dilation_type = cv::MORPH_RECT;
    int dilation_size=erodeSize;

    cv::Mat element = cv::getStructuringElement( dilation_type,
    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
    cv::Point( -1, -1 ) );

    cv::erode( gridMap, pathPlanningMap, element );

    MapSearchNode::PlanningMap = pathPlanningMap.clone();
//    cv::imshow("PlanningMap", MapSearchNode::PlanningMap);
//    cv::waitKey(1);

    drawpathMap=gridMap.clone();
}

AStarPathPlanning::AStarPathPlanning()
{

}

bool AStarPathPlanning::run(const cv::Point2i &StartNode, const cv::Point2i &EndNode, std::vector<cv::Point2d> &PathPointSet)
{
//    std::cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost
    // of travel across the terrain. Zero means the least possible difficulty
    // in travelling (think ice rink if you can skate) whilst 5 represents the
    // most difficult. 9 indicates that we cannot pass.

    // Create an instance of the search class...

//    AStarSearch<MapSearchNode> astarsearch;
    PathPointSet.clear();

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;

    while(SearchCount < NumSearches)
    {

        // Create a start state
        MapSearchNode nodeStart(StartNode);


        // Define the goal state
        MapSearchNode nodeEnd(EndNode);


        // Set Start and goal states

        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do
        {
            SearchState = astarsearch.SearchStep();

            SearchSteps++;

//#if DEBUG_LISTS

//            std::cout << "Steps:" << SearchSteps << "\n";

//            int len = 0;

//            std::cout << "Open:\n";
//            MapSearchNode *p = astarsearch.GetOpenListStart();
//            while( p )
//            {
//                len++;
//#if !DEBUG_LIST_LENGTHS_ONLY
//                ((MapSearchNode *)p)->PrintNodeInfo();
//#endif
//                p = astarsearch.GetOpenListNext();

//            }

//            std::cout << "Open list has " << len << " nodes\n";

//            len = 0;

//            std::cout << "Closed:\n";
//            p = astarsearch.GetClosedListStart();
//            while( p )
//            {
//                len++;
//#if !DEBUG_LIST_LENGTHS_ONLY
//                p->PrintNodeInfo();
//#endif
//                p = astarsearch.GetClosedListNext();
//            }

//            std::cout << "Closed list has " << len << " nodes\n";
//#endif

        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            std::cout << "Search found goal state\n";

            MapSearchNode *node = astarsearch.GetSolutionStart();

//#if DISPLAY_SOLUTION
//            std::cout << "Displaying solution\n";
//#endif
            int steps = 0;

            node->PrintNodeInfo();
            PathPointSet.push_back(cv::Point2d(node->x, node->y));
            for( ;; )
            {
                node = astarsearch.GetSolutionNext();

                if( !node )
                {
                    break;
                }

                node->PrintNodeInfo();
                PathPointSet.push_back(cv::Point2d(node->x, node->y));
                steps ++;

            };

            std::cout << "Solution steps " << steps << endl;

            // Once you're done with the solution you can free the nodes up
            astarsearch.FreeSolutionNodes();


        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
        {
            std::cout << "Search terminated. Did not find goal state\n";

        }

        // Display the number of loops the search went through
        std::cout << "SearchSteps : " << SearchSteps << "\n";

        SearchCount ++;

        astarsearch.EnsureMemoryFreed();
    }

}

AStarPathPlanning::~AStarPathPlanning()
{

}


// Cost Function
float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal)
{
    float xd = abs( ( (float)this->x - (float)nodeGoal.x) );
    float yd = abs( ( (float)this->y - (float)nodeGoal.y) );

    return xd + yd;
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal)
{
    if( (x == nodeGoal.x) &&
            (y == nodeGoal.y) )
    {
            return true;
    }

    return false;
}

bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node)
{
    std::cout << "GetSuccessors IN!" << std::endl;
    std::cout << "x = " << x << ", y = " << y << " GetGridMapValue = " << GetGridMapValue( x, y ) << std::endl;
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
    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetGridMapValue( x-1, y ) == 255) //down
            && !((parent_x == x-1) && (parent_y == y))
      )
    {
        std::cout << "down IN!" << std::endl;
            NewNode = MapSearchNode( x-1, y );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x, y-1 ) == 255) //left
            && !((parent_x == x) && (parent_y == y-1))
      )
    {
        std::cout << "left IN!" << std::endl;
            NewNode = MapSearchNode( x, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x+1, y )== 255) //right
            && !((parent_x == x+1) && (parent_y == y))
      )
    {
        std::cout << "right IN!" << std::endl;
            NewNode = MapSearchNode( x+1, y );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x, y+1 ) == 255)  //up
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        std::cout << "up IN!" << std::endl;
            NewNode = MapSearchNode( x, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }

    if( (GetGridMapValue( x-1, y-1 )== 255)  //up-left
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        std::cout << "up-left IN!" << std::endl;
            NewNode = MapSearchNode( x-1, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x-1, y+1 )== 255)  //down-left
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        std::cout << "down-left IN!" << std::endl;
            NewNode = MapSearchNode( x-1, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }


    if( (GetGridMapValue( x+1, y-1 )== 255)  // up-right
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        std::cout << "up-right IN!" << std::endl;
            NewNode = MapSearchNode( x+1, y-1 );
            astarsearch->AddSuccessor( NewNode );
    }
    if( (GetGridMapValue( x+1, y+1 )== 255)  // down-right
            && !((parent_x == x) && (parent_y == y+1))
            )
    {
        std::cout << "down-right IN!" << std::endl;
            NewNode = MapSearchNode( x+1, y+1 );
            astarsearch->AddSuccessor( NewNode );
    }
    return true;
}

float MapSearchNode::GetCost(MapSearchNode &successor)
{
    if( x < 0 ||
        x >= PlanningMap.cols ||
         y < 0 ||
         y >= PlanningMap.rows
      )
    {
        return 255;
    }

    return GetGridMapValue(x, y);
//    return 1;
}

bool MapSearchNode::IsSameState(MapSearchNode &rhs)
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

double MapSearchNode::GetGridMapValue(int x1, int y1)
{
    //free: 0(0~20)  occupy: 255(235~255)  unkown: 127(115~140)

    double num = MapSearchNode::PlanningMap.ptr<uchar>(y1)[3*x1];


//    cv::Mat show = MapSearchNode::PlanningMap.clone();
//    cv::circle(show, cv::Point2i(x1, y1), 1, cv::Scalar(0, 0, 255));
//    cv::imshow("show", show);
//    cv::waitKey(1);
//    std::cout << "num = " << num << std::endl;

    if( x1 < 0 ||x1 > MapSearchNode::PlanningMap.cols ||y1 < 0 ||y1 > MapSearchNode::PlanningMap.rows||(num>115 && num<140 ))
    {
            return 127;  //127
    }


    if(num >= 0 && num<20)
        return 0;

    if(num > 235 && num <= 255)
         return 255;
}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x,y );

    cout << str;
}
