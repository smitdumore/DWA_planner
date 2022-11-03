#include "dwa_planner/planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    Planner planner(nh);
    planner.run();
    std::cout << "works" << "\n";   
    return 0;
}