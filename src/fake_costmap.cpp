#include "fake_costmap.h"

void FakeCostmap::run()
{
    ros::NodeHandle n;

    ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);

    ros::Rate loop_rate(1);

    while (ros::ok()) 
    {
        nav_msgs::OccupancyGrid costmap = createGrid();
        costmap_pub.publish(costmap);
    }
}

nav_msgs::OccupancyGrid FakeCostmap::createGrid() 
{
    nav_msgs::OccupancyGrid costmap;
    return costmap;
}