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
    costmap.info.resolution = 0.2;
    costmap.info.width = 10;
    costmap.info.height = 10;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    costmap.header = header;

    for (int i = 0; i < 400; i++)
    {
        if (i > 150 && i < 250) {
            costmap.data.push_back(100);
        } else {
            costmap.data.push_back(0);
        }
    }
    
    return costmap;
}