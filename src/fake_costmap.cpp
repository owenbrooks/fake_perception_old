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
    const float resolution = 0.5;
    const int width = 15;
    const int height = 15;
    costmap.info.resolution = resolution;
    costmap.info.width = width;
    costmap.info.height = height;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    costmap.header = header;

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (8 < j && j < 12 && 3 < i && i < 12)
            {
                costmap.data.push_back(100);
            }
            else
            {
                costmap.data.push_back(0);
            }
        }
    }

    return costmap;
}