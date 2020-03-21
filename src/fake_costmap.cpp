#include "fake_costmap.h"

void FakeCostmap::run()
{
    ros::NodeHandle n;

    ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);

    ros::Rate loop_rate(1);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    while (ros::ok())
    {
        nav_msgs::OccupancyGrid costmap = createGrid();
        costmap_pub.publish(costmap);

        transform.setOrigin(tf::Vector3(-5.0, -5.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/costmap"));
    }
}

nav_msgs::OccupancyGrid FakeCostmap::createGrid()
{
    nav_msgs::OccupancyGrid costmap;
    const float resolution = 0.5;
    const int width = 20;
    const int height = 20;
    costmap.info.resolution = resolution;
    costmap.info.width = width;
    costmap.info.height = height;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "costmap";
    costmap.header = header;

    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (5 < j && j < 10 && 4 < i && i < 12)
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