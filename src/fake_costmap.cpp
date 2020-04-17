#include "fake_costmap.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void FakeCostmap::run()
{
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
    // ros::Publisher goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);
    // ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);

    int width;
    n.param("width", width, 20);
    int height;
    n.param("height", height, 20);
    double resolution;
    n.param("resolution", resolution, 0.5);

    ros::Rate loop_rate(1);

    // const geometry_msgs::PoseWithCovarianceStamped testInitialPose = initialPose(0.0, 0.0);
    // const geometry_msgs::PoseStamped testGoalPose = goalPose(9.0, 9.0);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    while (ros::ok())
    {
        nav_msgs::OccupancyGrid costmap = createGrid();
        costmap_pub.publish(costmap);

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/costmap"));

        // goal_pose_pub.publish(testGoalPose);
        // initial_pose_pub.publish(testInitialPose);
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

const geometry_msgs::PoseStamped FakeCostmap::goalPose(const float &x, const float &y)
{
    geometry_msgs::PoseStamped goalPose;
    goalPose.pose.position.x = x;
    goalPose.pose.position.y = y;

    goalPose.header.frame_id = "map";

    return goalPose;
}

const geometry_msgs::PoseWithCovarianceStamped FakeCostmap::initialPose(const float &x, const float &y)
{
    geometry_msgs::PoseWithCovarianceStamped initialPose;
    initialPose.pose.pose.position.x = x;
    initialPose.pose.pose.position.y = y;

    initialPose.header.frame_id = "map";

    return initialPose;
}
