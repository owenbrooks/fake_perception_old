#include "fake_costmap.h"
#include <vector>
#include <algorithm>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void FakeCostmap::run()
{
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
    // ros::Publisher goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);
    // ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);

    int width;
    n.param("costmap_width", width, 20);
    int height;
    n.param("costmap_height", height, 20);
    double resolution;
    n.param("costmap_resolution", resolution, 0.5);

    std::vector<Obstacle> obstacles;
    std::vector<double> obs_params;
    n.getParam("obstacles", obs_params);
    for(int i = 0; i < obs_params.size() / 4; i++)
    {
        Obstacle obs = Obstacle(obs_params[i*4], obs_params[i*4+1], obs_params[i*4+2], obs_params[i*4+3]);
        obstacles.push_back(obs);
    }    

    ros::Rate loop_rate(1);

    // const geometry_msgs::PoseWithCovarianceStamped testInitialPose = initialPose(0.0, 0.0);
    // const geometry_msgs::PoseStamped testGoalPose = goalPose(9.0, 9.0);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    while (ros::ok())
    {
        nav_msgs::OccupancyGrid costmap = createGrid(width, height, resolution, obstacles);
        costmap_pub.publish(costmap);

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/costmap"));

        // goal_pose_pub.publish(testGoalPose);
        // initial_pose_pub.publish(testInitialPose);
    }
}

nav_msgs::OccupancyGrid FakeCostmap::createGrid(const int &width, const int &height, const float &resolution, const std::vector<Obstacle> &obstacles)
{
    nav_msgs::OccupancyGrid costmap;
    costmap.info.resolution = (float)resolution;
    costmap.info.width = width;
    costmap.info.height = height;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "costmap";
    costmap.header = header;

    // initialise costmap to all zeros
    std::vector<int8_t> cm_data(width * height, 0);
    costmap.data = cm_data;

    // loop through obstacles, setting relevant indicies to 1
    for (const Obstacle &obs : obstacles)
    {
        // loop through each horizontal row of obstacle (divide that row according to the costmap resolution.)
        const int cm_origin_index = (obs.y / resolution) * costmap.info.width + obs.x / resolution; // 2/0.5 * 100 + 2/0.5

        for (int i = 0; i < obs.height / resolution; i++)
        {
            const int columnCount = obs.width / resolution; // 
            const auto start_iterator = costmap.data.begin() + cm_origin_index + i * costmap.info.width;
            const auto end_iterator = start_iterator + columnCount;
            std::fill(start_iterator, end_iterator, 100);
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
