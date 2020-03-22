#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class FakeCostmap
{
public:
    void run();

private:
    nav_msgs::OccupancyGrid createGrid();
    const geometry_msgs::PoseStamped goalPose(const float &x, const float &y);
    const geometry_msgs::PoseWithCovarianceStamped initialPose(const float &x, const float &y);
};