#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

struct Obstacle
{ // dimensions in metres
    double x;
    double y;
    double width;
    double height;
    Obstacle(double x_, double y_, double width_, double height_) : x(x_), y(y_), width(width_), height(height_)
    {}
};

struct TestStruct
{
    int id;
    TestStruct() : id(42)
    {
    }
};

class FakeCostmap
{
public:
    void run();

private:
    nav_msgs::OccupancyGrid createGrid(const int &width, const int &height, const float &resolution, const std::vector<Obstacle> &obstacles);
    const geometry_msgs::PoseStamped goalPose(const float &x, const float &y);
    const geometry_msgs::PoseWithCovarianceStamped initialPose(const float &x, const float &y);
};