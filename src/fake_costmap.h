#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/OccupancyGrid.h"

class FakeCostmap
{
public:
    void run();
private:
    nav_msgs::OccupancyGrid createGrid();
};