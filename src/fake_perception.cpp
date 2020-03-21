#include "fake_perception.h"
#include "fake_costmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_perception");
    FakeCostmap node;
    node.run();
    return 0;
}
