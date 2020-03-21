#include "fake_perception.h"
#include "fake_costmap.h"

int main(int argc, char **argv)
{
    std::cout << "Running fake_perception" << std::endl;
    ros::init(argc, argv, "fake_perception");
    FakeCostmap node;
    node.run();
    return 0;
}
