# Fake Perception
ROS Node to publish fake perception output for use in development/testing e.g. a costmap with obstacles

![Fake Costmap displayed in rviz](docs/fake_perception.png?raw=true "Fake Costmap")

## Output
A `nav_msgs/OccupancyGrid` message under the topic `/costmap`
This can be visualised in rviz 

## Steps to Build
1. Clone the repo to src directory of a catkin workspace
2. Add `autoware_msgs` to the src directory (found in the Path Planning folder on the MCAV Google Drive or at https://gitlab.com/autowarefoundation/autoware.ai/messages/-/tree/master/autoware_msgs)
3. Make sure the workspace has been sourced. Can run `source devel/setup.bash` to do so
4. Run `catkin_make` in the catkin workspace
5. Node can now be run via `rosrun fake_perception fake_perception` (ensure `roscore` is running in another terminal) and output visualised under the `/costmap` topic in rviz

## Configuration
* Size and resolution of costmap will soon be configurable
* As will size, location and number of objects

## Future State
* Simulate movement of obstacles to test dynamic obstacle avoidance in planning
