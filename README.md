# home-service-robot-project

This is a ROS project that makes a Turtlebot autonomously navigate between two points, pickup, and drop off an object.

## Installation

1. Create a folder titled "catkin_ws". Change directory in the command line to that folder.

2. Download this repository or type `$ git clone https://github.com/KDharmarajanDev/home-service-robot-project.git`

3. Ensure ROS is installed on the computer and type `catkin_make`.

4. Type `cd src/scripts`. The scripts present in this directory will allow one to test different parts of the robot.

5. For each script, type `chmod +x (name of script)`. Example: `chmod +x pick_objects.sh`

## Usage

To test navigation, run `./test_navigation`. This script uses the `turtlebot_gazebo` package, RVIZ, and adaptive Monte Carlo localization package to localize the robot within the environment. In order to test the AMCL algorithm, click on the 2D Navigation Goal in RVIZ. Then the robot should localize and move to the target.

To test SLAM, run `./test_slam`. This script uses the `turtlebot_gazebo` package, `gmapping` package, RVIZ, and `turtlebot_teleop` package to test the SLAM functionality. Once the script has been executed, locate the terminal that was run from the `turtlebot_teleop` package. Use the indicated buttons to make the robot move around, and see the result of the SLAM in RVIZ. `gmapping` will provide a 2D occupancy grid that represents the surroundings.

To test the robot navigation between the pickup and drop off points for the objects, type `./pick_objects`. This script uses the `turtlebot_gazebo` package, `AMCL` package, RVIZ, and the `pick_objects` node. This will make the robot localize using AMCL and then navigate between a pickup and a drop off point.

To test the marker system representing the objects, type `./add_marker`. This mainly uses the `add_markers` node to display in RVIZ in the `turtlebot_gazebo` world.

To have a full test of all systems, type `./home_service`. This script uses the `turtlebot_gazebo` package, `AMCL` package, RVIZ, `pick_objects` node, and `add_markers` node. Running this script will make the robot localize and navigate to the pickup spot. It will then pickup an item as indicated by the marker. Then it will localize and navigate to the drop off location and drop the item.

## License
This project uses the MIT License.
