# gazebo_walker

## Dependencies
- Ros 2 Humble
- turtlebot3
- Gazebo
- Basic understanding of ROS 2


## Instructions to build and run
```bash
# clone the package inside source
  cd ros2_ws/src
  git clone https://github.com/Surya-Sriramoju/gazebo_walker.git
# fix dependency issues
  rosdep install -i --from-path src --rosdistro humble -y
# build the code
  cd ~/ros2_ws
  colcon build --packages-select gazebo_walker
```

## Setting up the turtlebot
- Once the turtlebot package is installed, choose the turtlebot that you want to use:
```bash
export TURTLEBOT3_MODEL=burger
```
## Launching the turtlebot
- To launch the turtlebot in a pre-built world use:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Running the node
- After launching the turtlebot, in a new terminal run the `walker` node to control the turtlebot after sourcing ros and this package.
```bash
ros2 run gazebo_walker walker
```
## Launching the launch file
```bash
# with rosbag recording
  ros2 launch gazebo_walker walker_launch.py record_bad:=True
# without rosbag recording
  ros2 launch gazebo_walker walker_launch.py
```
## cppling and cppcheck test
```bash
# run cppcheck
   mkdir results -p && cppcheck --enable=all --std=c++17 src/ include/gazebo_walker/ --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList &> results/cppcheck.txt
# run cpplint
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name '*.cpp' -not -path './build/*') &> results/cpplint.txt
# google test
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")

```