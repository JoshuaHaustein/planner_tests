# planner_tests
This repostitory contains tests for the manipulation_planning_suite.

# Installation

## Dependencies
- yaml-cpp
- boost >= 1.58: system, filesystem
- Eigen3
- Qt4
- protobuf
- ompl (1.3.2)
- ROS Indigo Desktop, ROS Lunar Desktop, (maybe others work as well)

Apart from ompl you should be able to install all dependencies from the standard Ubuntu repositories. 
For the installation of ompl, go to http://ompl.kavrakilab.org/download.html and then follow the instructions on 
http://ompl.kavrakilab.org/installation.html to build and install it from source.

## Manipulation Planning Suite using Box2D
First, make sure your terminal session is initialized to work with ROS, i.e. run 
```shell
  source /opt/ros/lunar/setup.bash
```
Adapt to zsh or other ROS version, if you use those.

Next, you need to create a catkin workspace:

```shell
  cd ~/YOUR_PROJECTS
  mkdir planning_catkin
  cd planning_catkin
  mkdir src
  cd src 
  catkin_init_workspace
```

Now you need to download our repos:
```shell
  git clone git@gits-15.sys.kth.se:haustein/manipulation_planning_suite.git
  git clone git@gits-15.sys.kth.se:haustein/planner_tests.git
  git clone git@gits-15.sys.kth.se:haustein/sim_env.git
  git clone git@gits-15.sys.kth.se:haustein/box2d_sim_env.git
  git clone https://github.com/JoshuaHaustein/box2d_catkin.git
```

If all dependencies are installed correctly, you should be able to build everything using: 
```shell
  cd ~/YOUR_PROJECTS/planning_catkin
  catkin_make
```

## Running the planner
If everything built successfully, you can run the planner using rosrun:
```shell
  source devel/setup.bash
  rosrun planner_tests box2d_push_planner --planning_problem src/planner_tests/data/box2d/planning_problems/simple_test_problem.yaml
```
The argument --planning_problem specifies which planning problem to use. There are a couple problems defined in 'src/planner_tests/data/box2d/planning_problems'.
A planning problem specifies not only which world to plan in, but also some parameters such as which algorithm to use and parameters for it.

Alternatively, you can also only load an environment using 
```shell
  rosrun planner_tests box2d_push_planner --environment src/planner_tests/data/box2d/worlds/free_world_1m_0s.yaml
```
