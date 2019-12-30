Instructions:

clone repository to catkin workspace
copy map files to home

In each terminal before running next commands type:

source ./devel/setup.bash

export TURTLEBOT3_MODEL=burger

run:

roslaunch assignment3 maze.launch

in seperate terminal run:

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

and in another seperate terminal run:

rosrun assignment3 mazeSolver.py


