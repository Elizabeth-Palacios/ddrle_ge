# ddrle_ge
## Table of contents
- [Requirements](#requirements)
- [Project structure](#project-structure)
    - [Launch](#launch)  
    - [Nodes](#nodes)
    - [Worlds](#models)
## Requirements
Basic requirements:
- python2
- keras 2.1.5
- tensorflow 1.8.0
- ros kinetic

## Environment setup
# Git clone ddrl_ge scripts

```
$ cd ~/catkin_ws/src/
$ git@github.com:ELIZABETH1611/ddrle_ge.git
$ cd ~/catkin_ws && catkin build
```

### Start gazebo world 
To launch the corridor world
```
$ roslaunch ddrl_ge turtlebot3_empty_world.launch
```

To launch the turtlebot3_box world
```
$ roslaunch ddrl_ge turtlebot3_box.launch
```

### Start training
Before starting up the corridor world, change the target positions for that environment in ../catkin_ws/src/ddrl_ge/src/nodes/target_v1.py
```
$ roslaunch ddrl_ge turtlebot3_ddrl_ge.launch
