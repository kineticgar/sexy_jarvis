# Getting the package sources

Sexy jarvis depends on the following packages:

```shell
git clone https://github.com/garbear/sexy_jarvis.git
git clone --recursive https://github.com/garbear/motion_tracker.git
git clone https://github.com/ros-drivers/gscam.git
```

# Building sexy jarvis

First, [create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Clone the packages listed above into the `src` directory.

Run `catkin_make` in the catkin workspace.

