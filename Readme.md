# Preparing the workspace

First, [create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```shell
mkdir $HOME/ros_workspace
cd $HOME/ros_workspace
mkdir src
cd src
source /opt/ros/jade/setup.bash
catkin_init_workspace
```

# Getting the package sources

Sexy jarvis depends on the following packages:

```shell
git clone --recursive https://github.com/garbear/sexy_jarvis.git
git clone --recursive https://github.com/garbear/motion_tracker.git
git clone https://github.com/ros-drivers/gscam.git
```

Clone these packages into the `src` directory.

# Building sexy jarvis

Generate the user setup script. This will fail due to a missing dependency, but must be run first to generate the setup script needed to install dependencies.

```shell
cd $HOME/ros_workspace
catkin_make
```

Install dependencies for gscam using [rosdep](http://wiki.ros.org/jade/Installation/Ubuntu#Initialize_rosdep):

```shell
source $HOME/ros_workspace/devel/setup.bash
sudo rosdep init
rosdep update
rosdep install -y gscam
```

Run `catkin_make` in the catkin workspace again. This time it should succeed.

Make sure the following lines are included in `.bashrc`:

```shell
source /opt/ros/jade/setup.bash
source $HOME/ros_workspace/devel/setup.bash
export ROSLAUNCH_SSH_UNKNOWN=1
```

The `ROSLAUNCH_SSH_UNKNOWN=1` is recommended for network setups.

# Running sexy jarvis

```shell
roslaunch sexy_jarvis main.launch
```

# Running the camera driver

The camera driver is meant to be launched from the main node instead of started manually. To launch the camaera driver manually, ssh into the computer and enter:

```shell
roslaunch sexy_jarvis camera.launch NAMESPACE:=sexy_jarvis MACHINE:=$HOSTNAME PROCESS:=0
```

If you get an error about permissions, try adding the current user to the video group.

```shell
sudo usermod -a -G video `whoami`
```

# Running the motion detector

```shell
cd $HOME/ros_workspace
rosrun motion_tracker motion_tracker
```

# Viewing the camera stream

```shell
rosrun image_view image_view image:=/v4l/camera/image_raw _image_transport:=compressed
```

# Viewing the foreground and background

```shell
rosrun image_view image_view image:=/bgs/foreground _image_transport:=compressed
rosrun image_view image_view image:=/bgs/background _image_transport:=compressed
```

