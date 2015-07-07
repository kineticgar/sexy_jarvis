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

Generate the user setup script. This will fail due to a missing dependency, but `catkin_make` will generate the setup script needed to install dependencies.

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

Run `catkin_make` in the catkin workspace. This time it should succeed.

Add the following lines to `.bashrc`:

```shell
source /opt/ros/jade/setup.bash
source $HOME/ros_workspace/devel/setup.bash
```

Create a symlink to config folder for bgs configs

```shell
ln -s src/motion_tracker/lib/bgslibrary/vs2010mfc/config config
```

# Running the camera driver

```shell
roslaunch sexy_jarvis camera.launch
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

