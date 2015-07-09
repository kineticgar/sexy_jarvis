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
export ROS_MASTER_URI=http://<machine_name>:11311/
```

The `ROSLAUNCH_SSH_UNKNOWN=1` is recommended for network setups using ssh public-key authentication.

# Configuring sexy jarvis

Copy the file [machines.yaml.sample](https://github.com/garbear/sexy_jarvis/blob/master/config/machines.yaml.sample) to `machines.yaml`. Insert the details of the computers on your network.

# Running sexy jarvis

```shell
roslaunch sexy_jarvis main.launch
```

This launchfile is equivalent to the following commands:

```shell
ROS_NAMESPACE=/sexy_jarvis rosparam load `rospack find sexy_jarvis`/config/machines.yaml machines
ROS_NAMESPACE=/sexy_jarvis rosrun sexy_jarvis power_manager.py
ROS_NAMESPACE=/sexy_jarvis rosrun sexy_jarvis main.py
```

# Waking remote machines

Machines with a mac address specified in `machines.yaml` are sent a wake-on-lan packet when sexy jarvis is run.

To manually send a wake-on-lan packet, add the mac address to `machines.yaml` and, while sexy jarvis is running, execute:

```shell
ROS_NAMESPACE=/sexy_jarvis rosservice call wake_on_lan <machine_name>
```

# Running the camera driver

If ssh public-key authentication is used on your network, the camera drivers will be started on the remote machine for each computer with camera details in `machines.yaml`.

If ssh public-key authentication is not used on your network, open up [power_manager.py](https://github.com/garbear/sexy_jarvis/blob/master/scripts/power_manager.py) and change the global `USE_SSH` to `False`.

The start the camera driver manually, you will need to ssh into each machine and run:

```shell
roslaunch sexy_jarvis camera.launch NAMESPACE:=sexy_jarvis MACHINE:=$HOSTNAME PROCESS:=0
```

You will also need to start the image processing pipeline on the appropriate computer

```shell
roslaunch sexy_jarvis camera.launch NAMESPACE:=sexy_jarvis MACHINE:=<machine_name> DRIVER:=0
```

If you get an error about permissions, try adding the current user to the video group.

```shell
sudo usermod -a -G video `whoami`
```

# Running the motion detector

```shell
NAMESPACE=/sexy_jarvis/<macine_name> rosrun motion_tracker background_subtractor `rospack find motion_tracker`
```

# Viewing the image streams

```shell
NAMESPACE=/sexy_jarvis/<machine_name> rosrun image_view image_view image:=camera/image_raw _image_transport:=compressed
NAMESPACE=/sexy_jarvis/<machine_name> rosrun image_view image_view image:=camera/foreground
NAMESPACE=/sexy_jarvis/<machine_name> rosrun image_view image_view image:=camera/background
```

