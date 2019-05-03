# CarND Capstone Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The capstone (system integration) project in the Udacity's Self-Driving Car NanoDegree. The original project template can be found [here](https://github.com/udacity/CarND-Capstone). For more information about the project, see [the project introduction](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Implementation notes 

### ROS versions

The project was implemented based on ROS Melodic (Ubuntu 18.04). To launch the nodes on ROS Kinetic, reinitialize the catkin workspace:

```bash
# in ros/src
rm CMakeLists.txt
catkin_init_workspace
```

### Update rate

One of the major difficulties has been the inability of the simulator to handle incoming messages at the rate of **50 Hz** (the simulator kept freezing when camera streaming was enabled). It was observed, however, that for the track 1 in the simulator **15 Hz** was generally sufficient, and the program didn't freeze after enabling camera streaming. 

The `waypoint_updater` and `twist_controller` nodes were programmed to read the update rate as a ROS parameter if it is provided by the launch file, or otherwise fall back to 50 Hz. This means that by default, when launching the project from `launch/{styx, site}.launch`, the update rate is 50 Hz. To launch the project in the development mode with 15 Hz enabled, use two terminals: 

```bash
# terminal 1
roslaunch styx server.launch
```

```bash
# terminal 2
roslaunch launcher tl.launch
```

### Simulator

[The capstone project simular](https://github.com/udacity/CarND-Capstone/releases) was of version **1.3** when this project was implemented. 

For performance reasons, it can be beneficial to run the simulator and the ROS nodes on two physical machines. Since the simulator listens at port **4567** on its localhost, port forwarding can be realized using `ncat`:

```bash
# $ROS_HOST is the IP address of the Linux machine running ROS nodes
ncat --sh-exec "ncat $ROS_HOST 4567" -l 4567 --keep-open
```

### Dependencies

Part of the Python dependencies are installed from the Ubuntu repository:

```
sudo apt install python-eventlet python-werkzeug python-jinja2 python-itsdangerous
```

The rest of the dependencies (from `requirements.txt`) can be installed in an isolated virtual environment:

```bash
mkvirtualenv -p python --system-site-packages carndros

# In the `carndros` virtual environment:

pip install -r requirements.txt
```

Note: `eventlet` version 0.19.0 from PyPI doesn't have the `monkey_patch` function, while the one from the Ubuntu repo does. 

### Udacity workspace

A solution to the issue with `catkin_pkg` in Udacity workspace (ROS Kinetic), suggested by Hareendra Manuru on CarND Slack:

```bash
# changes version: 0.4.2 -> 0.4.12
pip install --upgrade catkin_pkg 
```
---

## Guidelines from Udacity

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
