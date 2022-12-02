# UoB IR Group 10 Project

TLDR; after installation, run `roslaunch socspioneer collabslam.launch` in order to get two robots exploring and SLAM'ing the map. Run `rosrun socspioneer mapMerge.py` to start the map merging algorithm, which will take in the current two maps produced by the exploring robots and attempt to merge them. It'll save the result as a `.jpg` in the directory the command is run in.

## Installation

**NOTE**: *This part assumes basic understanding of Linux terminal and
commandline usage. Basic understanding of ROS workflow and package
organisation is also important (eg. [Building a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).*

### Ideal Working Environment

- Ubuntu 20.04
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
(desktop-full)

**NOTE**: *Installing ROS Noetic by default installs Python 3 packages. For compatibility and lack of conflicts, stick to
using Python 3 when writing Python nodes.*

### Install dependencies

- `sudo apt install ros-$ROS_DISTRO-pr2-teleop ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-map-server`.

You may need to install additional python packages as required

### Build package

- Clone this repo to the `src` directory of your catkin workspace.
- Build the catkin workspace (`catkin_make` or `catkin build`).

**NOTE: The catkin workspace should be sourced each time a new
terminal session is loaded (run `source devel/setup.bash`). Alternatively,
add the line `source <catkin_ws>/devel/setup.bash` to your `.bashrc`
file to avoid repeating it every time.**
