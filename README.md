# ros4hri-tutorials

## Start the GitHub Codespaces

1. Fork this repository to your own account
2. Start the Github Codespaces by clicking on `Code`, then `Codespaces`

## Prepare the environment

### Install ROS noetic

We can follow the official [installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu).

In the terminal of your Codespaces, type:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-noetic-ros-base
```

### Start ROS

In the same terminal, source your ROS environment:

```
source /opt/ros/noetic
```

**Note: each time you open a new terminal, you first need to source your ROS environment with this command.**

Then, start `roscore`

```
roscore
```

### Install