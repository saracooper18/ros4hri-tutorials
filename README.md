# ros4hri-tutorials

## Start the GitHub Codespaces

Start the Github Codespaces by clicking on `Code`, then `Codespaces`.

### Connect to the remote desktop

![Port forwarding](images/vscode-ports.png)

In the Codespaces' VSCode interface, click on the `PORTS` tab (next to
`TERMINAL`), and click on the *Forwarded Address* URL next to the `6080` port forward. Click on the
small globe icon to open a VNC connection to the remote desktop. The password is
`vscode`.

This desktop environment is connected to the Codespaces: any GUI application
that you start from the Codespaces' terminal (like `rviz`) will open in the remote desktop.


## Prepare the environment

### Install ROS noetic

We can follow the official [installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu).

In the terminal of your Codespaces, type:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base python3-rosdep python3-catkin-tools ros-noetic-rviz ros-noetic-rqt-image-view
```

We will also create a basic ROS workspace, so that we can compile ROS nodes:

```
mkdir -p ws/src
cd ws
catkin init
sudo rosdep init
rosdep update
```

We are also going to configure the container so that it always uses the default
Ubuntu Python:

```
mv /usr/local/python /usr/local/python-bak
sudo mv /usr/local/python /usr/local/python-bak
sudo mv /opt/conda /opt/conda-bak
```

### Start ROS

In the same terminal, source your ROS environment:

```
source /opt/ros/noetic
```

**Note: each time you open a new terminal, you first need to source your ROS
environment with this command.**

Then, start `roscore`

```
roscore
```

### Display the test bag file

Open a new terminal by clicking on the `+` at the right of the termnial panel.

Source the ROS environment, and play the pre-recorded bag file:

```
source /opt/ros/noetic/setup.bash
rosbag play --loop bags/severin-train.bag
```

Open yet another termnial, source ROS, an open `rqt_image_view`:

```
source /opt/ros/noetic/setup.bash
rqt_image_view
```

Switch to the remote desktop tab. You should see the RQT `image_view`
interface. Select the `/usb_cam/image_raw` topic in the drop-down list. It
should display the video stream.

![rqt_image_view](images/rqt_image_view.png)

We can alos visualize the bag file in `rviz`, the main ROS tool for data
visualization. Stop `rqt_image_view` (either close the window, or press Ctrl+C
in the terminal), and start `rviz` instead:

```
rviz
```

*Add* an `Image` plugin, and select the `/usb_cam/image_raw` topic like on the
screenshot below:

![rviz](images/rviz.png)

### Install hri_face_detect

We first want to detect faces in our test bag file.

We will use a ROS4HRI-compatible node for that purpose: [`hri_face_detect`](https://github.com/ros4hri/hri_face_detect/)

To install it:

First, let's get the code:

```
cd ws/src
git clone https://github.com/ros4hri/hri_face_detect.git
cd ..
```

Then, let's install the dependencies:

```
pip3 install mediapipe
rosdep install -r -y --from-paths src
```

Finally, build it:

```
catkin build hri_face_detect
```

#### Visualising the result

We can check that the faces are detected and published at ROS message by simply typing:

```
rostopic echo /humans/faces/tracked
```

We can also use `rviz` to display the faces with the facial landmarks. First,
install the `rviz` ROS4HRI plugin:

```
sudo apt install ros-noetic-hri-rviz
```

Then, start `rviz` and enable the `Humans` plugins:

![rviz human plugin](images/rviz-humans-plugin.png)

Configure the plugin to use the `/usb_cam/image_raw` topic. You should see the
faces being displayed:

![rviz displaying faces](images/rviz-faces.png)


### Install hri_fullbody

Next, let's detect 3D skeletons in the image.

We will use the ROS4HRI-compatible [`hri_fullbody`](https://github.com/ros4hri/hri_fullbody/) node.

To install it:

First, let's get the code:

```
cd ws/src
git clone https://github.com/ros4hri/hri_fullbody.git
cd ..
```

Then, let's install the dependencies:

```
rosdep install -r -y --from-paths src
```

Finally, build it:

```
catkin build hri_fullbody
```

