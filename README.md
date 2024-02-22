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

### Initial environment preparation

Your environment (called a *devcontainer*) is based on [ROS
noetic](http://wiki.ros.org/noetic). It already contains almost all the standard
ROS tools that we need. We only add the `catkin` command-line tool:

```
sudo apt update
sudo apt install python3-pip python3-catkin-tools
```

Next, we create a basic ROS *workspace*, so that we can compile ROS nodes:

```
mkdir -p ws/src
cd ws
catkin init
catkin config --install
cd ..
rosdep update
```

## Playing ROS 'bags'

ROS bags are files containing ROS data (ROS *messages*) that have been
previously recorded. We can *play* ROS bag to re-create the datastream as they
were when they were recorded.

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

Type `rostopic list` to list the available ROS topic (ie, the ROS data
channels). You should see at least `/usb_cam/image_raw` that contains the raw
image pixels data.

Open yet another terminal, source ROS, and open `rqt_image_view`:

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


## Face detection

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

### Start the face detection node

The `hri_face_detect` node has been installed in the `install/` subfolder. We
need to tell ROS to look into that folder when starting a node. Type:

```
source ./install/setup.bash
```

Then, you can start the face detection node, remapping the default image topic
to the one actually published by the camera:

```
rosrun hri_face_detect detect image:=/usb_cam/image_raw
```

You should immediately see on the console that some faces are indeed detected.
Let's visualise them.


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

