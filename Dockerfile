FROM osrf/ros:noetic-desktop

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-hri-rviz \
    evince \
    wget \
    git \
    python3-pil \
    python3-opencv \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-pybind11-catkin \
    ros-noetic-robot-state-publisher \
    ros-noetic-image-geometry \
    ros-noetic-diagnostic-msgs \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    python3-numpy \
    ros-noetic-hri-msgs \
    python3-tk \
    ros-noetic-rosconsole \
    ros-noetic-human-description \
    ros-noetic-catkin \
    python3-scipy \
    pybind11-dev \
    libdlib-dev \
    libblas-dev \
    liblapack-dev \
    ros-noetic-hri \
    ros-noetic-rospy \
    ros-noetic-usb-cam

# Update rosdep
RUN rosdep update

# Install Python packages
RUN pip3 install mediapipe ikpy graphviz

# Create ROS workspace and clone repositories
RUN mkdir -p ~/ros4hri_ws/src && \
    cd ~/ros4hri_ws/src && \
    git clone --branch master https://github.com/ros4hri/hri_fullbody.git && \
    git clone --branch main https://github.com/ros4hri/hri_face_detect.git && \
    git clone --branch master https://github.com/ros4hri/hri_face_identification.git && \
    git clone --branch master https://github.com/ros4hri/hri_person_manager.git && \
    git clone --branch noetic-devel https://github.com/saracooper18/ros4hri-tutorials.git && \
    git clone https://github.com/ros4hri/pyhri

# Install ROS dependencies
RUN cd ~/ros4hri_ws && \
    rosdep install -r -y --from-paths src

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd ~/ros4hri_ws && catkin build"

# Move .so file
RUN mv ~/ros4hri_ws/devel/lib/yunet_detector.cpython-38-x86_64-linux-gnu.so \
    ~/ros4hri_ws/devel/lib/python3/dist-packages/

# Source workspace in .bashrc
RUN echo "source ~/ros4hri_ws/devel/setup.bash" >> ~/.bashrc


