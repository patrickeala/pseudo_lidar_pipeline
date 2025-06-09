FROM ros:noetic-ros-core

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive

# Remove old ROS sources and key
RUN rm -f /etc/apt/sources.list.d/ros-latest.list \
    && rm -f /etc/apt/sources.list.d/ros1-latest.list \
    && rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Install curl, add new ROS key and repo
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main' > /etc/apt/sources.list.d/ros1-latest.list


    
# Step 2: Install ROS packages and tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    git \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-pcl-ros \
    ros-noetic-compressed-image-transport \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Step 3: Install Python machine learning dependencies
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
        numpy \
        opencv-python \
        torch \
        torchvision \
        segmentation-models-pytorch \
        matplotlib \
        rospkg \
        catkin-pkg
