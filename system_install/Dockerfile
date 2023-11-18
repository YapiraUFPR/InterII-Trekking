# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-geographic-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-angles \
    ros-humble-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

# install libraries
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    build-essential \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    libboost-filesystem-dev \
    libglew-dev \
    libeigen3-dev \
    libgeographic-dev


# install pip requirements
RUN apt-get update && apt-get install python3-pip -y \
    &&  python3 -m pip install \
        Adafruit_Blinka \
        adafruit_circuitpython_bitbangio \
        adafruit_circuitpython_bno08x \
        adafruit_circuitpython_motor \
        adafruit_circuitpython_tcs34725 \
        adafruit_circuitpython_vl53l0x \
        adafruit_circuitpython_ina219 \
        adafruit_circuitpython_mpu6050 \
        numpy \
        opencv_python \
        pigpio \
        gpiozero \
        pytest \
        rdp \
        scipy \
        setuptools \
        smbus \
        RPi.GPIO 

# append setup to .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source /home/user/ws/install/local_setup.bash" >> /root/.bashrc \
    && echo "export ROS_HOSTNAME={$(hostname -I | awk '{print $1}')}" >> /root/.bashrc \
    && echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> /root/.bashrc

RUN bash