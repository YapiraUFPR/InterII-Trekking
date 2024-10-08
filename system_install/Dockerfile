FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO humble
ENV ROS_PYTHON_VERSION 3
ENV ROS_ROOT /opt/ros/${ROS_DISTRO}

# install bootstrap tools
RUN apt-get update \
    && apt-get install --no-install-recommends -y build-essential git \
    && python3 -m pip install \
        colcon-common-extensions \
        colcon-mixin \
        rosdep \
        vcstool \
        argcomplete \
        cython \
    && rm -rf /var/lib/apt/lists/*

# setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

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
    libgeographic-dev \
    nlohmann-json-dev \
    libasio-dev \
    libboost-all-dev \
    libssl-dev \
    libwebsocketpp-dev \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT} \
    && rosinstall_generator --deps --rosdistro ${ROS_DISTRO} \
    geographic_info \
    common_msgs \
    angles \
    > ros2.humble.packages.rosinstall \
    && vcs import src < ros2.humble.packages.rosinstall
RUN cd ${ROS_ROOT} && colcon build --merge-install

# install python libraries
RUN apt-get update && python3 -m pip install --upgrade pip \
&& python3 -m pip install \
    numpy \
    Adafruit_Blinka \
    adafruit_circuitpython_bitbangio \
    adafruit_circuitpython_bno08x \
    adafruit_circuitpython_motor \
    adafruit_circuitpython_tcs34725 \
    adafruit_circuitpython_vl53l0x \
    adafruit_circuitpython_ina219 \
    adafruit_circuitpython_mpu6050 \
    adafruit_circuitpython_pca9685 \
    pytest \
    rdp \
    scipy \
    setuptools \
    smbus \
    Jetson.GPIO 

# For packages that we need to compile from source
WORKDIR /install_deps 

# Install torch and torchvision
RUN wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl -O torch-1.9.0-cp36-cp36m-linux_aarch64.whl \
    && apt-get install python3-pip libopenblas-base libopenmpi-dev \
    && python3 -m pip install Cython pandas \
    && python3 -m pip install torch-1.9.0-cp36-cp36m-linux_aarch64.whl \
    && python3 -m pip install torchvision

# Install CUDA mesh

# Install ceres 1.14
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    cmake libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev libsuitesparse-dev
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz \
    && tar -xf ceres-solver-1.14.0.tar.gz \
    && mkdir ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-1.14.0 -DUSE_CUDA=ON \
    && make -j3 \
    && make install

# Install jetson-utils
RUN apt-get install -y --no-install-recommends \
		dialog \
		libglew-dev \
		glew-utils \
		gstreamer1.0-libav \
		gstreamer1.0-nice \
		libgstreamer1.0-dev \
		libgstrtspserver-1.0-dev \
		libglib2.0-dev \
		libsoup2.4-dev \
		libjson-glib-dev \
		python3-pip \
		python3-packaging \
		qtbase5-dev \
		avahi-utils \
        libgstreamer-plugins-bad1.0-dev
# Need to disable NVMM to use our IMX219 camera. Refer to https://github.com/dusty-nv/jetson-utils/issues/101
RUN git clone https://github.com/dusty-nv/jetson-utils \
    && cd jetson-utils && mkdir build \
    && cd build \ 
    && cmake -DENABLE_NVMM=off ../ \ 
    && make -j$(nproc) && make install \
    && ldconfig