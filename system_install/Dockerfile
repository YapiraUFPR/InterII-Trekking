FROM dustynv/l4t-pytorch:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO eloquent
ENV ROS_PYTHON_VERSION 3

#########################################
########### ROS2 Installation ###########
#########################################

# ros installation
RUN apt-get update && apt-get install curl gnupg2 lsb-release -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install ros-${ROS_DISTRO}-ros-base -y \
    && rm -rf /var/lib/apt/lists/*

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

# bootstrap rosdep
RUN rosdep init && rosdep update --include-eol-distros 

# setup colcon mixin and metadata
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-rosbag2 \
    && rm -rf /var/lib/apt/lists/*


###########################################
########### OpenCV Installation ###########
###########################################

# install dependancies
RUN apt-get update \
    && apt-get install --no-install-recommends -y cmake \
    && apt-get install --no-install-recommends -y gcc g++ \
    && apt-get install --no-install-recommends -y python3-dev python3-numpy libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev  libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev libeigen3-dev

# install libcudnn
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/cuda-ubuntu1804.pin \
#     && mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
#     && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/7fa2af80.pub \
#     && add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/ /" \
#     && apt-get update && apt-get install libcudnn8 libcudnn8-dev

# RUN python3 -m pip install opencv-python
# # build opencv
# RUN mkdir /opencv_installation
# WORKDIR /opencv_installation
# RUN git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git 
# RUN echo "/usr/local/cuda-10.2/lib64" >> /etc/ld.so.conf.d/nvidia.conf && ldconfig \
#     && mkdir opencv/build && cd opencv/build \
#     && cmake \
#         -D CMAKE_BUILD_TYPE=RELEASE \
#         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
#         -D WITH_TBB=ON \
#         -D ENABLE_FAST_MATH=1 \
#         -D CUDA_FAST_MATH=1 \
#         -D WITH_CUBLAS=1 \
#         -D WITH_CUDA=ON \
#         -D BUILD_opencv_cudacodec=OFF \
#         -D WITH_CUDNN=ON \
#         -D OPENCV_DNN_CUDA=ON \
#         -D CUDA_ARCH_BIN="5.3" \
#         -D CUDA_ARCH_PTX="sm_53" \
#         -D WITH_V4L=ON \
#         -D WITH_QT=OFF \
#         -D WITH_OPENGL=ON \
#         -D WITH_GSTREAMER=ON \
#         -D WITH_NVCUVID=OFF \
#         -D WITH_NVCUVENC=OFF \
#         -D OPENCV_GENERATE_PKGCONFIG=ON \
#         -D BUILD_opencv_python2=OFF \
#         -D PYTHON_EXECUTABLE=$(which python2) \
#         -D PYTHON3_EXECUTABLE=$(which python3) \
#         -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
#         -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
#         -D INSTALL_PYTHON_EXAMPLES=OFF \
#         -D INSTALL_C_EXAMPLES=OFF \
#         -D BUILD_EXAMPLES=OFF \
#         .. && make -j3 && make install && ldconfig


##########################################
########### Other dependancies ###########
##########################################

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

# install pip 3.7 requirements
# RUN apt-get update && apt-get install -y software-properties-common \
#     && add-apt-repository ppa:deadsnakes/ppa -y \
#     && apt-get install -y python3.7 python3.7-dev python3.7-distutils \
#     && python3.7 -m pip install --upgrade pip 

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
        pigpio \
        gpiozero \
        pytest \
        rdp \
        scipy \
        setuptools \
        smbus \
        Jetson.GPIO 

# append setup to .bashrc
RUN echo 'LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64' >> /root/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \