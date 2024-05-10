FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        curl \
        wget \
        gnupg \
        lsb-release \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Ensure we have universe
RUN apt-get update \
        && apt-get install -y software-properties-common \
        && add-apt-repository universe

# NVIDIA repository keys: https://developer.nvidia.com/blog/updating-the-cuda-linux-gpg-repository-key/
RUN apt-key del 7fa2af80 && mkdir -p /tmp && cd /tmp \
        && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb \
        && dpkg -i cuda-keyring_1.0-1_all.deb \
        && rm cuda-keyring_1.0-1_all.deb \
        && add-apt-repository --remove 'deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /'

# Add Isaac apt repository
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add -
RUN echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | tee -a /etc/apt/sources.list

# Fundamentals
RUN apt-get update && apt-get install -y \
        apt-transport-https \
        bash-completion \
        build-essential \
        ca-certificates \
        clang-format \
        cmake \
        curl \
        git \
        gnupg2 \
        iputils-ping \
        locales \
        lsb-release \
        rsync \
        software-properties-common \
        wget \
        vim \
        unzip \
        mlocate \
	libgoogle-glog-dev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Python basics
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Video utilities
RUN apt-get update && apt-get install -y \
        v4l-utils \
        mesa-utils \
        libcanberra-gtk-module \
        libcanberra-gtk3-module \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Core dev libraries
RUN apt-get update && apt-get install -y \
        libasio-dev \
        libbullet-dev \
        libtinyxml2-dev \
        libcunit1-dev \
        libyaml-cpp-dev \
        libopencv-dev \
        python3-opencv \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Python3 (PIP)
RUN python3 -m pip install -U \
        argcomplete \
        autopep8 \
        flake8==4.0.1 \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        onnx \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pydocstyle \
        scikit-learn

# Install Git-LFS
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
        apt-get update && apt-get install -y \
        git-lfs \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Upgrade cmake to 3.22.1 to match Ubuntu 22.04
# Key rotation 2024-01-10
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null \
    && apt-get update \
    && apt-get remove -y cmake && apt-get purge -y cmake && apt-get remove -y cmake-data && apt-get purge -y cmake-data \
    && apt-get install -y cmake=3.22.1-0kitware1ubuntu20.04.1 cmake-data=3.22.1-0kitware1ubuntu20.04.1 \
    && cmake --version \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Pytorch
RUN python3 -m pip install -U --extra-index-url https://download.pytorch.org/whl/cu113 \
        torch \
        torchvision \
        torchaudio

# Install VPI packages
ARG HAS_GPU="true"
RUN if [ "$HAS_GPU" = "true" ]; then \
	set -e ; \
        apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc ; \
        add-apt-repository 'deb http://repo.download.nvidia.com/jetson/x86_64/focal r35.4 main' ; \
        apt-get update ; \
        apt-get install libnvvpi2 vpi2-dev ; \
        rm -rf /var/lib/apt/lists/* ; \
        apt-get clean ; \
    fi

# Install Tao converter
RUN mkdir -p /opt/nvidia/tao && \
    cd /opt/nvidia/tao && \
    wget https://developer.nvidia.com/tao-converter-80 && \
    unzip tao-converter-80 && \
    chmod 755 $(find /opt/nvidia/tao -name "tao-converter") && \
    ln -sf $(find /opt/nvidia/tao -name "tao-converter") /opt/nvidia/tao/tao-converter && \
    rm tao-converter-80

ENV PATH="${PATH}:/opt/nvidia/tao"

# Update environment
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/tritonserver/lib"

# Add MQTT binaries and libraries
RUN apt-add-repository ppa:mosquitto-dev/mosquitto-ppa \
    && apt-get update \
    && apt-get install -y mosquitto mosquitto-clients \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

RUN python3 -m pip install -U \
        paho-mqtt

# Compression dependencies
RUN apt-get update && apt-get install -y \
        libv4l-dev \
        kmod \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install NVIDIA v4l2 extensions for h264 support
RUN apt-get update && apt-get install -y nvv4l2

# Upgrade libc to resolve vulnerabilities including CVE-2019-11477
RUN apt-get update && apt-get install -y --only-upgrade \
       linux-libc-dev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Downgrade protobuf
RUN python3 -m pip install \
        protobuf==3.20.1

# GPU usage monitoring on x86
RUN python3 -m pip install \
    gpustat==0.6.0

# Python3 (PIP)
RUN python3 -m pip install -U \
    mailcap-fix

# Resolve vulnerability in mailcap.py by removing it (CVE-2015-20107)
RUN rm -f /opt/tritonserver/backends/dali/conda/envs/dalienv/lib/python3.8/mailcap.py

# Remove any platform entangling dependencies
RUN touch \
    /usr/lib/x86_64-linux-gnu/libcuda.so \
    /usr/lib/x86_64-linux-gnu/libnvcuvid.so \
    /usr/lib/x86_64-linux-gnu/libnvidia.so \
    /usr/lib/firmware \
    /usr/local/cuda/compat/lib || true

RUN rm -rf \
    /usr/lib/x86_64-linux-gnu/libcuda.so* \
    /usr/lib/x86_64-linux-gnu/libnvcuvid.so* \
    /usr/lib/x86_64-linux-gnu/libnvidia-*.so* \
    /usr/lib/firmware \
    /usr/local/cuda/compat/lib || true

# Restore symlink for nvencode
RUN ln -f -s /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-encode.so

# Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources
RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Isaac apt repository
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' /etc/apt/sources.list || \
    echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | tee -a /etc/apt/sources.list

# ROS fundamentals
RUN apt-get update && apt-get install -y \
        build-essential \
        git \
        libyaml-cpp-dev \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        wget \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# ROS Python fundamentals
RUN python3 -m pip install -U \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        setuptools==65.7.0

# Basics
RUN apt-get update && apt-get install -y \
        curl \
        gnupg \
        locales \
        lsb-release \
        python3-bloom \
        libxtensor-dev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y ros-humble-ros-base \
    ros-humble-angles \
    ros-humble-apriltag \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-cv-bridge \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-diagnostic-updater \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-navigation2 \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-image-view \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Build Essentials
RUN apt-get update && apt-get install -y \
    build-essential \
    devscripts \
    dh-make \
    quilt \
    fakeroot \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Setup rosdep
RUN rosdep init \
    && mkdir -p /etc/ros/rosdep/sources.list.d/ \
    && curl -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml https://isaac.download.nvidia.com/isaac-ros/extra_rosdeps.yaml \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && rosdep update

# Install negotiated from source
RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/osrf/negotiated && cd negotiated && git checkout master \
    && source ${ROS_ROOT}/setup.bash \
    && cd negotiated_interfaces && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd negotiated && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install vda5050_msgs from source
RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ipa320/vda5050_msgs.git vda5050_root \
    && cd vda5050_root && git checkout ros2 && cd .. \
    && mv vda5050_root/vda5050_msgs/ vda5050_msgs \
    && rm -rf vda5050_root \
    && source ${ROS_ROOT}/setup.bash \
    && cd vda5050_msgs && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb \
    && cd ../ && rm -Rf src build log \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-humble-camera-info-manager \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbridge-suite \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean
