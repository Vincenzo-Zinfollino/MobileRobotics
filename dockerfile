
FROM nvcr.io/nvidia/isaac-sim:4.5.0

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

    # Set the timezone
ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Setup the sources
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    RUN  apt-get -y update \
     && apt install -y ros-dev-tools

    RUN apt-get update\
    && apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6


    # Install ROS 2 packages
    RUN apt-get update && apt-get upgrade -y && \
        apt-get install -y ros-humble-desktop 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    nano \
    iputils-ping \
    wget \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro humble

# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN apt-get update && apt-get install git-lfs
RUN git lfs install --skip-repo




RUN apt -y update \
    && apt install python3-genmsg \
    && apt -y install gnupg wget \
    && apt -y install software-properties-common  \
    && apt-get install -y apt-transport-https \
    && add-apt-repository universe 


RUN  apt update \
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" |  tee -a /etc/apt/sources.list 




RUN  DEBIAN_FRONTEND=noninteractive  mkdir -p  /workspaces/isaac_ros-dev/src \
    echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> .bashrc\
    source ~/.bashrc

RUN cd ${ISAAC_ROS_WS}/src && \
    git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

RUN sudo apt-get install -y curl jq tar




ENTRYPOINT ["/ros_entrypoint.sh"]
# Run bash
CMD ["bash"]


