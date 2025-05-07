# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

ARG PLATFORM=amd64
ARG BASE_IMAGE=nvidia/cuda:12.6.1-devel-ubuntu22.04

# --------------------------------------------------------------------------------------------------
# https://docs.nvidia.com/deeplearning/frameworks/user-guide/index.html
# https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html
FROM nvcr.io/nvidia/tritonserver:24.08-py3 AS base-amd64

FROM nvcr.io/nvidia/12.6.11-devel:12.6.11-devel-aarch64-ubuntu22.04 AS base-arm64
# --------------------------------------------------------------------------------------------------

FROM base-${PLATFORM} AS common
ARG PLATFORM

# Store list of packages (must be first)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/base-start-packages.csv

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]


#19
ENV ROS_DISTRO=humble
ENV ROS_ROOT=humble_ws
ENV ROS_PYTHON_VERSION=3

# Ensure we have universe
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update

# Fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    apt-transport-https \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    curl \
    git \
    git-lfs \
    gnupg2 \
    iputils-ping \
    libgoogle-glog-dev \
    locales \
    lsb-release \
    mlocate \
    rsync \
    tar \
    unzip \
    vim \
    wget \
    zlib1g-dev

# Python basics
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    python3-dev \
    python3-distutils \
    python3-flake8 \
    python3-pip \
    python3-pybind11 \
    python3-pytest \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-cov \
    python3-venv \
    python3-zmq \
    python3.10 \
    python3.10-venv

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Add Isaac apt repository
RUN --mount=type=cache,target=/var/cache/apt \
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | tee -a /etc/apt/sources.list \
    && apt-get update

# Core dev libraries
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    gfortran \
    graphicsmagick-libmagick-dev-compat \
    jq \
    kmod \
    lcov \
    libasio-dev \
    libassimp-dev \
    libatlas3-base \
    libatlas-base-dev \
    libblas3 \
    libboost-all-dev \
    libboost-dev \
    libbullet-dev \
    libceres-dev \
    libcunit1-dev \
    libffi7 \
    libfreetype6 \
    libgraphicsmagick++1-dev \
    libhidapi-libusb0 \
    libinput10 \
    libjpeg8 \
    liblapack3 \
    libmnl0 \
    libmnl-dev \
    libncurses5-dev \
    libode-dev \
    libopenblas0 \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 \
    libopenmpi3 \
    libpcap-dev \
    libpcl-dev \
    libpython3.10 \
    libsuitesparse-dev \
    libtinyxml2-dev \
    libturbojpeg \
    libunwind8 \
    libv4l-0 \
    libv4l-dev \
    libx264-dev \
    libxaw7-dev \
    libyaml-cpp-dev \
    linuxptp \
    llvm-14 \
    nlohmann-json3-dev \
    patchelf \
    python3-opencv=4.5.4+dfsg-9ubuntu4 \
    python3-scipy

# Downgrade to setuptools < 70.0.0
# https://github.com/pypa/setuptools/issues/4483
RUN python3 -m pip install -U \
    setuptools==65.7.0

# Python3 PIP buildtools
RUN python3 -m pip install -U \
    Cython \
    ninja \
    wheel

# Python3 PIP packages
RUN python3 -m pip install \
    argcomplete \
    autopep8 \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    gpustat==0.6.0 \
    importlib_resources \
    networkx \
    "numpy>=1.24.4,<2" \
    numpy-quaternion \
    onnx \
    pydocstyle \
    pymongo \
    pyyaml \
    "scipy>=1.7.0" \
    scikit-image \
    scikit-learn \
    "setuptools_scm>=6.2" \
    tqdm \
    trimesh \
    "warp-lang==0.11.0" \
    packaging==24.2 \
    "mapbox_earcut==1.0.1" \
    "yourdfpy==0.0.53" \
    opentelemetry-api==1.28.1 \
    opentelemetry-sdk==1.28.1 \
    opentelemetry-exporter-otlp==1.28.1

# Add MQTT binaries and libraries
RUN --mount=type=cache,target=/var/cache/apt \
    apt-add-repository ppa:mosquitto-dev/mosquitto-ppa \
    && apt-get update && apt-get install -y \
    mosquitto \
    mosquitto-clients

# Install Node.js
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs

# Install Yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - && \
    echo "deb https://dl.yarnpkg.com/debian/ stable main" > /etc/apt/sources.list.d/yarn.list && \
    apt-get update && apt-get install -y yarn && \
    corepack enable

# Install CuPy and HDBSCAN
RUN python3 -m pip install -U \
    cupy-cuda12x \
    hdbscan

# Setup Jetson debian repositories
RUN --mount=type=cache,target=/var/cache/apt \
    apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc ; \
    if [[ ${PLATFORM} == 'arm64' ]]; then \
    echo 'deb https://repo.download.nvidia.com/jetson/common r36.4 main' > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && echo 'deb https://repo.download.nvidia.com/jetson/t234 r36.4 main' >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list ; \
    elif [[ ${PLATFORM} == 'amd64' ]]; then \
    add-apt-repository "deb http://repo.download.nvidia.com/jetson/x86_64/$(lsb_release -cs) r36.4 main" ; \
    else \
    echo "Unrecognized platform: ${PLATFORM}" && exit 1 ; \
    fi ; \
    apt-get update

# Setup CUDA repositories
RUN --mount=type=cache,target=/var/cache/apt \
    if [[ ${PLATFORM} == 'arm64' ]]; then \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb ; \
    elif [[ ${PLATFORM} == 'amd64' ]]; then \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb ; \
    else \
    echo "Unrecognized platform: ${PLATFORM}" && exit 1 ; \
    fi ; \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm -Rf cuda-keyring_1.1-1_all.deb ; \
    apt-get update

# Install VPI
RUN --mount=type=cache,target=/var/cache/apt \
    if [[ ${PLATFORM} == 'arm64' ]]; then \
    # This is a temporary workaround required to install pva-allow-2 in docker which will not be necessary next release
    apt-get install pva-allow-2 || true ; \
    rm /var/lib/dpkg/info/pva-allow-2.post* ; \
    dpkg --configure pva-allow-2 ; \
    mkdir -p /etc/pva/allow.d && mkdir -p /tmp/vpi_dev \
    && cd /tmp/vpi_dev \
    && wget https://repo.download.nvidia.com/jetson/common/pool/main/v/vpi3-dev/vpi3-dev_3.2.4_arm64.deb \
    && wget https://repo.download.nvidia.com/jetson/common/pool/main/libn/libnvvpi3/libnvvpi3_3.2.4_arm64.deb \
    && dpkg -i libnvvpi3_3.2.4_arm64.deb \
    && dpkg -i vpi3-dev_3.2.4_arm64.deb \
    && cd /tmp && rm -Rf /tmp/vpi_dev ; \
    fi ; \
    apt-get update && apt-get install -y \
    libnvvpi3 \
    vpi3-dev

# Install cuDSS
RUN --mount=type=cache,target=/var/cache/apt \
    if [[ ${PLATFORM} == 'arm64' ]]; then \
    wget https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-aarch64/libcudss-linux-aarch64-0.3.0.9_cuda12-archive.tar.xz && \
    tar -xvf libcudss-linux-aarch64-0.3.0.9_cuda12-archive.tar.xz -C /usr/local && \
    rm libcudss-linux-aarch64-0.3.0.9_cuda12-archive.tar.xz && \
    cp -r /usr/local/libcudss-linux-aarch64-0.3.0.9_cuda12-archive /usr/local/libcudss-linux-0.3.0.9_cuda12-archive ; \
    elif [[ ${PLATFORM} == 'amd64' ]]; then \
    wget https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-x86_64/libcudss-linux-x86_64-0.3.0.9_cuda12-archive.tar.xz && \
    tar -xvf libcudss-linux-x86_64-0.3.0.9_cuda12-archive.tar.xz -C /usr/local && \
    rm libcudss-linux-x86_64-0.3.0.9_cuda12-archive.tar.xz && \
    cp -r /usr/local/libcudss-linux-x86_64-0.3.0.9_cuda12-archive /usr/local/libcudss-linux-0.3.0.9_cuda12-archive ; \
    else \
    echo "Unrecognized platform: ${PLATFORM}" && exit 1 ; \
    fi

# Based on cuDSS version (0.3.0.9) and architecture which is not expected to change frequently
ENV CUDSS_DIR=/usr/local/libcudss-linux-0.3.0.9_cuda12-archive
ENV CMAKE_PREFIX_PATH=$CUDSS_DIR:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=$CUDSS_DIR/lib:$LD_LIBRARY_PATH

# Install CV-CUDA
RUN --mount=type=cache,target=/var/cache/apt \
    cd /tmp ; \
    if [[ ${PLATFORM} == 'arm64' ]]; then \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-lib-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    dpkg -i nvcv-lib-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-dev-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    dpkg -i nvcv-dev-0.5.0_beta_DP-cuda12-aarch64-linux.deb ; \
    elif [[ ${PLATFORM} == 'amd64' ]]; then \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i nvcv-lib-0.5.0_beta-cuda12-x86_64-linux.deb && \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb && \
    dpkg -i nvcv-dev-0.5.0_beta-cuda12-x86_64-linux.deb ; \
    else \
    echo "Unrecognized platform: ${PLATFORM}" && exit 1 ; \
    fi

# --------------------------------------------------------------------------------------------------

FROM common AS extended-arm64

# Update environment
RUN update-alternatives --install /usr/bin/llvm-config llvm-config /usr/bin/llvm-config-14 14
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi3/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.6/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra/weston:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu-host"
ENV PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}"

# Install CUDA packages
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends --force-yes \
    cuda-cudart-12-6 \
    cuda-libraries-12-6 \
    cuda-nvml-dev-12-6 \
    cuda-sanitizer-12-6 \
    cuda-toolkit-12-6 \
    libcublas-12-6 \
    libcudnn9 \
    libcusparse-12-6 \
    libnpp-12-6

# Install TensorRT
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    libnvinfer10 \
    libnvinfer-plugin10 \
    libnvonnxparsers10 \
    libnvinfer-dispatch10 \
    libnvinfer-bin \
    tensorrt

ENV TRT_LIB_PATH="/usr/lib/aarch64-linux-gnu"
ENV TRT_INCLUDE_PATH="/usr/include/aarch64-linux-gnu"

# Install pva-allow-2 as a workaround
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p /tmp/pva && cd /tmp/pva \
    && wget https://repo.download.nvidia.com/jetson/common/pool/main/p/pva-allow-2/pva-allow-2_2.0.0~rc3_all.deb \
    && dpkg -i pva-allow-2_2.0.0~rc3_all.deb || true \
    && rm /var/lib/dpkg/info/pva-allow-2.post* \
    && dpkg --configure pva-allow-2 \
    && cd /tmp && rm -Rf /tmp/pva


# PyTorch (NV CUDA edition)
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
RUN python3 -m pip install --no-cache \
    https://developer.download.nvidia.cn/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
    libb64-0d \
    libre2-9 \
    rapidjson-dev \
    libopenblas-dev \
    libarchive-dev \
    libcusparselt0 \
    libcusparselt-dev

# Install Triton server from https://github.com/triton-inference-server/server/releases/tag/v2.49.0
RUN  --mount=type=cache,target=/var/cache/apt \
    cd /opt \
    && wget https://github.com/triton-inference-server/server/releases/download/v2.49.0/tritonserver2.49.0-igpu.tar.gz \
    && tar -xzvf tritonserver2.49.0-igpu.tar.gz \
    && chmod 644 /opt/tritonserver/backends/tensorflow/libtensorflow_cc.so.2 \
    && chmod 644 /opt/tritonserver/backends/tensorflow/libtensorflow_framework.so.2 \
    && rm tritonserver2.49.0-igpu.tar.gz

# Remove security vulnerability in Java examples
RUN rm -rf /opt/tritonserver/clients/java/examples

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/tritonserver/lib"

# Install boost version >= 1.78 for boost::span
# Current libboost-dev apt packages are < 1.78, so install from tar.gz
RUN --mount=type=cache,target=/var/cache/apt \
    wget -O /tmp/boost.tar.gz \
    https://archives.boost.io/release/1.80.0/source/boost_1_80_0.tar.gz \
    && (cd /tmp && tar xzf boost.tar.gz) \
    && cd /tmp/boost_1_80_0 \
    && ./bootstrap.sh --prefix=/usr \
    && ./b2 install \
    && rm -rf /tmp/boost*


# Install jtop
RUN python3 -m pip install -U \
    jetson-stats

# Compile ffmpeg without gpl and nonfree
RUN apt-get install yasm && wget https://ffmpeg.org/releases/ffmpeg-4.4.2.tar.bz2 && tar xjvf ffmpeg-4.4.2.tar.bz2 && cd ffmpeg-4.4.2 && \
    ./configure --prefix=/usr --toolchain=hardened --enable-shared \
    --libdir=/usr/lib/aarch64-linux-gnu --incdir=/usr/include/aarch64-linux-gnu --arch=arm64 \
    --disable-stripping && make -j 4 && make install

# --------------------------------------------------------------------------------------------------

FROM common AS extended-amd64

# Update environment
ENV TRT_LIB_PATH="/usr/lib/x86_64-linux-gnu"
ENV TRT_INC_PATH="/usr/include/x86_64-linux-gnu"


# Install nvv4l2 for GXF Multimedia h264 codec
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    nvv4l2 \
    && ln -s /usr/lib/x86_64-linux-gnu/libnvcuvid.so.1 /usr/lib/x86_64-linux-gnu/libnvcuvid.so \
    && ln -s /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-encode.so

# Pytorch
RUN python3 -m pip install -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch \
    torchvision \
    torchaudio

# Update environment
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/tritonserver/lib"

# Compile a ffmpeg without gpl and nonfree
RUN apt-get install -y yasm && wget https://ffmpeg.org/releases/ffmpeg-4.4.2.tar.bz2 && tar xjvf ffmpeg-4.4.2.tar.bz2 && cd ffmpeg-4.4.2 && \
    ./configure --prefix=/usr --toolchain=hardened --enable-shared \
    --libdir=/usr/lib/x86_64-linux-gnu --incdir=/usr/include/x86_64-linux-gnu --arch=amd64 \
    --disable-stripping && make -j 4 && make install

# --------------------------------------------------------------------------------------------------

FROM extended-${PLATFORM} AS base

# Install Ceres
RUN git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout 6fb3dae4eeef855568e47ebbb29a8ba4f3c9153f && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DUSE_CUDA=ON -Dcudss_DIR=$CUDSS_DIR && \
    make -j 4 && \
    make install && \
    sed -i 's/find_dependency(cudss 0.3.0)/find_dependency(cudss)/' /usr/local/lib/cmake/Ceres/CeresConfig.cmake

# Install Protobuf
# v5.26.0 is same tag as v26.0
RUN git clone https://github.com/protocolbuffers/protobuf.git -b v5.26.0 && \
    cd protobuf && \
    git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake .. \
    -Dprotobuf_BUILD_TESTS=OFF \
    -Dprotobuf_MSVC_STATIC_RUNTIME=OFF \
    -DABSL_PROPAGATE_CXX_STD=ON \
    -Dprotobuf_BUILD_SHARED_LIBS=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    cmake --build . --config Release && \
    make install

# Specify non-root admin user for container
ARG USERNAME=admin
ENV USERNAME=${USERNAME}

# Install prerequisites
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    gosu \
    sudo \
    udev

# Copy scripts
RUN mkdir -p /usr/local/bin/scripts
COPY scripts/*entrypoint.sh /usr/local/bin/scripts/
RUN chmod +x /usr/local/bin/scripts/*.sh || true

# Copy script additions
RUN mkdir -p /usr/local/bin/scripts/entrypoint_additions
COPY scripts/entrypoint_addition[s]/*.sh /usr/local/bin/scripts/entrypoint_additions/
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/*.sh || true

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY middleware_profile[s]/*profile.xml /usr/local/share/middleware_profiles/

# Store list of packages (must be last)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/base-end-packages.csv

# Install yq
RUN wget https://github.com/mikefarah/yq/releases/latest/download/yq_linux_amd64 -O /usr/local/bin/yq \
&& chmod +x /usr/local/bin/yq


#----------------------------------
#-INSTALLING ROS2:Humble-
#----------------------------------

    
# Store list of packages (must be first)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-start-packages.csv

# Env setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop
RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# Add ROS 2 apt repository
RUN --mount=type=cache,target=/var/cache/apt \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# ROS fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
        devscripts \
        dh-make \
        fakeroot \
        libxtensor-dev \
        python3-bloom \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        quilt

# Upgrade system setuptools
RUN python3 -m pip install --upgrade --force-reinstall --target=/usr/lib/python3/dist-packages setuptools==65.7.0

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
        matplotlib \
        pandas \
        rosbags \
        boto3 \
        setuptools==65.7.0

# Install ROS 2 Humble
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-ros-base \
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
    ros-humble-diagnostics \
    ros-humble-diagnostic-aggregator \
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
    ros-humble-nav2-graceful-controller \
    ros-humble-navigation2 \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbridge-suite \
    ros-humble-rosx-introspection \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-robot-monitor \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-vision-msgs-rviz-plugins

# Setup rosdep
COPY rosdep/extra_rosdeps.yaml /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml
RUN --mount=type=cache,target=/var/cache/apt \
    rosdep init \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && rosdep update

####### -- Install updated packages over installed debians

# Install negotiated from source
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/osrf/negotiated && cd negotiated && git checkout master \
    && source ${ROS_ROOT}/setup.bash \
    && cd negotiated_interfaces && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd negotiated && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install image_proc from 55bf2a38 with backported resize node fix
# https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15
# Revert breaking QoS changes in https://github.com/ros-perception/image_pipeline/pull/814
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-perception/image_pipeline.git && cd image_pipeline && git checkout 55bf2a38c327b829c3da444f963a6c66bfe0598f \
    && git config user.email "builder@nvidia.com" && git config user.name "NVIDIA Builder" \
    && git remote add fork https://github.com/schornakj/image_pipeline.git && git fetch fork && git cherry-pick 969d6c763df99b42844742946f7a70c605a72a15 \
    && source ${ROS_ROOT}/setup.bash \
    && cd image_proc && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb \
    && echo "image_pipeline (image_proc) https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15 on 55bf2a38" >> ${ROS_ROOT}/VERSION \
    && cd ../ && rm -Rf src build log

# Install patched rclcpp package with backported multithreadedexecutor fix
# https://github.com/ros2/rclcpp/commit/232262c02a1265830c7785b7547bd51e1124fcd8
COPY patches/rclcpp-disable-tests.patch /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && export RCLCPP_VERSION="release/humble/rclcpp/$(apt-cache madison ros-humble-rclcpp | grep -m1 -oP 'ros-humble-rclcpp \| \K[^j]+(?=jammy)')" \
    && echo ${RCLCPP_VERSION} \
    && git clone https://github.com/ros2-gbp/rclcpp-release.git && cd rclcpp-release && git checkout ${RCLCPP_VERSION} \
    && patch -i /tmp/rclcpp-disable-tests.patch \
    && unset RCLCPP_VERSION \
    && git config user.email "builder@nvidia.com" && git config user.name "NVIDIA Builder" \
    && git remote add rclcpp https://github.com/ros2/rclcpp.git && git fetch rclcpp \
    && git cherry-pick 232262c02a1265830c7785b7547bd51e1124fcd8 \
    && source ${ROS_ROOT}/setup.bash \
    && cd ../ && rosdep install -i -r --from-paths rclcpp-release/ --rosdistro humble -y \
    && cd rclcpp-release && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb \
    && echo "rclcpp https://github.com/ros2/rclcpp/commit/232262c02a1265830c7785b7547bd51e1124fcd8" >> ${ROS_ROOT}/VERSION \
    && cd ../ && rm -Rf src build log

# Install Moveit 2 ROS packages
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-control-msgs \
    ros-humble-controller-manager \
    ros-humble-geometric-shapes \
    ros-humble-gripper-controllers \
    ros-humble-interactive-markers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-joy \
    ros-humble-launch-param-builder \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-pluginlib \
    ros-humble-py-binding-tools \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-rviz-visual-tools \
    ros-humble-rviz2 \
    ros-humble-srdfdom \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-topic-based-ros2-control \
    ros-humble-ur \
    ros-humble-ur-bringup \
    ros-humble-ur-calibration \
    ros-humble-ur-client-library \
    ros-humble-ur-controllers \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-xacro

# Install various moveit_resources packages from source.
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit_resources.git -b humble \
    && cd moveit_resources && source ${ROS_ROOT}/setup.bash \
    && cd fanuc_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd fanuc_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd panda_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd panda_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd pr2_description && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb \
    && cd moveit_resources && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd .. && apt-get install -y ./*.deb && rm *.deb

# Install MoveIt task constructor from source.  The "demo" package depends on moveit_resources_panda_moveit_config,
# installed from source above.
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit_task_constructor.git -b humble \
    && cd moveit_task_constructor && source ${ROS_ROOT}/setup.bash \
    && cd msgs && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd rviz_marker_tools && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd core && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd capabilities && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd visualization && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd demo && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# MoveIt 2's hybrid planning package depends on moveit_resources_panda_moveit_config, installed from source above.
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-moveit-hybrid-planning

# Install moveit2_tutorials from source (depends on moveit_hybrid_planning).
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble \
    && cd moveit2_tutorials && source ${ROS_ROOT}/setup.bash \
    && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install paho-mqtt for isaac_ros_mission_client
RUN python3 -m pip install -U \
        paho-mqtt==1.6.1

# Install cuda-python for isaac_ros_pynitros
RUN python3 -m pip install \
        cuda-python
# Install fake cuda-python Debian package to satisfy apt install check
COPY rosdep/ros-humble-cuda-python-placeholder /tmp/ros-humble-cuda-python-placeholder
RUN --mount=type=cache,target=/var/cache/apt \
    cd /tmp && source ${ROS_ROOT}/setup.bash \
    && dpkg-deb --nocheck --build ros-humble-cuda-python-placeholder && apt-get install -y ./ros-humble-cuda-python-placeholder.deb \
    && rm -f ./ros-humble-cuda-python-placeholder.deb

# Patch gtest to make it work with CXX 17
RUN sudo sed -i '917i #ifdef GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '920i #endif' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '2392i #if defined(GTEST_INTERNAL_CPLUSPLUS_LANG) && \\\n    GTEST_INTERNAL_CPLUSPLUS_LANG < 201703L\n#define GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL 1\n#endif' \
    /usr/src/googletest/googletest/include/gtest/internal/gtest-port.h

# Install MCAP CLI
ARG TARGETPLATFORM
ARG MCAP_VERSION=v0.0.51
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
        wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-amd64 && \
        chmod +x mcap-linux-amd64 && \
        mv mcap-linux-amd64 /opt/ros/humble/bin/mcap; \
    elif [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
        wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-arm64 && \
        chmod +x mcap-linux-arm64 && \
        mv mcap-linux-arm64 /opt/ros/humble/bin/mcap; \
    else \
        echo "Unknown architecture, can't install MCAP CLI" && \
        exit -1; \
    fi

# Install custom vcstool with --delay flag to be robust against
# GitHub rate-limiting (nvbugs/4872446)
RUN mkdir -p /opt/ros/humble && cd /opt/ros/humble \
    && git clone https://github.com/andrewbest-tri/vcstool.git -b andrewbest/delay \
    && echo 'source /opt/ros/humble/vcstool/setup.sh' | tee --append /etc/bash.bashrc

# Make sure that the workspace is always sourced
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" | sudo tee --append /etc/bash.bashrc

# Colcon auto complete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" | sudo tee --append /etc/bash.bashrc

# Store list of packages (must be last)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-end-packages.csv


RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} rosidl_runtime_c rcutils rcl rmw tf2 tf2_msgs geometry_msgs nav_msgs std_msgs rosgraph_msgs sensor_msgs vision_msgs rclpy ros2topic ros2pkg ros2doctor ros2run ros2node ros_environment ackermann_msgs example_interfaces > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall


RUN apt  update -y && apt  upgrade -y  && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential  \
    && apt install -y python3-colcon-common-extensions

RUN source /opt/ros/humble/setup.bash 
#RUN  git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git:opt

COPY /humble_ws /opt/humble_ws

WORKDIR /opt/humble_ws

RUN rosdep install -i --from-path src --ignore-src  --rosdistro humble  -y 
   
RUN  source /opt/ros/humble/setup.bash \
    && colcon build \
    && source install/local_setup.bash 

# ZCAM

ARG UBUNTU_MAJOR=22
ARG UBUNTU_MINOR=04
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=6
ARG CUDA_PATCH=3
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=3

WORKDIR /opt/humble_ws

COPY ZED_SDK_Ubuntu.run /opt/humble_ws/

RUN   chmod +x ZED_SDK_Ubuntu.run\
    && ./ZED_SDK_Ubuntu.run -- silent

WORKDIR /opt/humble_ws/src
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git

WORKDIR /opt/humble_ws
RUN apt install ros-humble-zed-msgs -y\
    &&   source /opt/ros/humble/setup.bash \
    && rosdep update -y \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -r install \
    && rm -r build \
    && rm -r log \
    && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    && echo source $(pwd)/install/local_setup.bash >> ~/.bashrc \
    && source ~/.bashrc 


