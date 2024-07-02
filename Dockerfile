# Copy built binaries into final image
FROM ubuntu:jammy-20240530

ARG L4T_RELEASE_MAJOR=32.7
ARG L4T_RELEASE_MINOR=1
ARG CUDA=10.2
ARG DEBIAN_FRONTEND=noninteractive
ARG SOC="t194"


RUN apt-get update && apt-get install -y tmux && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y vim && rm -rf /var/lib/apt/lists/*


RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils software-properties-common && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN echo $L4T_RELEASE_MAJOR
ADD --chown=root:root https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
    echo "deb https://repo.download.nvidia.com/jetson/common r$L4T_RELEASE_MAJOR main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb https://repo.download.nvidia.com/jetson/${SOC} r$L4T_RELEASE_MAJOR main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean


#
# Update environment
#
ENV PATH /usr/local/cuda-$CUDA/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda-$CUDA/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH=/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV OPENBLAS_CORETYPE=ARMV8

RUN apt-get update && apt-get install -y build-essential libyamlcpp-dev
WORKDIR /root
CMD ["bash"]

# install ROS
# set locale from 
RUN << EOF
    apt update && apt-get install locales
    locale-gen en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    echo "LANG=en_US.UTF-8" >> ~/.bashrc
    locale
EOF

# install 
RUN apt install software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt upgrade  && \
    apt install ros-humble-desktop && \ 
    apt install ros-humble-ros-base && \
    apt install ros-dev-tools && \
    source /opt/ros/humble/setup.bash




