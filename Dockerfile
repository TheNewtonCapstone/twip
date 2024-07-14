FROM dustynv/ros:humble-desktop-l4t-r36.2.0
##
### Create a non-root user
ARG USERNAME=ros_runtime
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# installation for development tools 
RUN apt-get update && apt-get install -y \
    tmux \
    vim \ 
    && rm -rf /var/lib/apt/lists/*

ENV PIP_ROOT_USER_ACTION=ignore
RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*


RUN git clone --recursive https://github.com/microsoft/onnxruntime
RUN export CUDACXX="/usr/local/cuda/bin/nvcc"

RUN sudo apt install -y --no-install-recommends \
  build-essential software-properties-common libopenblas-dev \
  libpython3.10-dev python3-pip python3-dev python3-setuptools python3-wheel

RUN ./onnxruntime/build.sh --config Release --update --allow_running_as_root --build --parallel 2 --build_wheel \
--update --skip_submodule_sync --skip_tests --use_tensorrt --cuda_home /usr/local/cuda --cudnn_home /usr/lib/aarch64-linux-gnu \
--tensorrt_home /usr/lib/aarch64-linux-gnu --arm64





# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY config/dockerfiles/ros/entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

