FROM dustynv/ros:humble-desktop-l4t-r36.2.0

# Create a non-root user
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

RUN wget https://nvidia.box.com/shared/static/iizg3ggrtdkqawkmebbfixo7sce6j365.whl -O onnxruntime_gpu-1.16.0-cp38-cp38-linux_aarch64.whl \
    && pip3 install onnxruntime_gpu-1.16.0-cp38-cp38-linux_aarch64.whl
# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY config/ros/entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
