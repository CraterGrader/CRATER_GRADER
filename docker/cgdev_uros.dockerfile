# Get conda files
FROM continuumio/miniconda3 as conda_setup

# Use ros as the base image
FROM ros:galactic as ros_base
ENV PATH=/root/miniconda3/bin:/opt/conda/bin:${PATH}

# Setup vim theme
COPY docker/.vim/ /root/.vim/
COPY docker/.vimrc /root/.vimrc

# Setup zsh theme
COPY docker/.p10k.zsh /root/.p10k.zsh
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget \
  && PATH="$PATH:/usr/bin/zsh"

# Install Oh-My-Zsh with default theme
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)"

# Initialize custom zsh theme
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc

# Automatically setup and build MicroROS packages
WORKDIR /root/microros_ws_autobuild
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
  && apt-get update && rosdep update \
  && rosdep install --from-path src --ignore-src -y \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build \
  && . install/local_setup.sh \
  && ros2 run micro_ros_setup create_firmware_ws.sh host \
  && ros2 run micro_ros_setup build_firmware.sh \
  && . install/local_setup.sh \
  && ros2 run micro_ros_setup create_agent_ws.sh \
  && ros2 run micro_ros_setup build_agent.sh \
  && . install/local_setup.sh

# Install system packages "starter pack"
RUN apt-get update && apt-get install -y \
  figlet \
  libgl1-mesa-glx \
  vim \
  tmux \
  iputils-ping \
  tree

# Setup conda environment
COPY --from=conda_setup /opt/conda/ /opt/conda/
COPY environment.yml /root/
RUN conda init zsh \
  && conda env create --name cg -f /root/environment.yml --force \
  && rm -f /root/environment.yml

# Automatically build cg_ws packages
WORKDIR /root/cg_ws_autobuild/
COPY cg_ws/src/ /root/cg_ws_autobuild/src/
RUN conda init bash \
  && . /root/.bashrc \
  && conda activate cg \
  && . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build

# ------- Add custom packages below -------

# Install additional custom packages
RUN apt-get update && apt-get install -y \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-plotjuggler-ros \
  ros-$ROS_DISTRO-joy

# Avoid user input prompts, use default answers 
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq install openssh-client
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq --no-upgrade install openssh-client

# Setup entrypoint
COPY docker/cgdev_uros_entrypoint.sh /
RUN chmod +x /cgdev_uros_entrypoint.sh

# Make entry
WORKDIR /root/CRATER_GRADER/cg_ws/
ENTRYPOINT ["/cgdev_uros_entrypoint.sh"]
CMD ["zsh"]
