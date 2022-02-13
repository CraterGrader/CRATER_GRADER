# Get conda files
FROM continuumio/miniconda3 as conda_setup

# # Use microRos as the base image
# FROM microros/base:galactic as base
# ENV PATH=/root/miniconda3/bin:/opt/conda/bin:${PATH}

# # MicroRos setup
# WORKDIR /uros_ws
# RUN source /opt/ros/$ROS_DISTRO/setup.sh \
#   && source install/local_setup.sh \
#   && ros2 run micro_ros_setup create_agent_ws.sh \
#   && ros2 run micro_ros_setup build_agent.sh \
#   && rm -rf /root/CRATER_GRADER/cg_ws/firmware/ 

# # Clean up microRos setup
# WORKDIR /root/CRATER_GRADER/cg_ws/src/
# RUN rm -rf colcon.meta eProsima ros2/ ros2.repos uros

# Use ros as the base image
FROM ros:galactic as base
ENV PATH=/root/miniconda3/bin:/opt/conda/bin:${PATH}



# Vim theme
COPY docker/.vim /root/.vim
COPY docker/.vimrc /root/.vimrc

# Zsh theme
COPY docker/.p10k.zsh /root/
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget\
  && PATH="$PATH:/usr/bin/zsh"

# Default zsh powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)"

# Initialize zsh theme
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc

# MicroROS setup
WORKDIR /root/CRATER_GRADER/microros_ws/
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


# Set working directory
WORKDIR /root/CRATER_GRADER/cg_ws/
COPY cg_ws/src/ ./src/
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build

# Set ROS environment variable for building Docker
ARG ROSDISTRO=galactic
ENV ROSDISTRO=$ROSDISTRO

# Setup conda environment
COPY --from=conda_setup /opt/conda/. /opt/conda/
# COPY environment.yml /root/
# RUN conda init zsh &&\
#   conda env create --name cg -f /root/environment.yml --force &&\
#   rm -f /root/environment.yml

# # Install system packages
# RUN apt-get update && apt-get install -y \
#   figlet \
#   libgl1-mesa-glx \
#   vim \
#   tmux \
#   iputils-ping \
#   tree

# # Additional custom packages
# RUN apt-get update && apt-get install -y \
#   ros-$ROSDISTRO-rviz2 \
#   ros-$ROSDISTRO-plotjuggler-ros \
#   ros-$ROSDISTRO-joy

# Avoid user input prompts, use default answers 
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq install openssh-client
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq --no-upgrade install openssh-client


# Make entrypoint
COPY docker/cgdev_uros_entrypoint.sh /
RUN chmod +x /cgdev_uros_entrypoint.sh

ENTRYPOINT ["/cgdev_uros_entrypoint.sh"]
CMD ["zsh"]