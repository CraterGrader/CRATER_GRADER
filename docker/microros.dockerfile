# Initialize base image with MicroRos
FROM microros/base:galactic as base

# MicroRos setup
WORKDIR /uros_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  &&  . install/local_setup.sh \
  &&  ros2 run micro_ros_setup create_agent_ws.sh \
  &&  ros2 run micro_ros_setup build_agent.sh \
  && rm -rf /root/CRATER_GRADER/cg_ws/firmware/ 

# Clean up microRos setup
WORKDIR /root/CRATER_GRADER/cg_ws/src/
RUN rm -rf colcon.meta eProsima ros2/ ros2.repos uros

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

# # Multi-Stage with Conda image
# FROM continuumio/miniconda3 as condaenv
# COPY --from=base / /

# # Setup conda environment
# COPY environment.yml /root/
# ENV PATH=/root/miniconda3/bin:${PATH}
# RUN conda init zsh &&\
#   conda env create --name cg -f /root/environment.yml --force &&\
#   rm -f /root/environment.yml

# Set working directory
WORKDIR /root/CRATER_GRADER/cg_ws/

# Set ROS environment variable for building Docker
ARG ROSDISTRO=galactic
ENV ROSDISTRO=$ROSDISTRO

# Install any other system packages, including for ROS
RUN apt-get install -y\
  figlet\
  libgl1-mesa-glx\
  vim\
  tmux\
  iputils-ping\
  tree\
  ros-$ROSDISTRO-rviz2\
  ros-$ROSDISTRO-plotjuggler-ros\
  ros-$ROSDISTRO-joy

# Avoid user input prompts, use default answers 
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq install openssh-client



# Make entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["zsh"]