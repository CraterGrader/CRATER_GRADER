# -------- Build on existing docker images ----------------
# Get conda files
FROM continuumio/miniconda3:4.10.3p1 as conda_setup

# Use microros as the base image, to avoid rebuilding from source every time
FROM microros/micro-ros-agent:galactic as ros_base

# Copy over conda into ros_base image
ENV PATH=/root/miniconda3/bin:/opt/conda/bin:${PATH}
COPY --from=conda_setup /opt/conda/ /opt/conda/

### Automatically setup and build MicroROS packages from source, no longer needed but kept in case microros docker image fails, probably should be run with ros:$ROS_DISTRO image instead of microros/micro-ros-agent:$ROS_DISTRO
## WORKDIR /root/microros_ws_autobuild
## RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
##   && apt-get update && rosdep update \
##   && rosdep install --from-path src --ignore-src -y \
##   && . /opt/ros/$ROS_DISTRO/setup.sh \
##   && colcon build \
##   && . install/local_setup.sh \
##   && ros2 run micro_ros_setup create_firmware_ws.sh host \
##   && ros2 run micro_ros_setup build_firmware.sh \
##   && . install/local_setup.sh \
##   && ros2 run micro_ros_setup create_agent_ws.sh \
##   && ros2 run micro_ros_setup build_agent.sh \
##   && . install/local_setup.sh
# ---------------------------------------------------------

# -------- Base environment configuration ----------------------
# Setup vim theme
COPY docker/.vim/ /root/.vim/
COPY docker/.vimrc /root/.vimrc

# Setup shell
COPY docker/.p10k.zsh /root/.p10k.zsh
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget \
  && PATH="$PATH:/usr/bin/zsh" \
  # Install Oh-My-Zsh with default theme
  && sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)" \
  # Initialize custom zsh theme
  && echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc \
  # Conda
  && conda init zsh \
  && conda init bash \
  # Source ROS underlay
  && echo '# ROS' >> /root/.zshrc \
  && echo '# ROS' >> /root/.bashrc \
  && echo 'source /opt/ros/$ROS_DISTRO/setup.zsh' >> /root/.zshrc \
  && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc \
  # Source MicroROS overlay
  && echo 'source /uros_ws/install/setup.zsh' >> /root/.zshrc \
  && echo 'source /uros_ws/install/setup.bash' >> /root/.bashrc \
  # Export MicroROS DDS settings, assumes ROS_LOCALHOST_ONLY != 1
  && echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml' >> /root/.zshrc \
  && echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml' >> /root/.bashrc \
  # Try sourcing cg_ws packages
  && echo 'source /root/CRATER_GRADER/cg_ws/install/setup.zsh > /dev/null 2>&1' >> ~/.zshrc \
  && echo 'source /root/CRATER_GRADER/cg_ws/install/setup.bash > /dev/null 2>&1' >> ~/.bashrc \
  # Fix zsh autocomplete for ROS2 packages
  && echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc \
  && echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc \
  # Start new shells in the cg conda environment
  && echo '# Conda environment' >> /root/.zshrc \
  && echo '# Conda environment' >> /root/.bashrc \
  && echo "conda activate cg" >> ~/.zshrc \
  && echo "conda activate cg" >> ~/.bashrc \
  # Welcome message
  && echo '# Welcome message' >> /root/.zshrc \
  && echo "figlet -f slant 'CraterGrader'" >> ~/.zshrc
# ---------------------------------------------------------

# -------- Base system packages ----------------------
# Install "starter pack" of some basic tools
RUN apt-get update && apt-get install -y \
  figlet \
  libgl1-mesa-glx \
  vim \
  tmux \
  iputils-ping \
  tree \
  # Install vnc, xvfb for VNC configuration, fluxbox for VNC window managment
  x11vnc \
  xvfb \
  fluxbox
# ---------------------------------------------------------

# -------- VNC GUI Configuration --------------------------
# Setup a VNC password
RUN  mkdir ~/.vnc \
  && x11vnc -storepasswd cratergrader ~/.vnc/passwd \
  # Start the VNC server
  && echo '# VNC setup' >> /root/.zshrc \
  && echo '# VNC setup' >> /root/.bashrc \
  && echo "export DISPLAY=:20" >> ~/.zshrc \
  && echo "export DISPLAY=:20" >> ~/.bashrc \
  # Always try to start windows management in background to be ready for VNC
  && echo "( fluxbox > /dev/null 2>&1 & )" >> ~/.zshrc \
  && echo "( fluxbox > /dev/null 2>&1 & )" >> ~/.bashrc \
  # Clean up unnecessary output files
  && echo "rm -f /root/CRATER_GRADER/cg_ws/nohup.out" >> ~/.zshrc \
  && echo "rm -f /root/CRATER_GRADER/cg_ws/nohup.out" >> ~/.bashrc
# ---------------------------------------------------------

# -------- Setup CraterGrader environment packages --------
# Setup conda environment
COPY environment.yml /root/
RUN conda env create --name cg -f /root/environment.yml --force \
    && rm -f /root/environment.yml

# Apt packages
# Run the following with DEBIAN_FRONTEND=noninteractive to avoid prompt for keyboard language, see https://askubuntu.com/questions/876240/how-to-automate-setting-up-of-keyboard-configuration-package
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  # General ROS2 debug
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-rqt-graph \
  ros-$ROS_DISTRO-rqt-reconfigure \
  ros-$ROS_DISTRO-plotjuggler-ros \
  # Teleop
  ros-$ROS_DISTRO-joy \
  # Realsense
  ros-$ROS_DISTRO-realsense2-camera \
  # robot_localization package ekf/ukf
  ros-$ROS_DISTRO-robot-localization \
  # PCL
  keyboard-configuration \
  libpcl-dev \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-msgs \
  # SunCam
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-apriltag \
  # Health monitoring gui
  ros-$ROS_DISTRO-rosbridge-suite \
  # Yaml CPP for STag
  libyaml-cpp-dev


# Prep for colcon build, but don't build anything yet
WORKDIR /root/CRATER_GRADER/cg_ws
COPY cg_ws/src/ /root/CRATER_GRADER/cg_ws/src/
RUN rosdep install --from-paths src --ignore-src -r -y
# ---------------------------------------------------------

# -------- Container entrypoint ---------------------------
# Setup entrypoint
COPY docker/cgdev_entrypoint.sh /
RUN chmod +x /cgdev_entrypoint.sh

# Make entry
WORKDIR /root/CRATER_GRADER/cg_ws/
ENTRYPOINT ["/cgdev_entrypoint.sh"]
CMD ["zsh"]
# ---------------------------------------------------------
