# -------- Build on existing docker images ----------------
# Get conda files
FROM continuumio/miniconda3:latest as conda_setup

# Use microros as the base image, to avoid rebuilding from source every time
FROM microros/micro-ros-agent:galactic as ros_base
ENV PATH=/root/miniconda3/bin:/opt/conda/bin:${PATH}
# ---------------------------------------------------------

# -------- Environment configuration ----------------------
# Setup vim theme
COPY docker/.vim/ /root/.vim/
COPY docker/.vimrc /root/.vimrc

# Setup zsh theme
COPY docker/.p10k.zsh /root/.p10k.zsh
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget \
  && PATH="$PATH:/usr/bin/zsh" \
  # Install Oh-My-Zsh with default theme
  && sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)" \
  # Initialize custom zsh theme
  && echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc
# ---------------------------------------------------------

# Install system packages "starter pack"
RUN apt-get update && apt-get install -y \
  figlet \
  libgl1-mesa-glx \
  vim \
  tmux \
  iputils-ping \
  tree \
  # Install vnc, xvfb for VNC configuration, fluxbox for window managment
  x11vnc \
  xvfb \
  fluxbox
# ---------------------------------------------------------

# -------- VNC GUI Configuration --------------------------
# Setup a VNC password
RUN  mkdir ~/.vnc \
  && x11vnc -storepasswd cratergrader ~/.vnc/passwd \
  # Start the VNC server
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
COPY --from=conda_setup /opt/conda/ /opt/conda/
COPY environment.yml /root/
RUN conda init zsh && conda init bash \
  && conda env create --name cg -f /root/environment.yml --force \
  && rm -f /root/environment.yml
# ---------------------------------------------------------

# -------- Custom and transient packages ------------------

# Run the following with DEBIAN_FRONTEND=noninteractive to avoid prompt for keyboard language
# https://askubuntu.com/questions/876240/how-to-automate-setting-up-of-keyboard-configuration-package
# RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
# Install additional custom packages
# RUN apt-get update && apt-get install -y \
#   ros-$ROS_DISTRO-rviz2 \
#   ros-$ROS_DISTRO-plotjuggler-ros \
#   ros-$ROS_DISTRO-joy \
#   ros-$ROS_DISTRO-realsense2-camera \
#   ros-$ROS_DISTRO-robot-localization \
#   libpcl-dev \
#   ros-$ROS_DISTRO-pcl-conversions \
#   ros-$ROS_DISTRO-pcl-ros \
#   ros-$ROS_DISTRO-pcl-msgs \
#   ros-$ROS_DISTRO-rqt-graph \
#   ros-$ROS_DISTRO-rqt-reconfigure \
#   ros-$ROS_DISTRO-rqt-graph \
#   ros-$ROS_DISTRO-rqt-reconfigure

# Prep for colcon build, but don't build anything yet
WORKDIR /root/CRATER_GRADER/cg_ws
COPY cg_ws/src/ /root/CRATER_GRADER/cg_ws/src/
RUN rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1
# ---------------------------------------------------------

# -------- Container entrypoint ---------------------------
# Setup entrypoint
COPY docker/micro_entrypoint.sh /
RUN chmod +x /micro_entrypoint.sh

# Make entry
WORKDIR /root/CRATER_GRADER/cg_ws/
ENTRYPOINT ["/micro_entrypoint.sh"]
CMD ["zsh"]
# ---------------------------------------------------------
