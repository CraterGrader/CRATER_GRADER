#!/bin/zsh
set -e

# Udev rules
# /lib/systemd/systemd-udevd --daemon
# udevadm control --reload-rules
# udevadm trigger

# # Source ROS underlay binaries
echo 'source /opt/ros/$ROS_DISTRO/setup.zsh' >> /root/.zshrc
echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc

# Source microROS overlay packages
echo 'source /root/microros_ws_autobuild/install/local_setup.zsh' >> /root/.zshrc
echo 'source /root/microros_ws_autobuild/install/local_setup.bash' >> /root/.bashrc

# Fix zsh autocomplete for ROS2 packages
echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Source cg_ws overlay packages
echo 'source /root/cg_ws_autobuild/install/local_setup.zsh' >> /root/.zshrc
echo 'source /root/cg_ws_autobuild/install/local_setup.bash' >> /root/.bashrc

# Start new shells in the cg conda environment
echo "conda activate cg" >> ~/.zshrc
echo "conda activate cg" >> ~/.bashrc

# Welcome message
echo "figlet -f slant 'CraterGrader'" >> ~/.zshrc

# Start the VNC server, in shared mode so more than one user can access
x11vnc -forever -usepw -create -shared

exec "$@"