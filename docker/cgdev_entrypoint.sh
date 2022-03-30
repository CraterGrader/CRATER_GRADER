#!/bin/zsh
set -e

# -------- ROS and MicroROS -------------------------------
# Source ROS underlay
echo 'source /opt/ros/$ROS_DISTRO/setup.zsh' >> /root/.zshrc
echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc

# Source MicroROS overlay
echo 'source /uros_ws/install/local_setup.zsh' >> /root/.zshrc
echo 'source /uros_ws/install/local_setup.bash' >> /root/.bashrc

# Export MicroROS DDS settings
if [ "$ROS_LOCALHOST_ONLY" = "1" ] ; then
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm_localhost_only.xml' >> /root/.zshrc
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm_localhost_only.xml' >> /root/.bashrc
else
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml' >> /root/.zshrc
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml' >> /root/.bashrc
fi
# ---------------------------------------------------------

# -------- Setup Shell Environment ------------------------
# Fix zsh autocomplete for ROS2 packages
echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Try sourcing cg_ws packages
echo 'source /root/CRATER_GRADER/cg_ws/install/setup.zsh > /dev/null 2>&1' >> ~/.zshrc
echo 'source /root/CRATER_GRADER/cg_ws/install/setup.bash > /dev/null 2>&1' >> ~/.bashrc

# Start new shells in the cg conda environment
echo "conda activate cg" >> ~/.zshrc
echo "conda activate cg" >> ~/.bashrc
# ---------------------------------------------------------

# -------- Last Setup -------------------------------------
# Welcome message
echo "figlet -f slant 'CraterGrader'" >> ~/.zshrc

# Start the VNC server, in shared mode so more than one user can access, must be last command to run
x11vnc -forever -usepw -create -shared
# ---------------------------------------------------------

exec "$@"
