#!/bin/zsh
set -e

# # Source ros with every new shell
echo 'source /opt/ros/$ROSDISTRO/setup.zsh' >> /root/.zshrc
echo 'source /opt/ros/$ROSDISTRO/setup.bash' >> /root/.bashrc

# Source local packages
echo 'source /root/CRATER_GRADER/cg_ws/install/local_setup.zsh' >> /root/.zshrc
echo 'source /root/CRATER_GRADER/cg_ws/install/local_setup.bash' >> /root/.bashrc

# Source microRos
# echo 'source /uros_ws/install/local_setup.zsh' >> /root/.zshrc
# echo 'source /uros_ws/install/local_setup.bash' >> /root/.bashrc

# Start new shells in the conda environment
# echo "conda activate cg" >> ~/.zshrc

# Welcome message
echo "figlet -f slant 'CraterGrader'" >> ~/.zshrc

exec "$@"