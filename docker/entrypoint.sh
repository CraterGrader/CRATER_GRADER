#!/bin/zsh
set -e

# Source ros with every new shell
echo 'source /opt/ros/galactic/setup.zsh' >> /root/.zshrc

# Start new shells in the conda environment
echo "conda activate cg" >> ~/.zshrc

# Welcome message
echo "figlet -f slant 'CraterGrader'" >> ~/.zshrc

exec "$@"