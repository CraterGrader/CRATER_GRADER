# Get conda files
FROM continuumio/miniconda3 as conda_setup

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

# Set working directory
WORKDIR /root/CRATER_GRADER/cg_ws/

# Set ROS environment variable for building Docker
ARG ROSDISTRO=galactic
ENV ROSDISTRO=$ROSDISTRO

# Setup conda environment
COPY --from=conda_setup /opt/conda/. /opt/conda/
COPY environment.yml /root/
ENV PATH=/root/miniconda3/bin:${PATH}
RUN conda init zsh &&\
  conda env create --name cg -f /root/environment.yml --force &&\
  rm -f /root/environment.yml

# Install any other system packages, including for ROS
# RUN apt-get install -y \
#   figlet \
#   libgl1-mesa-glx \
#   vim \
#   tmux \
#   iputils-ping \
#   tree \
#   ros-$ROSDISTRO-rviz2 \
#   ros-$ROSDISTRO-plotjuggler-ros \
#   ros-$ROSDISTRO-joy

# Avoid user input prompts, use default answers 
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq install openssh-client
# RUN DEBIAN_FRONTEND=noninteractive apt-get -yq --no-upgrade install openssh-client


# Make entrypoint
COPY docker/cgdev_entrypoint.sh /
RUN chmod +x /cgdev_entrypoint.sh

ENTRYPOINT ["/cgdev_entrypoint.sh"]
CMD ["zsh"]