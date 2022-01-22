# Initialize base image
FROM ros:galactic
ARG ROSDISTRO=galactic
ENV ROSDISTRO=$ROSDISTRO

# Set working directory
WORKDIR /root/CRATER_GRADER

# Entrypoint
COPY docker/entrypoint.sh /

# Vim theme
COPY docker/.vim /root/.vim
COPY docker/.vimrc /root/.vimrc

# Zsh theme, and make entrypoint executable
COPY docker/.p10k.zsh /root/
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget &&\
  PATH="$PATH:/usr/bin/zsh" &&\
  chmod +x /entrypoint.sh

# Default zsh powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)"

# Initialize zsh theme
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc

# Setup conda environment
COPY environment.yml /root/
ENV PATH=/root/miniconda3/bin:${PATH}
RUN wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh &&\
  zsh Miniconda3-latest-Linux-x86_64.sh -b -u &&\
  rm -f Miniconda3-latest-Linux-x86_64.sh &&\
  conda init zsh &&\
  conda env create --name cg -f /root/environment.yml --force &&\
  rm -f /root/environment.yml

# Install any other system packages
RUN apt-get install -y\
  libgl1-mesa-glx\
  vim\
  tmux\
  figlet\
  iputils-ping\
  ssh-client

# Make entry
ENTRYPOINT ["/entrypoint.sh"]
CMD ["zsh"]