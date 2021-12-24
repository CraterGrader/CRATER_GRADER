# Initialize base image
FROM ubuntu:focal

# Set working directory
WORKDIR /root/CRATER_GRADER

# Install zsh shell
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget vim tmux
RUN PATH="$PATH:/usr/bin/zsh"

# Default powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)"


CMD ["zsh"]
