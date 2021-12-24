# Initialize base image
FROM ros:foxy

# Set working directory
WORKDIR /root/CRATER_GRADER

# Install zsh shell
ENV TERM=xterm-256color
RUN apt-get update && apt-get install -y zsh bash wget vim tmux
RUN PATH="$PATH:/usr/bin/zsh"

# Default powerline10k theme, no plugins installed
RUN sh -c "$(wget -O- https://raw.githubusercontent.com/deluan/zsh-in-docker/master/zsh-in-docker.sh)"

# Setup zsh theme
COPY /docker/.p10k.zsh /root/
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> ~/.zshrc

# Setup vim theme
COPY /docker/.vim /root/.vim
COPY /docker/.vimrc /root/.vimrc

# Entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["zsh"]
