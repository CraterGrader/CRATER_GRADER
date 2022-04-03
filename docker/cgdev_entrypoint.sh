#!/bin/zsh
set -e

# -------- Final container setup --------------------------
# This file runs every time the container is brought up

# Start the VNC server, in shared mode so more than one user can access, must be last command to run
x11vnc -forever -usepw -create -shared
# ---------------------------------------------------------

exec "$@"
