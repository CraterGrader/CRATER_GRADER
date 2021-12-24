#!/bin/bash
set -e

echo 'source /opt/ros/foxy/setup.zsh' >> /root/.zshrc

exec "$@"