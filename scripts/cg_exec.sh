#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "${SCRIPT_DIR}/../"

# Add device names here
DEVICES=(
  "/dev/ttyACM0"
  "/dev/something_else"
)

# docker-compose down

# Check for existence of devices
DOCKER_COMPOSE_FILE="${SCRIPT_DIR}/../docker-compose.yml"
cp "${DOCKER_COMPOSE_FILE}" "${SCRIPT_DIR}"
for device in "${DEVICES[@]}"; do
  if [ -e "${device}" ]; then
    echo "Found device: ${device}"
    sed -i "/devices/a \ \ \ \ \ \ - ${device}:${device}" "${DOCKER_COMPOSE_FILE}"
  else
    echo "Device not found: ${device}"
  fi
done

docker-compose up -d

cp "${SCRIPT_DIR}/docker-compose.yml" "${DOCKER_COMPOSE_FILE}"
rm -f "${SCRIPT_DIR}/docker-compose.yml"

docker-compose exec cg-dev zsh 
