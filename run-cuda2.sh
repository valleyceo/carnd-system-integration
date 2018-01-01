#! /bin/bash
# Note: use this if you have already created capstone-gpu in another repository
# Create capstone-gpu# image: nvidia-docker build . -f Dockerfile.gpu -t capstone-gpu#
set -e

# Settings from environment
UDACITY_SOURCE=${UDACITY_SOURCE:-`pwd`}
UDACITY_IMAGE=${UDACITY_IMAGE:-capstone-gpu2}
CONTAINER_NAME="capstone2"

if [ "$(docker ps -a | grep ${CONTAINER_NAME})" ]; then
  echo "Attaching to running container..."
  nvidia-docker exec -it ${CONTAINER_NAME} bash $@
else
  nvidia-docker run --name ${CONTAINER_NAME} --rm -it -p 4567:4567 -v "${UDACITY_SOURCE}:/udacity" ${UDACITY_IMAGE} $@
fi
