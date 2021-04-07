#! /usr/bin/env bash

# --rm			Automatically remove the container when it exits
# --interactive , -i	Keep STDIN open even if not attached
# --tty , -t		Allocate a pseudo-TTY
# --volume , -v		Bind mount a volume
# --publish , -p	Publish a container's port(s) to the host

HOST_A8_ROOT=${PWD}
DOCKER_APP_ROOT=/app/

docker run --rm -it -v ${HOST_A8_ROOT}:${DOCKER_APP_ROOT} -p 4567:4567 path_planning



