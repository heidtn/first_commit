#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
mkdir -p ${DIR}/out
docker run \
  --user ${USER} \
  -e USER=${USER} \
  -e USERID=${UID} \
  -v ${DIR}:/home/${USER}/first_commit \
  first_commit/build
