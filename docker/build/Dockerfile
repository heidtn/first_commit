FROM ubuntu:16.04

MAINTAINER Kevin Greene <kgreenek@gmail.com>

ARG USER
ARG USERID

RUN apt-get update && \
    apt-get install -y git mercurial build-essential autoconf automake libtool pkg-config unzip \
        cmake ninja-build curl ruby-dev bison autopoint gettext && \
    apt-get install -y vim net-tools && \
    apt-get clean

RUN echo "creating new ${USER} with UID ${USERID}" && \
    # Add user to tty group because the linux-util install step requires the ability to run
    # "chgrp tty ..."
    useradd -m ${USER} -u ${USERID} -G tty && \
    adduser ${USER} sudo && \
    mkdir -p /home/${USER} && \
    chown -R ${USER} /home/${USER}

ENTRYPOINT \
    echo "Building external projects..." && \
    mkdir -p /home/${USER}/first_commit/out/external/src && \
    mkdir -p /home/${USER}/first_commit/out/external/build && \
    cd /home/${USER}/first_commit/out/external/build && \
    cmake /home/${USER}/first_commit/external -G Ninja && \
    ninja && \
    echo "Building fcmt..." && \
    mkdir -p /home/${USER}/first_commit/out/build && \
    cd /home/${USER}/first_commit/out/build && \
    cmake /home/${USER}/first_commit -G Ninja && \
    ninja install

#ENTRYPOINT \
#    export LD_LIBRARY_PATH="/home/${USER}/out/lib":"${LD_LIBRARY_PATH}" && \
#    cd /home/${USER}/build && \
#    cmake "/home/${USER}/first_commit" -G Ninja \
#        -DFIRST_COMMIT_OUT_DIR="/home/${USER}/out" \
#        -DFIRST_COMMIT_EXTERNAL_DIR="/home/${USER}/external" && \
#    ninja
# TODO(kgreenek): Add tests and targets to install.
#    ninja test && \
#    ninja install
