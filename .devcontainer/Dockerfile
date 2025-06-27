# Dockerfile
ARG BASE_IMAGE=ros:jazzy-perception
FROM ${BASE_IMAGE}

# Remove the default "ubuntu" user so that UID 1000 is free.
RUN userdel -r ubuntu || true

# Set environment variables to avoid interactive prompts and to configure terminal behavior.
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV TERM=xterm-256color

# Install common APT packages, then conditionally install Mapviz on x86.
RUN apt update && \
    apt install -y \
      libasio-dev \
      apt-utils \
      nano \
      wget \
      unzip \
      git-all \
      libx11-dev \
      libxrender1 \
      libxext6 \
      libfontconfig1 \
      libglib2.0-0 \
      kmod \
      iproute2 \
      can-utils \
      net-tools \
      python3-tk \
      python3-pyproj \
      ros-jazzy-xacro \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-teleop-twist-keyboard \
      sudo && \
    \
    # Detect device architecture.
    arch="$(dpkg --print-architecture)" && \
    echo "Detected architecture: ${arch}" && \
    if [ "${arch}" = "amd64" ]; then \
      echo "Installing Mapviz packages on x86…" && \
      apt install -y \
        ros-jazzy-mapviz \
        ros-jazzy-mapviz-plugins \
        ros-jazzy-tile-map ; \
    else \
      echo "Skipping Mapviz packages on ${arch}" ; \
    fi && \
    \
    # Clean up apt cache.
    rm -rf /var/lib/apt/lists/*

# Build arguments for the new user.
ARG USER_NAME=dev
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create a new group and user with the specified UID/GID, and grant passwordless sudo.
RUN groupadd --gid ${GROUP_ID} ${USER_NAME} || echo "Group ${USER_NAME} already exists" && \
    useradd --uid ${USER_ID} --gid ${GROUP_ID} --shell /bin/bash --create-home ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} && \
    chmod 0440 /etc/sudoers.d/${USER_NAME}

# As root, add global shell configuration for color support to /etc/bash.bashrc.
USER root
RUN echo "alias ls='ls --color=auto'" >> /etc/bash.bashrc && \
    echo "export PS1='\[\e[0;32m\]\u@\h:\w\$ \[\e[m\]'" >> /etc/bash.bashrc

# Switch to the new "dev" user.
USER ${USER_NAME}

# Add the ROS environment setup to the dev user's .bashrc.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USER_NAME}/.bashrc

# Ensure the user's local bin is in PATH.
ENV PATH=/home/${USER_NAME}/.local/bin:${PATH}

# Set DISPLAY for X11 forwarding.
ENV DISPLAY=:0
