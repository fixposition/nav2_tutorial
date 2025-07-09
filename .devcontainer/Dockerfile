# Dockerfile
ARG BASE_IMAGE=ros:jazzy-perception
FROM ${BASE_IMAGE}

# Build arguments for the new user.
ARG USER_NAME=dev
ARG USER_ID=1000
ARG GROUP_ID=1000

# Remove the default "ubuntu" user so that UID 1000 is free.
RUN userdel -r ubuntu || true

# Set environment variables to avoid interactive prompts and to configure terminal behavior.
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV TERM=xterm-256color

# Install common APT packages, then conditionally install Mapviz on x86.
RUN set -e \
 && apt update -o Acquire::AllowInsecureRepositories=true || true \
 && apt install -y --no-install-recommends curl gnupg ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor --yes --batch --no-tty \
            --output /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
 && cp /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
       /usr/share/keyrings/ros-archive-keyring.gpg \
 && apt update && \
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
      ros-jazzy-xacro \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-launch-xml \
      python3-venv \
      python3-pip \
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
WORKDIR /home/${USER_NAME}

# Create virtual environment for Python packages
RUN python3 -m venv nav2_env && \
    /bin/bash -c "source nav2_env/bin/activate && pip install --upgrade pip && pip install 'setuptools<69' 'pip<24'"

# Install Python packages from requirements.txt
COPY requirements.txt ./
RUN /bin/bash -c "source nav2_env/bin/activate && \
                  pip install -r requirements.txt"

# Make interactive shells auto-source ROS and ROS workspace
RUN echo 'source /opt/ros/jazzy/setup.bash'    >> ~/.bashrc \
 && echo 'source ~/ros_ws/install/setup.bash'  >> ~/.bashrc

# Ensure the user's local bin is in PATH.
ENV PATH=/home/${USER_NAME}/.local/bin:${PATH}

# Expose venv to every subsequent shell
ENV VIRTUAL_ENV=/home/${USER_NAME}/nav2_env
ENV PATH=/home/${USER_NAME}/nav2_env/bin:$PATH
ENV PYTHONPATH=/home/dev/nav2_env/lib/python3.12/site-packages:$PYTHONPATH

# Set DISPLAY for X11 forwarding.
ENV DISPLAY=:0
