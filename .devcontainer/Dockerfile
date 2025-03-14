# Dockerfile
# Accept BASE_IMAGE as a build argument with a default value
ARG BASE_IMAGE=osrf/ros:iron-desktop
FROM ${BASE_IMAGE}

# Set environment variable to prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND                 noninteractive
ENV QT_X11_NO_MITSHM                1
ENV TERM                            xterm-256color

# Install APT packages including Python and pip
RUN apt-get update && apt-get install -y \
    # Ubuntu essentials    
    apt-utils \
    wget \
    unzip \
    git-all \
    # Python essentials
#     python3 \
#     python3-pip \
#     python3-venv \
#     python3-tk \
    # Libraries for X11 visualization
    libx11-dev \
    libxrender1 \
    libxext6 \
    libfontconfig1 \
    # Other dependencies
    libglib2.0-0 \
    # Agilex dependencies
    libasio-dev \
    kmod \
    iproute2 \
    can-utils \
    net-tools \
    ros-iron-xacro \
    # Nav2 dependencies
    ros-iron-navigation2 \
    ros-iron-nav2-bringup \
    ros-iron-mapviz \
    ros-iron-mapviz-plugins \
    ros-iron-tile-map \
    # Remove cached data
    && rm -rf /var/lib/apt/lists/*

# # Upgrade pip to the latest version (optional but recommended)
# # RUN pip3 install --upgrade pip --break-system-packages

# # Install Python packages
# RUN pip3 install --no-cache-dir --break-system-packages \
#     scikit-learn \
#     opencv-python \
#     matplotlib \
#     pandas \
#     ipython \
#     tqdm

# Add Build Arguments for UID and GID
ARG USER_NAME=ubuntu
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create a group with the same GID as the host
RUN groupadd --gid ${GROUP_ID} ${USER_NAME} || echo "Group ${USER_NAME} already exists"

# Create a user with the specified UID and GID
RUN useradd --uid ${USER_ID} --gid ${GROUP_ID} --shell /bin/bash --create-home ${USER_NAME}

# **Add passwordless sudo for devuser**
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} && \
    chmod 0440 /etc/sudoers.d/${USER_NAME}

# Switch to the non-root user
USER ${USER_NAME}

# Make sure new shells source ROS setup
RUN echo "source /opt/ros/iron/setup.bash" >> /home/${USER_NAME}/.bashrc

# Add /home/${USER_NAME}/.local/bin to PATH
ENV PATH=/home/${USER_NAME}/.local/bin:${PATH}

# Set DISPLAY variable
ENV DISPLAY=:0