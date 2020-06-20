FROM osrf/ros:melodic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG TERM=xterm

RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    build-essential \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    # python-vcstool \    
    wget \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Additional ROS dependencies
# in rosdeps:
#   cgal
#   ocl-icd-opencl-dev
#   opencl-headers
# 
# not in rosdeps
#   fftw3
#   libclfft-dev
#   libfftw3-dev
# 
RUN apt-get update && apt-get install -y --no-install-recommends \	
    fftw3 \
    ros-melodic-hector-gazebo-plugins \
    libcgal-dev \
    libclfft-dev \
    libfftw3-dev \
    ocl-icd-opencl-dev \
    opencl-headers \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN pip install --upgrade \
    catkin_tools \
    wheel

# Use bash
SHELL ["/bin/bash", "-c"]

# Create a catkin workspace
RUN mkdir -p catkin_ws/src

# Copy packages into the workspace
WORKDIR /catkin_ws
COPY /src/asv_sim src/
COPY /src/asv_wave_sim src/
COPY /src/rs750 src/

# Configure and build
RUN source /opt/ros/melodic/setup.bash \
    catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && catkin build

# Define entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
