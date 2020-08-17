FROM osrf/ros:melodic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG TERM=xterm

RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    build-essential \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-common-dev \
    mesa-opencl-icd \
    mesa-utils \
    mesa-utils-extra \
    mesa-vulkan-drivers \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
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
    libcgal-dev \
    libclfft-dev \
    libfftw3-dev \
    ocl-icd-opencl-dev \
    opencl-headers \
    ros-melodic-hector-gazebo-plugins \
    ros-melodic-imu-tools \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN pip install --upgrade \
    wheel

RUN pip install --upgrade \
    catkin_tools

# Use bash
SHELL ["/bin/bash", "-c"]

#
# Wave Gazebo plugin and rs750 sailboat model
#
RUN mkdir -p /catkin_ws/src

# Clone packages into the workspace
WORKDIR /catkin_ws/src
RUN git clone https://github.com/srmainwaring/asv_wave_sim.git -b feature/ardupilot_sailboat_poc  \
  && git clone https://github.com/srmainwaring/asv_sim.git -b feature/wrsc-devel \
  && git clone https://github.com/srmainwaring/rs750.git -b feature/ardupilot_sailboat_poc

# Configure, build and cleanup
WORKDIR /catkin_ws
RUN source /opt/ros/melodic/setup.bash \
    && catkin init \
    && catkin clean -y \
    && catkin config \
        --extend /opt/ros/melodic \
        --install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && catkin build \
    && rm -rf .catkin_tools .vscode build devel logs src  

#
# ArduPilot Gazebo plugin (not a catkin package)
#
RUN mkdir -p /gazebo_plugins

# Clone packages into the workspace, build and install ardupilot_gazebo
WORKDIR /gazebo_plugins
RUN  git clone https://github.com/srmainwaring/ardupilot_gazebo.git -b feature/gazebo_sailboat_poc \
  && cd ardupilot_gazebo \
  && mkdir build && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make && make install && cd .. && rm -rf build

#
# ArduPilot - adapted from https://github.com/edrdo/ardupilot-sitl-docker
#
WORKDIR /ardupilot

RUN useradd -U -d /ardupilot ardupilot && \
    usermod -G users ardupilot

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

ENV USER=ardupilot
RUN cd / && git clone https://github.com/srmainwaring/ardupilot.git -b feature/gazebo_sailboat_poc

RUN echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot
RUN chmod 0440 /etc/sudoers.d/ardupilot

RUN chown -R ardupilot:ardupilot /ardupilot

USER ardupilot
RUN /ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN make sitl

# Prebuild rover for SITL
WORKDIR /ardupilot
RUN ./waf configure --board sitl &&  ./waf rover

ENV CCACHE_MAXSIZE=1G
ENV PATH /usr/lib/ccache:/ardupilot/Tools:${PATH}
ENV PATH /ardupilot/Tools/autotest:${PATH}
ENV PATH /ardupilot/.local/bin:${PATH}

# Define shell scripts used to start Gazebo and SITL
COPY ./gazebo_entrypoint.sh /gazebo_entrypoint.sh
COPY ./process1.sh /process1.sh
COPY ./process2.sh /process2.sh
COPY ./multi_process.sh /multi_process.sh
RUN sudo chmod +x /gazebo_entrypoint.sh \
    &&  sudo chmod +x /process1.sh \
    && sudo chmod +x /process2.sh \
    && sudo chmod +x /multi_process.sh

# Clear the inherited ROS entrypoint
ENTRYPOINT [""]
CMD ["bash"]
