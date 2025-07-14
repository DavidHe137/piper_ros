FROM ros:humble-ros-base

RUN apt update

# Hardware
RUN apt install -y python3 python3-dev python3-pip net-tools can-utils ethtool iproute2
RUN pip3 install python-can piper_sdk


# Simulator
RUN apt update
RUN apt install -y gazebo \
                   ros-humble-gazebo-ros-pkgs \
                   ros-humble-gazebo-ros2-control \
                   ros-humble-ros2-control \
                   ros-humble-ros2-controllers \
                   ros-humble-xacro \
                   ros-humble-joint-state-publisher-gui \
                   ros-humble-rviz2
RUN pip3 install scipy

ADD https://api.github.com/repos/skim743/piper_ros/git/refs/heads/humble version.json
RUN git clone -b humble https://github.com/skim743/piper_ros.git

RUN cd piper_ros && \
    source /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

ENTRYPOINT ["/piper_ros/entrypoint.sh"]

# # Get python + deps.  Most of these are actually for netifaces install
# RUN apt-get -y install git python3 python3-dev python3-pip gcc g++ cmake musl-dev net-tools netbase nano

# # Get ssh and sshpass for shutting down the Raspberry Pi within a Docker container
# RUN apt-get -y install openssh-client sshpass

# # Install firmware
# RUN cd /home && \
#     git clone https://github.com/skim743/GTernal_ROS.git && \
#     cd GTernal_ROS/ && \
#     . /opt/ros/humble/setup.sh && \
#     rosdep install -i --from-path src --rosdistro humble -y && \
#     colcon build --packages-select firmware_interfaces && \
#     colcon build --packages-select firmware

# # Install VRPN for Vicon Tracker
# RUN rm -rf /var/lib/apt/lists/*

# ADD https://api.github.com/repos/pglotfel/vrpn/git/refs/heads/master version.json
# RUN cd /home && \
# git clone https://github.com/pglotfel/vrpn.git && \
# cd vrpn && mkdir build && cd build && \
# cmake .. && make -j $(($(nproc)-1)) && make install

# # VRPN cmake puts this in the wrong place for some reason
# RUN cp /usr/local/lib/python3.1/dist-packages/vrpn.so /usr/lib/python3.10/

# RUN python3 -m pip uninstall -y paho-mqtt
# RUN python3 -m pip install paho-mqtt==1.3.1

# RUN cd /home/GTernal_ROS/ && \
#     . /opt/ros/humble/setup.sh && \
#     colcon build --packages-select vicon_tracker