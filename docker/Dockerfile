FROM nvidia/cuda:12.9.1-base-ubuntu22.04 AS cuda_base

FROM ros:humble-ros-base

COPY --from=cuda_base /usr/local/cuda /usr/local/cuda
COPY --from=cuda_base /usr/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu

RUN apt update

## Hardware
# Piper
RUN apt install -y python3 python3-dev python3-pip net-tools can-utils ethtool iproute2
RUN pip3 install python-can piper_sdk

# RealSense
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    && tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update
RUN apt-get install -y --no-install-recommends librealsense2-utils librealsense2-udev-rules
RUN pip3 install pyrealsense2 numpy opencv-python



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

ADD https://api.github.com/repos/skim743/piper_sdk/git/refs/heads/master version.json
RUN git clone -b master https://github.com/skim743/piper_sdk.git

RUN cd /piper_ros && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# This is needed to run the gazebo simulator for some reason
RUN chmod +x /piper_ros/install/piper_gazebo/lib/piper_gazebo/joint8_ctrl.py

RUN echo "source /piper_ros/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/piper_ros/entrypoint.sh"]
CMD ["bash"]

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