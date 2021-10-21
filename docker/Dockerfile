FROM osrf/ros:foxy-desktop

RUN apt-get update && apt-get install -y \
      ros-foxy-ros2-control \
      ros-foxy-ros2-controllers \
      ros-foxy-joint-state-publisher-gui \
      ros-foxy-xacro \
      ros-foxy-moveit
      # Some source builds require a package.xml be downloaded via wget from an external location
      wget \
      vim && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
    
## Configuration the ROS 2 environment
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc 

## Install proxy tool
## Change the proxy setting so that it works when the network of the docker is set as "host"
RUN apt-get update && apt-get install -y proxychains4 && \
    sed '$d' /etc/proxychains4.conf && sed '$c socks5  127.0.0.1 1080' /etc/proxychains4.conf 

CMD ["bash"]
