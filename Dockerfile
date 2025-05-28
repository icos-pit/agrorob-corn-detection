FROM ros:humble-ros-base

# Make bash the default shell for RUN
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Create workspace and install build tools
WORKDIR /ros2_ws/src
RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pip python3-numpy python3-opencv libboost-python-dev \
        ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-tf2 \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Clone deps and copy your package
RUN git clone https://github.com/icos-pit/corn_detection_msgs.git
COPY . /ros2_ws/src/corn_yolo_ros_interface

# Build once, after everything is in place
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Install Python dependencies
WORKDIR /ros2_ws/src/corn_yolo_ros_interface
RUN pip3 install -r requirements.txt 

WORKDIR /ros2_ws
# Source the overlay when a shell starts
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash", "-c", \
     "source /ros2_ws/install/setup.bash && \
      ros2 run corn_yolo_ros_interface detection.py \
      --ros-args -p model_path:=/ros2_ws/src/corn_yolo_ros_interface/model/best.pt"]

