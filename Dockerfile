FROM ros:jazzy

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    can-utils \
    git \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install customtkinter

WORKDIR /workspace

CMD ["bash"]

CMD ["bash"]
