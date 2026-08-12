FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-pip \
    portaudio19-dev \
    python3-tk \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    websockets \
    opencv-python \
    "numpy<2" \
    SpeechRecognition \
    pyaudio \
    faster-whisper

WORKDIR /root/ros2_ws
COPY src ./src

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
