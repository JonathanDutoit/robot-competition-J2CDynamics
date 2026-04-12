FROM arm64v8/ros:humble

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Install Dependencies for libcamera. Working as of https://github.com/raspberrypi/libcamera/commit/8cebd777cae428daf415998cc588fe60c6de0d66
RUN apt update && apt install -y git python3-pip git python3-jinja2 \
      libboost-dev \
      libgnutls28-dev openssl libtiff-dev pybind11-dev \
      meson cmake \
      python3-yaml python3-ply \
      libglib2.0-dev libgstreamer-plugins-base1.0-dev

# Clone and build raspberrypi's libcamera fork
RUN git clone https://github.com/raspberrypi/libcamera.git \
  && cd libcamera \
  && meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
  && ninja -C build install

# Install Picamera2 and its dependencies
RUN apt update && apt install -y \
      python3-pyprctl \
      python3-piexif \
      python3-kms++ \
      python3-prctl \
    && pip3 install picamera2 --break-system-packages

# Expose the libcamera Python bindings built from source
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH

# Copy your local package and build it
RUN mkdir -p /app/src
COPY ros2_ws/src/camera_stream /app/src/camera_stream

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
  && cd /app \
  && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
  && colcon build --event-handlers=console_direct+

COPY entrypoint.sh /app/

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]