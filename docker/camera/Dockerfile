FROM arm64v8/ros:humble
SHELL ["/bin/bash", "-c"]
WORKDIR /app

# 1. Standardize build tools
RUN apt update && apt install -y git python3-pip python3-jinja2 \
      python3-dev python3-setuptools libboost-dev libgnutls28-dev openssl libtiff-dev \
      cmake ninja-build python3-yaml python3-ply \
      libglib2.0-dev libgstreamer-plugins-base1.0-dev \
      libfmt-dev libdrm-dev libexif-dev libboost-program-options-dev \
      libepoxy-dev libcap-dev pybind11-dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    gettext-base \
    && rm -rf /var/lib/apt/lists/*

# Pin pybind11 to 2.x 
RUN pip3 install meson "pybind11>=2.12,<3.0"

# 2. libcamera
WORKDIR /usr/local
RUN git clone https://github.com/raspberrypi/libcamera.git --branch v0.3.1+rpt20240906
WORKDIR /usr/local/libcamera
RUN meson setup build --buildtype=release \
      -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
      -Dv4l2=true -Dgstreamer=enabled -Dtest=false \
      -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled \
      -Ddocumentation=disabled -Dpycamera=enabled \
    && ninja -C build install \
    && ldconfig

# 3. kmsxx
WORKDIR /usr/local
RUN git clone https://github.com/tomba/kmsxx.git
WORKDIR /usr/local/kmsxx
RUN git reset --hard 6cf6e88715ac034f568603bce9a1b8f4a30c12ce \
    && git submodule update --init \
    && meson setup build -Dpykms=enabled -Dutils=false \
    && ninja -C build install \
    && ldconfig

# 4. Python dependencies
RUN apt update && apt install -y \
      python3-piexif python3-prctl python3-numpy python3-opencv \
      ros-humble-rmw-cyclonedds-cpp \ 
    && pip3 install --upgrade pip \
    && pip3 install \
        "numpy<2" \
        pyprctl \
        flask \
        onnxruntime \
        picamera2==0.3.21

# 5. Environment setup & Manual Link Fix
# We use hardcoded 3.10 paths since ROS Humble is Ubuntu 22.04 (Python 3.10)
RUN mkdir -p /usr/local/lib/python3.10/dist-packages && \
    # Link libcamera folder
    ln -sf /usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/libcamera /usr/local/lib/python3.10/dist-packages/libcamera && \
    # Link pykms shared objects
    find /usr/local/lib/aarch64-linux-gnu/python3.10/site-packages/ -name "pykms*" -exec ln -sf {} /usr/local/lib/python3.10/dist-packages/ \;

# Cover all bases for the Linker and Python
ENV LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages:/usr/local/lib/python3.10/site-packages:$PYTHONPATH

# 6. Verify imports with diagnostic info
RUN echo "--- DIAGNOSTICS ---" && \
    ls -l /usr/local/lib/python3.10/dist-packages/libcamera/ && \
    python3 -c "import sys; print('Search paths:', sys.path)" && \
    python3 -c "import libcamera; print('✓ libcamera imported')" && \
    python3 -c "import pykms; print('✓ pykms imported')" && \
    python3 -c "from picamera2 import Picamera2; print('✓ picamera2 imported')"

# ROS package setup
WORKDIR /app
RUN mkdir -p /app/src
COPY ros2_ws/src/j2cdynamics_camera /app/src/j2cdynamics_camera
COPY camera/model/best_int8.onnx /app/model/best_int8.onnx

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /app \
    && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && colcon build --base-paths src --event-handlers=console_direct+

COPY docker/camera/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["bash"]