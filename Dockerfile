FROM ros:foxy
ENV DEBIAN_FRONTEND=noninteractive

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

RUN echo "" && \
    echo "[Note] OS version  >>> Ubuntu 20.04 (Focal Fossa)" && \
    echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy" && \
    echo "[Note] Ellie workspace   >>> $HOME/ellie_ws" && \
    echo ""

ARG name_ros_version="foxy"
ARG name_ellie_workspace="ellie_ws"

RUN apt-get -y update  && apt-get install -y sudo locales && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
ENV LANG=en_US.UTF-8

ENV HOME=/home

WORKDIR $HOME
ENV VIRTUAL_ENV=$HOME/venv
RUN apt-get install -y python-is-python3 python3-virtualenv 
RUN virtualenv -p python3 $VIRTUAL_ENV 
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:$VIRTUAL_ENV/lib/python3.8/site-packages

RUN . /opt/ros/${name_ros_version}/setup.sh && \
    apt install -y --no-install-recommends python3-argcomplete python3-colcon-common-extensions python3-vcstool git wget && \
    apt install -y --no-install-recommends chromium-browser flac \
    libhdf5-dev libc-ares-dev libeigen3-dev gcc gfortran libgfortran5 \
    libatlas3-base libatlas-base-dev libopenblas-dev libopenblas-base libblas-dev \
    liblapack-dev cython3 openmpi-bin libopenmpi-dev libatlas-base-dev python3-dev &&\
    apt-get install -y python3-pyqt5 python3-pyqt5.qtwebengine && \
    apt-get install -y ffmpeg portaudio19-dev python3-pyaudio python3-opencv && \
    rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list


RUN pip install gdown && \ 
    gdown https://drive.google.com/u/0/uc?id=13_otGSGrsE1atBB2RjcPhOvONErKSFef && \
    pip install tensorflow-2.6.0-cp38-cp38-linux_aarch64.whl && \
    rm tensorflow-2.6.0-cp38-cp38-linux_aarch64.whl
    
COPY src/requirements_arm64.txt requirements_arm64.txt
RUN pip install -r requirements_arm64.txt

COPY EllieHumanoid $HOME/$name_ellie_workspace 
WORKDIR $HOME/$name_ellie_workspace 
RUN colcon build --symlink-install

RUN echo ". /opt/ros/$name_ros_version/setup.bash" >> /root/.bashrc && \
    echo ". $HOME/$name_ellie_workspace/install/local_setup.bash" >> /root/.bashrc && \
    echo "1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer">> /root/.bashrc && \
    apt-get update -y
ENV QT_XCB_GL_INTEGRATION=none
ENV QTWEBENGINE_DISABLE_SANDBOX=1

ENTRYPOINT ["/bin/bash"]


