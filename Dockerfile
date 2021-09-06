FROM osrf/ros2:devel
ENV DISPLAY=192.168.178.20:0
WORKDIR /home/ellie_ws
ENV ROOT=TRUE
ADD ./ /home/ellie_ws/
EXPOSE 9090
RUN sudo apt update && sudo apt upgrade -y
RUN sudo pip3 install virtualenv -y
RUN cd /home/ellie_ws
RUN sudo virtualenv -p python3 ./venv
RUN source ./venv/bin/activate
RUN touch ./venv/COLCON_IGNORE

RUN sudo apt install ros-foxy-desktop
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN pip install PyQt5 -y
RUN pip install PyQtWebEngine -y
RUN pip install tflearn -y
RUN pip install nltk -y
RUN pip install tensorflow -y
RUN pip install SpeechRecognition -y
RUN pip install imutils -y
RUN pip install opencv-python -y
RUN pip install dlib -y
RUN pip install ffmpeg-python -y
RUN pip install gtts -y
RUN pip install pydub -y
RUN apt-get install -y gconf-service libasound2 libatk1.0-0 libc6 libcairo2 libcups2 libdbus-1-3 libexpat1 libfontconfig1 libgcc1 libgconf-2-4 libgdk-pixbuf2.0-0 libglib2.0-0 libgtk-3-0 libnspr4 libpango-1.0-0 libpangocairo-1.0-0 libstdc++6 libx11-6 libx11-xcb1 libxcb1 libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 libxi6 libxrandr2 libxrender1 libxss1 libxtst6 ca-certificates fonts-liberation libappindicator1 libnss3 lsb-release xdg-utils wget