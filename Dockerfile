FROM ellie:1.0.0
ENV DISPLAY=192.168.178.20:0
VOLUME D:/SS2021/PoppyProject/Projektarbeit/raspberry_pi4_debian=/home/ellie
WORKDIR /home/ellie
ENV ROOT=TRUE
