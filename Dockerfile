FROM ubuntu:20.04
RUN apt update -y && apt install sudo && sudo apt install wget && \
    wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/install.sh && \
    sudo chmod 755 ./install.sh
RUN bash ./install.sh
RUN sudo apt update -y
ENTRYPOINT ["/bin/bash"]