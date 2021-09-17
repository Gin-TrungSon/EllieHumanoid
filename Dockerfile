FROM ros:foxy
RUN apt update -y && apt install sudo && sudo apt install wget && \
    wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/install_docker.sh && \
    sudo chmod 755 ./install.sh
RUN bash ./install_docker.sh
RUN sudo apt update -y
ENTRYPOINT ["/bin/bash"]