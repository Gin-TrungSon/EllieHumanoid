FROM ros:foxy
RUN apt-get update -y && apt-get install sudo && sudo apt-get install wget && \
    wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/install_docker.sh && \
    sudo chmod 755 ./install.sh
RUN bash ./install_docker.sh
RUN sudo apt-get update -y
ENTRYPOINT ["/bin/bash"]
