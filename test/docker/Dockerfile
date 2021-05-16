FROM ryuichiueda/emcl-test-env:latest
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt update -y && apt upgrade -y
RUN apt install -y ros-melodic-grid-map
RUN cd /catkin_ws/src && \
    git clone https://github.com/ryuichiueda/value_iteration.git

RUN source ~/.bashrc && \
    cd /catkin_ws && \
    catkin_make
