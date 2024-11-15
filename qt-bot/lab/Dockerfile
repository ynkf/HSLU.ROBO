FROM ros:noetic
LABEL maintainer "Stefan Siegler <stefan.siegler@stud.hslu.ch>"

SHELL ["/bin/bash","-c"]
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

# update and install git
RUN apt-get update && apt-get install -y git \
    && rm -rf /var/lib/apt/lists/* 

# clone the luxai software repo and copy the headers to the catkin_ws/src folder
RUN mkdir -p root/luxai && cd root/luxai \
    && git clone https://github.com/luxai-qtrobot/software \
    && cp -R software/headers $CATKIN_WS/src/headers 

COPY src ${CATKIN_WS}/src

COPY ai_nodes/custom/custom_interfaces ${CATKIN_WS}/src/custom_interfaces

# compile catkin workspace
RUN cd ${CATKIN_WS}/src \
	&& source /opt/ros/${ROS_DISTRO}/setup.bash \
	&& cd ${CATKIN_WS}  && catkin_make

WORKDIR /root/catkin_ws/src
RUN rm -rf QTrobot

# source .bash_aliases
RUN echo 'source /root/.bash_aliases' >> /root/.bashrc