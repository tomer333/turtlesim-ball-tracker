FROM ros:humble-ros-base
WORKDIR /app

# download apt
RUN apt update
RUN apt install -y ros-humble-vision-opencv python3-pip ros-humble-turtlesim ros-humble-rqt*
# install python packages
COPY requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt

# add user for dev container
RUN useradd -ms /bin/bash user
#no password for sudo
RUN echo '%user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# startup script instead of entrypoint
RUN echo 'source "/ros_exports.sh"' >> /etc/bash.bashrc
