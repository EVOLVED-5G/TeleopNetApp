# to a base image, add below required dependencies to install and execute commands

# base image in the template provides a linux environment with Python 3.10

# change base image if necessary

# dependencies of requirements.txt will be installed

# add/change commands to be executed in the container if necessary


FROM ros:noetic

# install dependencies
WORKDIR /app
RUN apt update && apt install -y python3-pip python3-catkin-tools
COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt


# copy all files and folders of the NetApp Python project into the image

COPY . .
WORKDIR ./src/evolvedApi/
RUN python3 setup.py install

# build the Teleop NetApp
WORKDIR /app
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin build'

# Assign environment variables
ENV FLASK_APP=./src/evolvedApi/evolvedApi/endpoint.py
ENV FLASK_ENV=development

#execute commands in the container
COPY package_build.sh .
COPY prepare.sh .
COPY capif_registration.json .

CMD ["./package_build.sh"]
