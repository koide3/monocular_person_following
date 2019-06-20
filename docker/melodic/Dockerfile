FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends && apt-get install -y --no-install-recommends wget nano tmux swig python-pip build-essential libflann-dev ros-melodic-rviz ros-melodic-tf-conversions ros-melodic-cv-bridge ros-melodic-eigen-conversions \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

### update pip ###
RUN pip install -U pip
RUN pip install -U setuptools

### install dlib ###
WORKDIR /root
RUN wget http://dlib.net/files/dlib-19.17.tar.bz2
RUN tar xvf dlib-19.17.tar.bz2
ENV DLIB_ROOT=/root/dlib-19.17

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

### install tf-pose-estimation ###
RUN git clone https://github.com/koide3/tf-pose-estimation.git
RUN sed -i '15d' tf-pose-estimation/CMakeLists.txt
# WORKDIR /root/catkin_ws/src/tf-pose-estimation
# RUN pip install -r requirements.txt

WORKDIR /root/catkin_ws/src/tf-pose-estimation/tf_pose/pafprocess
RUN swig -python -c++ pafprocess.i && python setup.py build_ext --inplace

# WORKDIR /root/catkin_ws/src/tf-pose-estimation
# RUN python setup.py install

### install monocular_person_following ###
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/koide3/open_face_recognition.git
RUN git clone https://github.com/koide3/ccf_person_identification.git
RUN git clone https://github.com/koide3/monocular_people_tracking.git --recursive

COPY . /root/catkin_ws/src/monocular_person_following/

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
