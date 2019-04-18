# monocular_person_following

This package provides a monocular vision-based person tracking and identification framework for person following robots. It first detects people using *tf-pose-estimation*, and then track them in the robot space with Unscented Kalman filter with the ground plane information. The combination of Convolutional Channel Features and Online Boosting runs on the top of the tracking module to keep tracking the target person with a re-identification capability. The entire system is designed so that it can be run on a Jetson TX2, and it can easily be reproduced and reused on a new mobile robot platform.

![system](data/imgs/system.png)

[[video]](https://www.youtube.com/watch?v=w-f8l1VNT9Q)

## Dependencies

- tf-pose-estimation
- monocular_people_tracking
- ccf_person_identification


## Install

### Install using TX2 image

You can flush your TX2 with the following image file. It contains all the required packages.

[[link]](https://drive.google.com/open?id=1c3A-jV_ozHyNqVMTGeVOCneFSiwHBC07)

### Install from source

#### Flann

```bash
sudo apt-get install libflann-dev
```

#### dlib

```bash
wget http://dlib.net/files/dlib-19.17.tar.bz2
tar xvf dlib-19.17.tar.bz2

echo "export DLIB_ROOT=/path/to/dlib" >> ~/.bashrc
source ~/.bashrc
```

#### tf-pose-estimation

```bash
cd catkin_ws/src
git clone https://github.com/koide3/tf-pose-estimation

# Follow the official installation instruction
cd tf-pose-estimation
sudo pip install -r requirements.txt

cd tf_pose/pafprocess
swig -python -c++ pafprocess.i && python setup.py build_ext --inplace

cd ../..
sudo python setup.py install
```

#### other packages

```bash
cd catkin_ws/src
git clone https://github.com/koide3/open_face_recognition.git
git clone https://github.com/koide3/ccf_person_identification.git
git clone https://github.com/koide3/monocular_people_tracking.git --recursive
git clone https://github.com/koide3/monocular_person_following.git
```

## Setup

### tf tree

Transformations between "odom" <-> "base_footprint" <--> "base_link" <--> "camera's frame" have to be defined.
[[example]](data/imgs/frames.png)

### camera calibration

Make sure that the intrinsic and extrinsic parameters of your camera have been calibrated. You have to be able to subscribe the intrinsic parameters from *camera_info* topic, and a proper transformation between "camera_frame" and "base_footprint" from tf.

## Start

In case you use Pioneer mobile base and Jetson TX2, you can use the following launch files.

```bash
roscd monocular_person_following/rviz
rviz -d monocular_person_following.rviz
```

```bash
# publish tf transformations and start pioneer
roslaunch monocular_person_following start_robot.launch
```

```bash
# start people detection/tracking/identification
roslaunch monocular_people_following jetson_person_following.launch
```

```bash
# start robot controller
rosrun monocular_person_following robot_controller.py cmd_vel:=/RosAria/cmd_vel
```


## Related packages

- [ccf_person_identification](https://github.com/koide3/ccf_person_identification)
- [monocular_people_tracking](https://github.com/koide3/monocular_people_tracking)
- [monocular_person_following](https://github.com/koide3/monocular_person_following)


## Papers
- Kenji Koide, Jun Miura, and Emanuele Menegatti, Monocular Person Tracking and Identification with Online Deep Feature Selection for Person Following Robots, Robotics and Autonomous Systems special issue on IAS-15. (under review)

- Kenji Koide and Jun Miura, Convolutional Channel Features-based Person Identification for Person Following Robots, 15th International Conference IAS-15, Baden-Baden, Germany, 2018 [[link]](https://www.researchgate.net/publication/325854919_Convolutional_Channel_Features-Based_Person_Identification_for_Person_Following_Robots_Proceedings_of_the_15th_International_Conference_IAS-15).