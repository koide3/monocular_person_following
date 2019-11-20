# monocular_person_following

This package provides a monocular vision-based person tracking and identification framework for person following robots. It first detects people using *tf-pose-estimation*, and then track them in the robot space with Unscented Kalman filter with the ground plane information. The combination of Convolutional Channel Features and Online Boosting runs on the top of the tracking module to keep tracking the target person with a re-identification capability. The entire system is designed so that it can be run on a Jetson TX2/Xavier, and it can easily be reproduced and reused on a new mobile robot platform.

![system](data/imgs/system.png)

[[video]](https://www.youtube.com/watch?v=SsIrXxnOgaQ)

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/c7664fce1722461db5ffdc27eae59e9c)](https://www.codacy.com/app/koide3/monocular_person_following?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=koide3/monocular_person_following&amp;utm_campaign=Badge_Grade) [![Build Status](https://travis-ci.org/koide3/monocular_person_following.svg?branch=master)](https://travis-ci.org/koide3/monocular_person_following) on kinetic & melodic

## Dependencies

- dlib
- flann
- tensorflow
- tf-pose-estimation


## Install

### Install using system image

You can flush your TX2/Xavier with the following image file. It contains all the required packages.

- [[Jetson TX2 image (Jetpack 3.3)]](https://willbeavailable.soon)
- [[Jetson Xavier image (Jetpack 4.2)]](https://willbeavailable.soon)

### Install from source

We tested this package on TX2/Jetpack 3.3 and Xavier/Jetpack 4.2.

[[Install from source]](https://github.com/koide3/monocular_person_following/wiki/Install-from-source)

## Quick Test

Put your camera on a fixed place (horizontally) and run the following commands. Change *camera_xyz* and *camera_rpy* depending on your camera placement.

### Webcam

```bash
roscd monocular_person_following/rviz
rviz -d monocular_person_following.rviz
```

```bash
roslaunch monocular_person_following start_robot.launch webcam:=true publish_dummy_frames:=true camera_xyz:="0 0 0.9" camera_rpy:="0 0 0"
```

```bash
roslaunch monocular_person_following jetson_person_following.launch camera_name:=/top_front_camera/qhd
```

### CSI camera

```bash
roscd monocular_person_following/rviz
rviz -d monocular_person_following.rviz
```

```bash
roslaunch monocular_person_following start_robot.launch webcam:=false publish_dummy_frames:=true camera_xyz:="0 0 0.9" camera_rpy:="0 0 0"
```

```bash
roslaunch monocular_person_following jetson_person_following.launch camera_name:=/csi_cam_0/sd
```

![screenshot](data/imgs/screenshot.jpg)

## Setup your own person following robot

[[Setup your robot]](https://github.com/koide3/monocular_person_following/wiki/Setup-your-own-person-following-robot)

## Related packages

- [ccf_person_identification](https://github.com/koide3/ccf_person_identification)
- [monocular_people_tracking](https://github.com/koide3/monocular_people_tracking)
- [monocular_person_following](https://github.com/koide3/monocular_person_following)


## Papers
- Kenji Koide, Jun Miura, and Emanuele Menegatti, Monocular Person Tracking and Identification with Online Deep Feature Selection for Person Following Robots, Robotics and Autonomous Systems [[link]](https://www.researchgate.net/publication/336871285_Monocular_Person_Tracking_and_Identification_with_On-line_Deep_Feature_Selection_for_Person_Following_Robots).


- Kenji Koide and Jun Miura, Convolutional Channel Features-based Person Identification for Person Following Robots, 15th International Conference IAS-15, Baden-Baden, Germany, 2018 [[link]](https://www.researchgate.net/publication/325854919_Convolutional_Channel_Features-Based_Person_Identification_for_Person_Following_Robots_Proceedings_of_the_15th_International_Conference_IAS-15).
