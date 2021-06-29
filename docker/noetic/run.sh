#/bin/bash
sudo docker run -it --rm \
                --net host \
                --gpus all \
                --device /dev/video0:/dev/video0 \
                -v $(realpath ~/.ros/camera_info):/root/.ros/camera_info \
                koide3/monocular_person_following:noetic $@
