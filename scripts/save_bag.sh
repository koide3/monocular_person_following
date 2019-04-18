#!/bin/bash

rosbag record -O $1 "/visualization_node/visualize/compressed" "/tf" "/scan" -e ".*/markers"  -e ".*/tracks" -e ".*/target" -e ".*/features/compressed" -e ".*/pose"
