#!/usr/bin/python
import os
import sys
import cv2
import numpy

import rospy
import rosbag

from sensor_msgs.msg import *


def main():
	input_dir = sys.argv[1]
	output_bag = sys.argv[2]

	camera_info = CameraInfo()
	camera_info.K = [300, 0, 320, 0, 300, 240, 0, 0, 1]
	camera_info.D = [0, 0, 0, 0]

	image_files = sorted([input_dir + '/' + x for x in os.listdir(input_dir) if '.jpg' in x])

	with rosbag.Bag(output_bag, 'w') as dst_bag:
		stamp = rospy.Time(0)
		for image_file in image_files:
			print image_file
			image = cv2.imread(image_file)
			if image is None:
				print 'failed to read image...'
				continue

			stamp += rospy.Duration(1.0 / 14.2)

			compressed_msg = CompressedImage()
			compressed_msg.header.stamp = stamp
			compressed_msg.header.frame_id = 'camera_optical_frame'
			camera_info.header = compressed_msg.header

			ret, encoded = cv2.imencode('.jpg', image)
			buf = numpy.vstack([encoded]).tostring()

			compressed_msg.data = buf

			dst_bag.write('/top_front_camera/camera_info', camera_info, stamp)
			dst_bag.write('/top_front_camera/image_raw/compressed', compressed_msg, stamp)

			if rospy.is_shutdown():
				break


if __name__ == '__main__':
	main()
