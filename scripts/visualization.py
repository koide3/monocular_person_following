#!/usr/bin/python
import cv2
import math
import numpy

import tf
import rospy
import cv_bridge
import message_filters
from tfpose_ros.msg import *
from sensor_msgs.msg import *
from monocular_people_tracking.msg import *

from tf_pose import common
from tf_pose.estimator import Human, BodyPart


class VisualizationNode:
	def __init__(self):
		self.tf_listener = tf.TransformListener()

		color_palette = numpy.uint8([(180 / 10 * i, 255, 255) for i in range(10)]).reshape(-1, 1, 3)
		self.color_palette = cv2.cvtColor(color_palette, cv2.COLOR_HSV2BGR).reshape(-1, 3)

		self.image = numpy.zeros((128, 128, 3), dtype=numpy.uint8)
		subs =  [
			message_filters.Subscriber('/top_front_camera/image_rect', Image),
			message_filters.Subscriber('/pose_estimator/pose', Persons),
			message_filters.Subscriber('/monocular_people_tracking/tracks', TrackArray)
		]
		self.sync = message_filters.TimeSynchronizer(subs, 50)
		self.sync.registerCallback(self.callback)

	def callback(self, image_msg, poses_msg, tracks_msg):
		image = cv_bridge.CvBridge().imgmsg_to_cv2(image_msg, 'bgr8')

		humans = []
		for p_idx, person in enumerate(poses_msg.persons):
			human = Human([])
			for body_part in person.body_part:
				part = BodyPart('', body_part.part_id, body_part.x, body_part.y, body_part.confidence)
				human.body_parts[body_part.part_id] = part

			humans.append(human)

		image = self.draw_humans(image, humans, imgcopy=False)

		for track in tracks_msg.tracks:
			self.draw_expected_measurement(image, track)

		self.image = image

	# taken from tfpose_ros/tf_pose/estimator.py
	def draw_humans(self, npimg, humans, imgcopy=False):
		if imgcopy:
			npimg = np.copy(npimg)

		canvas = npimg.copy()
		image_h, image_w = npimg.shape[:2]
		centers = {}
		for human in humans:
			# draw point
			for i in range(common.CocoPart.Background.value):
				if i not in human.body_parts.keys():
					continue

				body_part = human.body_parts[i]
				center = (int(body_part.x * image_w + 0.5), int(body_part.y * image_h + 0.5))
				centers[i] = center
				cv2.circle(canvas, center, 3, common.CocoColors[i], thickness=3, lineType=8, shift=0)

			# draw line
			for pair_order, pair in enumerate(common.CocoPairsRender):
				if pair[0] not in human.body_parts.keys() or pair[1] not in human.body_parts.keys():
					continue

				# npimg = cv2.line(npimg, centers[pair[0]], centers[pair[1]], common.CocoColors[pair_order], 3)
				cv2.line(canvas, centers[pair[0]], centers[pair[1]], common.CocoColors[pair_order], 3)

		npimg = npimg / 2 + canvas / 2
		return npimg

	def draw_expected_measurement(self, image, track):
		meas_mean = numpy.float32(track.expected_measurement_mean).flatten()
		meas_cov = numpy.float32(track.expected_measurement_cov).reshape(4, 4)

		def error_ellipse(cov, kai):
			w, v = numpy.linalg.eig(cov)

			extents = numpy.sqrt(kai * kai * w)
			angle = math.atan2(v[0, 1], v[1, 1])

			return (extents[0], extents[1], angle)

		neck_ellipse = error_ellipse(meas_cov[:2, :2], 3.0)
		ankle_ellipse = error_ellipse(meas_cov[2:, 2:], 3.0)
		neck_pos = tuple(meas_mean[:2].astype(numpy.int32))
		ankle_pos = tuple(meas_mean[2:].astype(numpy.int32))

		color = self.color_palette[track.id % len(self.color_palette)]
		color = tuple(int(x) for x in color)

		cv2.ellipse(image, neck_pos, neck_ellipse[:2], neck_ellipse[-1], 0, 360, color, 2)
		cv2.ellipse(image, ankle_pos, ankle_ellipse[:2], neck_ellipse[-1], 0, 360, color, 2)
		cv2.line(image, neck_pos, ankle_pos, color, 2)

	def spin(self):
		cv2.imshow('image', self.image)
		cv2.waitKey(10)


def main():
	rospy.init_node('visualization_node')
	node = VisualizationNode()

	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()


if __name__ == '__main__':
	main()
