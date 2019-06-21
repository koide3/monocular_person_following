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
from monocular_person_following.msg import *

from tf_pose import common
from tf_pose.estimator import Human, BodyPart


class VisualizationNode:
	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.image_pub = rospy.Publisher('~visualize', Image, queue_size=1)

		color_palette = numpy.uint8([(180 / 10 * i, 255, 255) for i in range(10)]).reshape(-1, 1, 3)
		self.color_palette = cv2.cvtColor(color_palette, cv2.COLOR_HSV2BGR).reshape(-1, 3)

		self.target_id = 0
		self.state_name = "NONE"
		self.confidences = {}
		self.use_face = rospy.get_param('~use_face', True)
		self.target_sub = rospy.Subscriber('/monocular_person_following/target', Target, self.target_callback)

		self.image = numpy.zeros((128, 128, 3), dtype=numpy.uint8)
		subs = [
			message_filters.Subscriber('image_rect', Image),
			message_filters.Subscriber('/pose_estimator/pose', Persons),
			message_filters.Subscriber('/monocular_people_tracking/tracks', TrackArray)
		]

		if self.use_face:
			subs.append(message_filters.Subscriber('/face_detector/faces', FaceDetectionArray))
			self.sync = message_filters.TimeSynchronizer(subs, 50)
			self.sync.registerCallback(self.callback)
		else:
			self.sync = message_filters.TimeSynchronizer(subs, 50)
			self.sync.registerCallback(self.callback_wo_face)

	def target_callback(self, target_msg):
		self.state_name = target_msg.state.data
		self.target_id = target_msg.target_id

		for i in range(len(target_msg.track_ids)):
			track_id = target_msg.track_ids[i]
			confidence = target_msg.confidences[i]
			if track_id not in self.confidences:
				self.confidences[track_id] = {}

			self.confidences[track_id]['conf'] = confidence

			num_classifiers = len(target_msg.classifier_names)
			for j in range(num_classifiers):
				self.confidences[track_id][target_msg.classifier_names[j].data] = target_msg.classifier_confidences[num_classifiers * i + j]

	def callback_wo_face(self, image_msg, poses_msg, tracks_msg):
		self.callback(image_msg, poses_msg, tracks_msg, None)

	def callback(self, image_msg, poses_msg, tracks_msg, faces_msg):
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
			self.draw_bounding_box(image, track)

			if track.id == self.target_id:
				self.draw_target_icon(image, track)

		if faces_msg is not None:
			face_scale = image_msg.width / float(faces_msg.image_width)

			def draw_face(bb, color, line_width):
				tl = numpy.float32((bb.x, bb.y)) * face_scale
				br = numpy.float32((bb.x + bb.width, bb.y + bb.height)) * face_scale

				cv2.rectangle(image, tuple(tl.astype(numpy.int32)), tuple(br.astype(numpy.int32)), color, line_width)

			for face in faces_msg.faces:
				if not len(face.face_roi):
					continue
				draw_face(face.face_roi[0], (255, 0, 0), 1)

				if not len(face.face_region):
					continue

				face_confidence = 0.0
				if face.track_id in self.confidences:
					face_confidence = self.confidences[face.track_id]['face']
				face_confidence = (face_confidence + 1.0) / 2.0

				color = (0, int(face_confidence * 255), int((1.0 - face_confidence) * 255))
				draw_face(face.face_region[0], color, 2)

		cv2.putText(image, self.state_name, (15, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 3)
		cv2.putText(image, self.state_name, (15, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 1)

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

	def draw_bounding_box(self, image, track):
		neck_ankle = numpy.float32(track.expected_measurement_mean).flatten()
		center = (neck_ankle[:2] + neck_ankle[2:]) / 2.0
		height = (neck_ankle[-1] - neck_ankle[1]) * 1.5
		width = height * 0.25
		half_extents = (width / 2.0, height / 2.0)

		tl = tuple(numpy.int32(center - half_extents))
		br = tuple(numpy.int32(center + half_extents))

		cv2.putText(image, "id:%d" % track.id, (tl[0] + 5, tl[1] + 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)
		cv2.putText(image, "id:%d" % track.id, (tl[0] + 5, tl[1] + 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 1)

		body_confidence = 0.0
		if track.id in self.confidences:
			body_confidence = self.confidences[track.id]['body']

			def order(name):
				return {'conf': 0, 'body': 1, 'face': 2}[name]
			keys = sorted(self.confidences[track.id].keys(), key=order)

			for i, key in enumerate(keys):
				conf = self.confidences[track.id][key]

				cv2.putText(image, "%s:%.2f" % (key, conf), (tl[0] + 5, br[1] - 5 - 12 * i), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), 2)
				cv2.putText(image, "%s:%.2f" % (key, conf), (tl[0] + 5, br[1] - 5 - 12 * i), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), 1)

		body_confidence = body_confidence + 0.5
		color = (0, int(255 * body_confidence), int(255 * (1 - body_confidence)))
		cv2.rectangle(image, tl, br, color, 2)

	def draw_target_icon(self, image, track):
		neck_ankle = numpy.float32(track.expected_measurement_mean).flatten()
		height = neck_ankle[-1] - neck_ankle[1]

		pt = (neck_ankle[:2] + (0, -height * 0.33)).astype(numpy.int32)
		pts = numpy.array([pt, pt + (15, -15), pt + (-15, -15)]).reshape(-1, 1, 2)
		cv2.polylines(image, [pts], True, (0, 0, 255), 2)

	def spin(self):
		if rospy.get_param('~show', True):
			cv2.imshow('image', self.image)
			cv2.waitKey(10)

		if self.image_pub.get_num_connections():
			img_msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.image, 'bgr8')
			self.image_pub.publish(img_msg)


def main():
	rospy.init_node('visualization_node')
	node = VisualizationNode()

	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()


if __name__ == '__main__':
	main()
