#!/usr/bin/python
import cv2
import numpy
import dlib
import rospy
import cv_bridge
import message_filters
from tfpose_ros.msg import *
from sensor_msgs.msg import *
from monocular_people_tracking.msg import *
from monocular_person_following.msg import *


class FaceDetectorNode:
	def __init__(self):
		self.cv_bridge = cv_bridge.CvBridge()
		self.face_detector = dlib.get_frontal_face_detector()
		self.roi_scale = rospy.get_param('~roi_scale', 1.0)
		self.face_detection_upscale_pyramid = rospy.get_param('~face_detection_upscale_pyramid', 1)
		self.confidence_thresh = rospy.get_param('~confidence_thresh', -0.5)
		self.face_expantion_scale = rospy.get_param('~face_expantion_scale', 1.5)

		subs = [
			message_filters.Subscriber('image_rect', Image),
			message_filters.Subscriber('/monocular_people_tracking/tracks', TrackArray)
		]
		self.sync = message_filters.TimeSynchronizer(subs, 30)
		self.sync.registerCallback(self.callback)

		self.faces_pub = rospy.Publisher('~faces', FaceDetectionArray, queue_size=5)

	def callback(self, image_msg, tracks_msg):
		image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')

		faces_msg = FaceDetectionArray()
		faces_msg.header = image_msg.header
		faces_msg.image_width = image_msg.width
		faces_msg.image_height = image_msg.height

		for person in tracks_msg.tracks:
			face_msg = FaceDetection()
			face_msg.track_id = person.id
			faces_msg.faces.append(face_msg)

			roi = self.calc_roi(image.shape[1], image.shape[0], person)
			if roi is None:
				continue

			face_msg.face_roi.append(self.tlbr2bb(roi[0], roi[1]))

			face = self.detect_face(image, roi)
			if face is None:
				continue

			face_msg.face_region.append(self.tlbr2bb(face[0], face[1]))
			face_image = self.extract_roi(image, face[0], face[1])
			face_msg.face_image.append(self.cv_bridge.cv2_to_imgmsg(face_image, 'bgr8'))

		self.faces_pub.publish(faces_msg)

	def calc_roi(self, width, height, track_msg):
		if not len(track_msg.associated):
			return None

		pose = track_msg.associated[0]
		head = [x for x in pose.body_part if x.part_id == 0]
		neck = [x for x in pose.body_part if x.part_id == 1]

		if not len(head) or not len(neck):
			return None

		image_size = numpy.float32([width, height])
		head = numpy.float32([head[0].x, head[0].y]) * image_size
		neck = numpy.float32([neck[0].x, neck[0].y]) * image_size

		neck2head = (neck[1] - head[1])

		tl = head - neck2head * self.roi_scale
		br = head + neck2head * self.roi_scale

		tl = max(0, min(width, tl[0])), max(0, min(height, tl[1]))
		br = max(0, min(width, br[0])), max(0, min(height, br[1]))

		return tuple(numpy.int32(tl)), tuple(numpy.int32(br))

	def detect_face(self, image, roi):
		if roi[1][0] - roi[0][0] < 10 or roi[1][1] - roi[0][1] < 10:
			print 'too small roi!!'
			return None

		tl, br = roi

		roi_image = self.extract_roi(image, tl, br)
		dets, confidences, labels = self.face_detector.run(roi_image, self.face_detection_upscale_pyramid, self.confidence_thresh)

		if not len(dets):
			return None

		largest = sorted(dets, key=lambda x: -x.area())[0]

		center = largest.dcenter()
		face_left = int(center.x - (center.x - largest.left()) * self.face_expantion_scale)
		face_top = int(center.y - (center.y - largest.top()) * self.face_expantion_scale)
		face_right = int(center.x + (center.x - largest.left()) * self.face_expantion_scale)
		face_bottom = int(center.y + (center.y - largest.top()) * self.face_expantion_scale)

		face_tl = (tl[0] + face_left, tl[1] + face_top)
		face_br = (tl[0] + face_right, tl[1] + face_bottom)
		return face_tl, face_br

	def extract_roi(self, image, tl, br):
		return image[tl[1]: br[1], tl[0]: br[0], :]

	def tlbr2bb(self, tl, br):
		bb = BoundingBox2D()
		bb.x = tl[0]
		bb.y = tl[1]
		bb.width = br[0] - tl[0]
		bb.height = br[1] - tl[1]

		return bb


def main():
	print '--- face_detector_node ---'
	rospy.init_node('face_detector_node')
	node = FaceDetectorNode()

	print 'ready'
	rospy.spin()

if __name__ == '__main__':
	main()
