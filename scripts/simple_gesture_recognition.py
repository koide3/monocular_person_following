#!/usr/bin/python
import rospy
from monocular_people_tracking.msg import *
from monocular_person_following.srv import *


class GestureRecognizer:
	def __init__(self, track_id):
		self.track_id = track_id
		self.last_stamp = rospy.Time.now()

	def callback(self, track_msg, imprint):
		if not len(track_msg.associated):
			self.last_stamp = rospy.Time.now()
			return

		skeleton = track_msg.associated[0]

		neck = [x for x in skeleton.body_part if x.part_id == 1]
		r_elbow = [x for x in skeleton.body_part if x.part_id == 3]
		r_hand = [x for x in skeleton.body_part if x.part_id == 4]

		if not len(neck) or not len(r_elbow) or not len(r_hand):
			self.last_stamp = rospy.Time.now()
			return

		if r_elbow[0].y < neck[0].y and r_hand[0].y < neck[0].y:
			print self.track_id, rospy.Time.now() - self.last_stamp
			if rospy.Time.now() - self.last_stamp > rospy.Duration(5.0):
				imprint(track_msg.id)
				self.last_stamp = rospy.Time.now()


class SimpleGestureRecognitionNode:
	def __init__(self):
		print '--- simple_gesture_recognition ---'
		self.recognizers = {}
		print 'wait for service'
		rospy.wait_for_service('/monocular_person_following_node/imprint')
		self.imprint_service = rospy.ServiceProxy('/monocular_person_following_node/imprint', Imprint)

		self.sub = rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray, self.callback)
		print 'done'

	def callback(self, track_msg):
		for track in track_msg.tracks:
			if track.id not in self.recognizers:
				self.recognizers[track.id] = GestureRecognizer(track.id)

			self.recognizers[track.id].callback(track, self.imprint)

	def imprint(self, target_id):
		print 'reset target', target_id
		req = ImprintRequest()
		req.target_id = target_id

		self.imprint_service(req)


def main():
	rospy.init_node('simple_gesture_recognition_node')
	node = SimpleGestureRecognitionNode()
	rospy.spin()


if __name__ == '__main__':
	main()
