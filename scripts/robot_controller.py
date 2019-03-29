#!/usr/bin/python
import tf
import math
import rospy

from geometry_msgs.msg import *
from monocular_people_tracking.msg import *
from monocular_person_following.msg import *


class RobotControllerNode:
	def __init__(self):
		self.tf_listener = tf.TransformListener()

		self.enable_back = rospy.get_param('~enable_back', False)
		self.max_vx = rospy.get_param('~max_vx', 0.1)
		self.max_va = rospy.get_param('~max_va', 0.1)
		self.gain_vx = rospy.get_param('~gain_vx', 0.1)
		self.gain_va = rospy.get_param('~gain_va', 0.1)
		self.distance = rospy.get_param('~distance', 3.5)
		self.timeout = rospy.get_param('~timeout', 0.5)

		self.last_time = rospy.Time(0)
		self.target_id = -1

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.target_sub = rospy.Subscriber('/monocular_person_following/target', Target, self.target_callback)
		self.tracks_sub = rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray, self.tracks_callback)

	def target_callback(self, target_msg):
		self.target_id = target_msg.target_id

	def tracks_callback(self, tracks_msg):
		self.last_time = tracks_msg.header.stamp

		target = [x for x in tracks_msg.tracks if x.id == self.target_id]

		if not len(target):
			self.cmd_vel_pub.publish(Twist())
			return

		point = PointStamped()
		point.header = tracks_msg.header
		point.point = target[0].pos

		try:
			self.tf_listener.waitForTransform('base_link', tracks_msg.header.frame_id, tracks_msg.header.stamp, rospy.Duration(0.5))
			target_pos = self.tf_listener.transformPoint('base_link', point)
		except:
			print 'failed to lookup transform between base_link and', tracks_msg.header.frame_id
			self.cmd_vel_pub.publish(Twist())
			return

		theta = math.atan2(target_pos.point.y, target_pos.point.x)
		print 'x, y, theta:', target_pos.point.x, target_pos.point.y, theta, math.radians(45)

		va = min(self.max_va, max(-self.max_va, theta * self.gain_va))
		vx = 0.0
		if abs(theta) < math.radians(45):
			vx = (target[0].pos.x - self.distance) * self.gain_vx
			print 'raw vx', vx
			min_vx = -self.max_vx if self.enable_back else 0.0
			vx = min(self.max_vx, max(min_vx, vx))
		else:
			print 'rotation too big'


		twist = Twist()
		twist.linear.x = vx
		twist.angular.z = va

		self.cmd_vel_pub.publish(twist)

	def spin(self):
		if (rospy.Time.now() - self.last_time).to_sec() > self.timeout:
			self.cmd_vel_pub.publish(Twist())


def main():
	rospy.init_node('robot_controller_node')
	node = RobotControllerNode()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

if __name__ == '__main__':
	main()
