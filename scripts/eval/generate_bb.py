#!/usr/bin/python
import sys
import numpy
import rosbag


def main():
	images_bag = sys.argv[1]
	results_bag = images_bag[:images_bag.find('.bag')] + '_results.bag'

	dst_file = sys.argv[2]

	'read image timestamps'
	image_stamps = []
	with rosbag.Bag(images_bag, 'r') as bag:
		for topic, msg, stamp in bag.read_messages():
			if 'compressed' not in topic:
				continue

			image_stamps.append(msg.header.stamp)

	timestamp_map = {}
	for i, stamp in enumerate(image_stamps):
		timestamp_map[stamp] = i

	init_bbox = None
	with rosbag.Bag(results_bag, 'r') as bag:
		print 'read results'
		all_msgs = sorted([x for x in bag.read_messages()], key=lambda x: x[2])

		target_id = -1

		bbs = {}
		for topic, msg, stamp in all_msgs:
			if 'target' in topic:
				target_id = msg.target_id
			if 'tracks' in topic:
				target = [x for x in msg.tracks if x.id == target_id]

				if not len(target):
					continue

				if False:
					neck_ankle = target[0].expected_measurement_mean
					neck = numpy.float32(neck_ankle[:2])
					ankle = numpy.float32(neck_ankle[2:])
				else:
					if not len(target[0].associated_neck_ankle):
						continue

					neck_ankle = target[0].associated_neck_ankle
					neck = numpy.float32([neck_ankle[0].x, neck_ankle[0].y])
					ankle = numpy.float32([neck_ankle[1].x, neck_ankle[1].y])

				center = (neck + ankle) / 2.0
				height = (ankle[1] - neck[1])
				width = height * 0.25

				center[1] -= height * 0.1
				height = height * 1.1

				if width > 1e6 or height > 1e6:
					continue

				tl = numpy.int32((center - (width / 2, height / 2)))
				wh = numpy.int32((width, height))

				image_id = timestamp_map[msg.header.stamp]
				bbs[image_id] = tl.tolist() + wh.tolist()

				if init_bbox is None:
					init_bbox = tl.tolist() + wh.tolist()

	print init_bbox
	with open(dst_file, 'w') as file:
		bbox = init_bbox
		for i, timestamp in enumerate(image_stamps):
			if i in bbs:
				bbox = bbs[i]

			print >> sys.stdout, '%d\t%d\t%d\t%d\t%d' % tuple([i] + bbox)
			print >> file, '%d\t%d\t%d\t%d\t%d' % tuple([i] + bbox)


if __name__ == '__main__':
	main()
