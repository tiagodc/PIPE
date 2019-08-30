	import rosbag
	import rospy

	def rosbag_reader(bagfile):
		bag = rosbag.Bag(bagfile)
		for topic, msg in bag.read_messages(topics=['velodyne_points']):
 			print msg
		bag.close()

	if __name__=='__main__':
		rosbag_reader(sys.argv[1:])
		print(sys.argv[1:])
