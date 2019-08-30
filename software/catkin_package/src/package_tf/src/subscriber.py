#!/usr/bin/env python

#import rospy


PKG = 'tf'
import roslib
roslib.load_manifest(PKG)


import csv
import rospy
import tf as TFX
from tf import transformations
import numpy
from std_msgs.msg import Int32
from tf.msg import tfMessage
import rosgraph.masterapi
import geometry_msgs.msg
import sensor_msgs.msg
from tf.srv import FrameGraph,FrameGraphResponse
#god
import tf2_ros
import tf2_msgs.msg
import re



################ funções para manipulação do msg
def fromTranslationRotation(self, translation, rotation):


      return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

def asMatrix(self, target_frame, hdr):

    translation,rotation = self.lookupTransform(target_frame, hdr.frame_id, hdr.stamp)
    return self.fromTranslationRotation(translation, rotation)




#pega as transformações de camera_initpara aft_mapped via listener
def callback(data):
	""" TODO: ADD DOC """
	p = re.compile('\d+')
	result = p.findall(str(data))
	print(result)
	rospy.loginfo("I receive %s", data)
	(trans,rot) = listener.lookupTransform('/camera_init', '/aft_mapped', rospy.Time(0))
	row = [result[1], result[2], str(trans[0]), str(trans[1]),str(trans[2]),str(rot[0]),str(rot[1]),str(rot[2]),str(rot[3])]
	print(trans,rot)
	with open('tf.csv', 'a') as csvFile:
		writer = csv.writer(csvFile)
		writer.writerow(row)
	csvFile.close();

def transformPoint(self, target_frame, ps):

    mat44 = self.asMatrix(target_frame, ps.header)
    xyz = tuple(numpy.dot(mat44, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3]
    r = geometry_msgs.msg.PointStamped()
    r.header.stamp = ps.header.stamp
    r.header.frame_id = target_frame
    r.point = geometry_msgs.msg.Point(*xyz)
    return r

rospy.init_node('random_subscriber', anonymous=True)

listener = TFX.TransformListener()


csvHeader = ['sec','nsec', 'transx', 'transy', 'transz', 'rotx','roty','rotz','rotw']

with open('tf.csv', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(csvHeader)
csvFile.close(); 	

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	#try:

		rospy.Subscriber('tf',tfMessage, callback)
		rospy.spin()
	#	(trans,rot) = listener.lookupTransform('/camera_init', '/aft_mapped', rospy.Time(0))
	#	print(trans)
	#except (TFX.LookupException, TFX.ConnectivityException, TFX.ExtrapolationException):
	#	continue

	#rospy.spin()

