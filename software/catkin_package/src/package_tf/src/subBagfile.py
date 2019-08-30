#!/usr/bin/env python


import rosbag
import rospy
import sys
import sensor_msgs.point_cloud2
import csv


#simplesmente le o bag 

def rosbag_reader(bagfile):
  print("entrei no reader")
  bag = rosbag.Bag(bagfile, 'r')
  print(bag)
  for topic, msg,t in bag.read_messages(topics=['/velodyne_points']):
    #convertendo as mensagens do ring para x,y,z
    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
      #print(point[0]) ##x
      #print(point[1]) ##y
      #print(point[2]) ##z
      row = [str(t.secs), str(t.nsecs), str(point[0]), str(point[1]),str(point[2]),str(point[3]),str(point[4])]
      with open('bagfileConverted.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)
      #csvFile.close();
      print("saving %s", point)
      #a=1
    #print("entrei no loop out")
  #print("sai no reader")
  bag.close()

if __name__=='__main__':
  csvHeader = ['sec','nsec', 'Pointx', 'Pointy', 'Pointz','Intensity','id']
  with open('bagfileConverted.csv', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(csvHeader)
  csvFile.close();
  rosbag_reader(sys.argv[1])
  
