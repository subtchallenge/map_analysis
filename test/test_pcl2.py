#!/user/bin/env python

import rospy
import random
import sys

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def test_cloud(topic):
	publisher = rospy.Publisher(topic, PointCloud2, queue_size=10)
	rospy.init_node('test_pcl2', anonymous=True)
	rate = rospy.Rate(0.5)
	point_cloud_size = 500
	while not rospy.is_shutdown():
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'darpa'

		fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1)]

		points = []
		for i in range(int(point_cloud_size)):
			point = [(random.random()-0.5) * 20, (random.random()-0.5) * 20, (random.random()-0.5) * 10]
			points.append(point)
		
		point_cloud_size *= 1.01

		cloudmsg = point_cloud2.create_cloud(header, fields, points)

		publisher.publish(cloudmsg)
		rate.sleep()

if __name__ == '__main__':
  topic = 'slam/cloud'
  if len(sys.argv) > 1:
    topic = sys.argv[1]
  try:
    test_cloud(topic)
  except rospy.ROSInterruptException:
    pass

