#!/usr/bin/python
import rospy
import tf

rospy.init_node("tf_restamper")
tfpublisher= rospy.Publisher("tf",tf.msg.tfMessage, queue_size=100)

def tfcallback(tfmessage):
	for transf in tfmessage.transforms:
	    transf.header.stamp=rospy.Time.now()
	tfpublisher.publish(tfmessage)

tfproxy = rospy.Subscriber("tf_old",tf.msg.tfMessage,tfcallback)
rospy.spin()
