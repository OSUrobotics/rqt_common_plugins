#!/usr/bin/env python


import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
from interactive_markers.menu_handler import *



class RewardMarker(object):
	def __init__(self):
		self.server = InteractiveMarkerServer("simple_marker")
		self.menu_handler = MenuHandler()
		self.pose = Pose()
		self.pose_origin = Pose()
		self.reward = None

	def make_arrow_marker(self):
		arrow_marker = Marker()
		arrow_marker.header.frame_id = "base_link"
		arrow_marker.type = Marker.ARROW
		arrow_marker.pose.position.x = self.pose.position.x
		arrow_marker.pose.position.y = self.pose.position.y
		arrow_marker.pose.position.z = self.pose.position.z
		arrow_marker.scale.x = 1.0
		arrow_marker.scale.y = 0.20
		arrow_marker.scale.z = 0.25
		return arrow_marker

	def feedback(self, feedback):
		self.pose.position.x = feedback.pose.position.x + self.pose_origin.position.x
		self.pose.position.y = feedback.pose.position.y + self.pose_origin.position.y
		self.pose.position.z = feedback.pose.position.z + self.pose_origin.position.z
		print(self.pose_origin)

	def set_controls(self, feedback):
		a = 1
	def update(self, reward):
		if reward == self.reward:
			return
		else:
			self.reward = reward

		print("NEW MARKER")
		# create an interactive marker for our server
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		#int_marker.header.stamp=rospy.Time.now()
		int_marker.name = "reward_marker"
		int_marker.description = "Reward"

		# create a grey box marker
		box_marker = Marker()
		box_marker.type = Marker.CUBE
		entry = self.menu_handler.insert( "Controller", callback=self.set_controls );

		self.pose_origin.position.x = self.pose.position.x
		self.pose_origin.position.y = self.pose.position.y
		self.pose_origin.position.z = self.pose.position.z
		#print(self.pose_origin)
		print(self.pose)
		box_marker.pose = self.pose
		box_marker.scale.x = 0.45
		box_marker.scale.y = 0.45
		box_marker.scale.z = 0.45
		box_marker.color.r = 1.0
		box_marker.color.g = 1.0
		box_marker.color.b = 1.0
		box_marker.color.a = 1.0

		if self.reward > 0:
			box_marker.color.r = 0.0
			box_marker.color.g = self.reward/10.0
			box_marker.color.b = 0.0

		elif self.reward < 0:
			temp_reward = abs(self.reward)
			box_marker.color.r = temp_reward/10.0
			box_marker.color.g = 0.0
			box_marker.color.b = 0.0

		box_control = InteractiveMarkerControl()
		box_control.always_visible = True
		box_control.markers.append( box_marker )

		# add the control to the interactive marker
		int_marker.controls.append( box_control )

		# create a control which will move the box
		# this control does not contain any markers,
		# which will cause RViz to insert two arrows
		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 1
		control.orientation.y = 0
		control.orientation.z = 0
		control.name = "move_x"
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		arrow = self.make_arrow_marker()
		arrow.pose.orientation = control.orientation
		control.markers.append(arrow)
		int_marker.controls.append(control);

		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 0
		control.orientation.y = 0
		control.orientation.z = 1
		control.name = "move_y"
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		arrow = self.make_arrow_marker()
		arrow.pose.orientation = control.orientation
		control.markers.append(arrow)
		int_marker.controls.append(control);

		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 0
		control.orientation.y = 1
		control.orientation.z = 0
		control.name = "move_z"
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		arrow = self.make_arrow_marker()
		arrow.pose.orientation = control.orientation
		control.markers.append(arrow)
		int_marker.controls.append(control);

		# 'commit' changes and send to all clients
		self.server.clear()
		self.server.applyChanges()
		self.server.insert(int_marker, self.feedback)
		self.server.applyChanges()