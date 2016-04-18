import os
import time

import rospy
import rospkg
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QRect
from python_qt_binding import loadUi
import xacro

class AnnotationWidget(QWidget):
	def __init__(self, timeline_frame):
		QWidget.__init__(self)
		self.timeline_frame = timeline_frame
		rp = rospkg.RosPack()
		ui_file = os.path.join(rp.get_path('rqt_bag_annotation'), 'resource', 'annotation_widget.ui')
		#self.show()
		loadUi(ui_file, self)
		self.setObjectName('AnnotationWidget')
		rd_file = os.path.join(rp.get_path('turtlebot_description'), 'robots', 'kobuki_hexagons_kinect.urdf.xacro')
		a = xacro.parse(rd_file)
		xacro.process_includes(a, rd_file)
		xacro.eval_self_contained(a)
		rospy.set_param('robot_description', a.toxml()) 
		
		self._cur_annotation = None
		self.save_button.clicked[bool].connect(self.save)

	def activate(self):
		self.show()

	def fill_data(self, annotation, annotation_left, annotation_right):
		self._cur_annotation = annotation
		self.annotation_reward.setText(str(self._cur_annotation.reward))
		self.annotation_it.setText("Annotation Number: " + str(self._cur_annotation.it))
		self.label_x_left.setText("Min Time Stamp: " + str(round(self._cur_annotation.x_left,2)))
		self.label_x_right.setText("Max Time Stamp: " + str(round(self._cur_annotation.x_right,2)))

	def save(self):
		self._cur_annotation.reward = int(self.annotation_reward.text())
		self.timeline_frame._reward_changed = True
		self.timeline_frame.scene().update()

