import os
import time

import rospy
import rospkg
import rosmsg
import rostopic
import bag_helper
import roslib
import collections
from rospy_message_converter import message_converter

from python_qt_binding.QtGui import QWidget, QIcon
from python_qt_binding.QtCore import Qt, QRect
from python_qt_binding import loadUi

from .publisher_tree_widget import PublisherTreeWidget
from .publisher_tree_model import PublisherTreeModel

class ExportWidget(QWidget):
	def __init__(self):
		QWidget.__init__(self)
		rp = rospkg.RosPack()
		ui_file = os.path.join(rp.get_path('rqt_bag_annotation'), 'resource', 'export_widget.ui')
		loadUi(ui_file, self)

		self.add_topic_button.setIcon(QIcon.fromTheme('list-add'))
		self.remove_topic_button.setIcon(QIcon.fromTheme('list-remove'))
		self.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))

		self.add_topic_button.clicked[bool].connect(self._handle_add_topic_clicked)
		self.remove_topic_button.clicked[bool].connect(self._handle_remove_topic_clicked)
		self.refresh_button.clicked[bool].connect(self._handle_refresh_clicked)
		self.export_button.clicked[bool].connect(self._handle_export_clicked)

		self.export_location_edit.setPlainText("./export_file.txt")
		self.rospack = rospkg.RosPack()

		self._exported_topics = list()
		self._exported_publisher_info = list()
		self._annotations = list()
		self._active_topics = list()
		self._dt = 0.1

		self._current_topic_paths = dict()
		self._current_msg_paths = dict()
		self._id_counter = 0
		self.current_output = ""

	def update(self):
		self._current_topic_paths = dict()
		self._current_msg_paths = dict()
		idx = 0
		while self.publisher_tree_widget.model().item(idx,0) != None:
			cur_item = self.publisher_tree_widget.model().item(idx,0)
			parent_topic = cur_item.text().encode("latin-1")
			self.convert_to_path(parent_topic, "" , cur_item)
			idx += 1

		timestamp_to_data = dict()

		for annotation in self._annotations:
			start_time = annotation.x_left
			end_time = annotation.x_right
			for time_stamp in self.drange(start_time, end_time, self._dt):
				timestamp_to_data[time_stamp] = dict()
				timestamp_to_data[time_stamp]['Reward'] = annotation.reward
			if len(self._exported_topics) is not 0:
				messages = self.bag.read_messages(topics=self._exported_topics, start_time=rospy.Time.from_sec(annotation.x_left), end_time=rospy.Time.from_sec(annotation.x_right))
				for msg_name, msg, time in messages:
					dictionary = dict()
					sec = time.to_sec()
					key = min(timestamp_to_data.keys(), key=lambda x:abs(x-sec))
					self.convert_msg_to_paths(msg_name, "", msg, msg_name, dictionary)
					timestamp_to_data[key] = dict(timestamp_to_data[key].items() + dictionary.items())
					
		timestamp_to_data = collections.OrderedDict(sorted(timestamp_to_data.items()))

		self.current_output = ""
		for key,value in timestamp_to_data.items():
			self.current_output += str(key) + ":" + str(value) + "\n"


		self.export_display_contents.setPlainText(self.current_output)

	def convert_msg_to_paths(self, parent_topic, path, cur_msg, cur_msg_topic, dictionary):
		if parent_topic not in self._current_topic_paths:
			return
		has_children = hasattr(cur_msg, '__slots__')
		if cur_msg_topic[0] == '/':
			path += cur_msg_topic
		else:
			path += "/" + cur_msg_topic
		if has_children:
			child_msg_topics = cur_msg.__slots__
			for child_msg_topic in child_msg_topics:
				child_msg = getattr(cur_msg, child_msg_topic)
				self.convert_msg_to_paths(parent_topic, path, child_msg, child_msg_topic, dictionary)
		else:
			if parent_topic not in self._current_msg_paths:
				self._current_msg_paths[parent_topic] = list()
			if path in self._current_topic_paths[parent_topic]:
				data = cur_msg
				dictionary[path] = data
			self._current_msg_paths[parent_topic].append(path)

	def convert_to_path(self, parent_topic, path, cur_item):
		num_children = cur_item.rowCount()
		cur_parent = cur_item.text().encode("latin-1")
		if cur_parent[0] == '/':
			path += cur_parent
		else:
			path += "/" + cur_parent
		for child_it in range(0, num_children):
			child = cur_item.child(child_it)
			self.convert_to_path(parent_topic, path, child)
		if num_children == 0:
			if cur_item.checkState() == 0:
				print("NOT CHECKED")
				return
			if parent_topic not in self._current_topic_paths:
				self._current_topic_paths[parent_topic] = list()
			self._current_topic_paths[parent_topic].append(path)

	def add_bag(self, bag):
		self.bag = bag
		self.update_bag()

	def update_bag(self):
		self.topics_dropdown.clear()
		for topic in bag_helper.get_topics(self.bag):
			if topic in self._active_topics:
				self.topics_dropdown.addItem(topic)
		self.update()

	def _handle_add_topic_clicked(self):
		selected_topic = str(self.topics_dropdown.currentText())

		if selected_topic in self._exported_topics:
			return
		topic_type = rostopic.get_topic_type(selected_topic)[0]


		publisher_info = {
			'topic_name': str(selected_topic),
			'type_name': str(topic_type),
			'enabled': bool(True),
		}

		publisher_info['publisher_id'] = self._id_counter
		self._id_counter += 1
		publisher_info['counter'] = 0
		publisher_info['enabled'] = publisher_info.get('enabled', False)
		publisher_info['expressions'] = publisher_info.get('expressions', {})

		publisher_info['message_instance'] = self._create_message_instance(publisher_info['type_name'])
		if publisher_info['message_instance'] is None:
			return

		self.publisher_tree_widget.model().add_publisher(publisher_info)
		self._exported_topics.append(selected_topic)
		self._exported_publisher_info.append(publisher_info)
		#self.update()

	def _handle_remove_topic_clicked(self):
		selected_topic = str(self.topics_dropdown.currentText())
		if selected_topic in self._exported_topics:
			self._exported_topics.remove(selected_topic)
		for publisher_info in self._exported_publisher_info:
			if publisher_info['topic_name'] == selected_topic:
				self.publisher_tree_widget.model().remove_publisher(publisher_info['publisher_id'])
				self._exported_publisher_info.remove(publisher_info)

		#self.update()

	def drange(self, start, stop, step):
		r = start
		while r < stop:
			yield r
			r += step

	def _create_message_instance(self, type_str):
		base_type_str, array_size = self._extract_array_info(type_str)

		base_message_type = roslib.message.get_message_class(base_type_str)
		if base_message_type is None:
			print 'Could not create message of type "%s".' % base_type_str
			return None

		if array_size is not None:
			message = []
			for _ in range(array_size):
				message.append(base_message_type())
		else:
			message = base_message_type()
		return message

	def _extract_array_info(self, type_str):
		array_size = None
		if '[' in type_str and type_str[-1] == ']':
			type_str, array_size_str = type_str.split('[', 1)
			array_size_str = array_size_str[:-1]
			if len(array_size_str) > 0:
				array_size = int(array_size_str)
			else:
				array_size = 0

		return type_str, array_size


	def _handle_refresh_clicked(self):
		self.update()

	def _handle_export_clicked(self):
		export_location = self.export_location_edit.toPlainText()

		#If using a directory, check if it exists
		if '/' in export_location:
			directory = export_location[0:export_location.rfind('/')]
			print(os.path.isdir(directory))
		f = open(export_location, 'w')
		f.write(str(self.current_output))
		f.close()