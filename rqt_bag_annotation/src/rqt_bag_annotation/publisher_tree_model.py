import threading

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QStandardItem

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.data_items import ReadonlyItem, CheckableItem


class PublisherTreeModel(MessageTreeModel):
    _column_names = ['topic', 'type']

    def __init__(self, parent=None):
        super(PublisherTreeModel, self).__init__(parent)
        self.setColumnCount(2)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)

    def clear(self):
        super(PublisherTreeModel, self).clear()
        self.setHorizontalHeaderLabels(self._column_names)

    def get_publisher_ids(self, index_list):
        return [item._user_data['publisher_id'] for item in self._get_toplevel_items(index_list)]

    def remove_items_with_parents(self, index_list):
        for item in self._get_toplevel_items(index_list):
            self.removeRow(item.row())

    def handle_item_changed(self, item):
        if not self._item_change_lock.acquire(False):
            #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
            return

        if item.checkState() == Qt.Checked:
            new_value = Qt.Checked
        else:
            new_value = Qt.Unchecked
        self._remove_checked_children(item, new_value)

        # release lock
        self._item_change_lock.release()

    def _remove_checked_children(self, item, new_value):
        num_children = item.rowCount()
        for child_it in range(0, num_children):
            child = item.child(child_it)
            self._remove_checked_children(child, new_value)

        topic_name = item._path
        column_name = self._column_names[item.column()]
        item.setCheckState(new_value)

    def remove_publisher(self, publisher_id):
        for top_level_row_number in range(self.rowCount()):
            item = self.item(top_level_row_number)
            if item is not None and item._user_data['publisher_id'] == publisher_id:
                self.removeRow(top_level_row_number)
                return top_level_row_number
        return None

    def update_publisher(self, publisher_info):
        top_level_row_number = self.remove_publisher(publisher_info['publisher_id'])
        self.add_publisher(publisher_info, top_level_row_number)

    def add_publisher(self, publisher_info, top_level_row_number=None):
        # recursively create widget items for the message's slots
        parent = self
        slot = publisher_info['message_instance']
        slot_name = publisher_info['topic_name']
        slot_type_name = publisher_info['message_instance']._type
        slot_path = publisher_info['topic_name']
        user_data = {'publisher_id': publisher_info['publisher_id']}
        kwargs = {
            'user_data': user_data,
            'top_level_row_number': top_level_row_number,
            'expressions': publisher_info['expressions'],
        }
        top_level_row = self._recursive_create_items(parent, slot, slot_name, slot_type_name, slot_path, **kwargs)
        self.setColumnCount(2)
        # fill tree widget columns of top level item
        #if publisher_info['enabled']:
            #top_level_row[self._column_index['topic']].setCheckState(Qt.Checked)
        

    def _get_data_items_for_path(self, slot_name, slot_type_name, slot_path, **kwargs):
        if slot_name.startswith('/'):
            return (CheckableItem(slot_name), ReadonlyItem(slot_type_name), QStandardItem(''), ReadonlyItem(''))
        expression_item = QStandardItem('')
        expression_item.setToolTip('enter valid Python expression here, using "i" as counter and functions from math, random and time modules')
        return (CheckableItem(slot_name), QStandardItem(slot_type_name), ReadonlyItem(''), expression_item)

    def _recursive_create_items(self, parent, slot, slot_name, slot_type_name, slot_path, expressions={}, **kwargs):
        row, is_leaf_node = super(PublisherTreeModel, self)._recursive_create_items(parent, slot, slot_name, slot_type_name, slot_path, expressions=expressions, **kwargs)
        if is_leaf_node:
            expression_text = expressions.get(slot_path, repr(slot))
        row[self._column_index['topic']].setCheckState(Qt.Checked)
        return row