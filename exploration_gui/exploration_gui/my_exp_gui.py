import sys
import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QStringListModel
from std_msgs.msg import String  # Change this to your message type

from exploration_manager_actions.action import RequestExploration
from exploration_manager_msgs.msg import ExplorationStatus

from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal

class MyExplorationGUI(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(MyExplorationGUI, self).__init__()
        uic.loadUi("/home/user/data/forest_ws/src/robot_exploration/exploration_gui/ui_files/exploration_gui.ui", self)  # Load UI file
        self.node = node  # Store ROS2 node reference
        
        self.tab_widget = self.findChild(QtWidgets.QTabWidget, "tabWidget")

        #STATUS TAB
        self.exp_status_text = self.findChild(QtWidgets.QLabel, "exploration_status")
        self.task_type_text = self.findChild(QtWidgets.QLabel, "task_type")

        self.target_object_text = self.findChild(QtWidgets.QLabel, "target_object")
        self.target_location_text = self.findChild(QtWidgets.QLabel, "target_location")
        self.images_collected_text = self.findChild(QtWidgets.QLabel, "images_collected")


        self.robot_status_text = self.findChild(QtWidgets.QLabel, "robot_status")
        self.robot_pos_text = self.findChild(QtWidgets.QLabel, "robot_pos")
        self.nav_pos_text = self.findChild(QtWidgets.QLabel, "nav_pos")

        #Cancel Button
        self.cancel_expl_button = self.findChild(QtWidgets.QPushButton, "cancel_expl_button")
        self.cancel_expl_button.clicked.connect(self.cancel_exploration_button_cb)

        #SEND GOAL TAB
        #Acquire objects from "Send Goal Window"
        self.objects_menu_text = self.findChild(QtWidgets.QComboBox, "objects_menu")
        self.other_object_text = self.findChild(QtWidgets.QPlainTextEdit, "other_object")
        self.obj_update_button = self.findChild(QtWidgets.QPushButton, "update_button")
        self.obj_update_button.clicked.connect(self.update_object_list_button_cb)

        #Send Goal Button
        self.send_goal_button = self.findChild(QtWidgets.QPushButton, "send_goal_button")
        self.send_goal_button.clicked.connect(lambda:self.send_goal_button_cb(1))
        #Inspect
        self.inspect_button = self.findChild(QtWidgets.QPushButton, "inspect_button")
        self.inspect_button.clicked.connect(lambda:self.send_goal_button_cb(2))
        
        # Parameters
        self.update_objs_needed = True
        self.objects_in_menu = ["Others"]

        #OBJECTS TAB
        self.obj_update_button2 = self.findChild(QtWidgets.QPushButton, "update_button_2")
        self.obj_update_button2.clicked.connect(self.update_object_list_button_cb)

        self.known_objects_list = self.findChild(QtWidgets.QListWidget, "known_objects_list")
        self.obj_centroid_text = self.findChild(QtWidgets.QLabel, "obj_centroid")
        self.properties_window = self.findChild(QtWidgets.QWidget, "properties_window")

        #DATA MANAGER TAB
        self.save_data_button = self.findChild(QtWidgets.QPushButton, "save_data_button")
        self.save_data_button.clicked.connect(self.save_data_button_cb)

        self.saved_frame = self.findChild(QtWidgets.QFrame, "saved_frame")
        self.saved_frame.setVisible(False)
        self.save_frame_timer = 0

        # ROS2 STATUS TAB
        self.save_img_label = self.findChild(QtWidgets.QLabel, "save_img_label")
        self.cancel_goal_label = self.findChild(QtWidgets.QLabel, "cancel_goal_label")
        self.get_frontiers_label = self.findChild(QtWidgets.QLabel, "get_frontiers_label")
        self.nav_to_pose_label = self.findChild(QtWidgets.QLabel, "nav_to_pose_label")
        self.get_objects_label = self.findChild(QtWidgets.QLabel, "get_objects_label")
        self.get_inspect_label = self.findChild(QtWidgets.QLabel, "get_inspect_label")
        self.set_cand_target_label = self.findChild(QtWidgets.QLabel, "set_cand_target_label")

        # ROS Setup
        #RequestExploration Action Client
        self.req_exploration_action_srv = ActionClient(self.node, RequestExploration, '/request_exploration')
        self.req_exploration_action_srv.wait_for_server()

        #ExplorationStatus Subscriber
        self.subscription = self.node.create_subscription(
            ExplorationStatus,
            '/exploration_status',
            self.exploration_status_cb,
            5)
        self.exploration_status = []

        #Cancel Request Srv Client
        self.cancel_exploration_srv = self.node.create_client(CancelGoal,
                                                              '/request_exploration/_action/cancel_goal')
        #Image Save Srv Client
        self.save_image_srv = self.node.create_client(Trigger,
                                                      '/save_image')

        while not self.cancel_exploration_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.cancel_exploration_req = CancelGoal.Request()
        self.objects = []
        
    # Callback for Update Obj Button
    def update_object_list_button_cb(self):
        self.update_objs_needed = True

    def send_goal_button_cb(self, task_id = 0):
        goal_msg = RequestExploration.Goal()
        
        if self.objects_menu_text.currentText() == "Others":
            goal_msg.object_name = self.other_object_text.toPlainText()
        else:
            goal_msg.object_name = self.objects_menu_text.currentText()

        goal_msg.task_id = task_id

        self.req_exploration_action_srv.send_goal_async(goal_msg)

        self.target_object_text.setText(goal_msg.object_name)

        print("Send Exploration Request (Action ID: "+ str(task_id)+")")


    def save_data_button_cb(self):
        trigger_req = Trigger.Request()
        
        future = self.save_image_srv.call_async(trigger_req)
        rclpy.spin_until_future_complete(self.node, future)

        self.saved_frame.setVisible(True)
        self.save_frame_timer = time.time()

    def cancel_exploration_button_cb(self):
        future = self.cancel_exploration_srv.call_async(self.cancel_exploration_req)
        rclpy.spin_until_future_complete(self.node, future)

    def exploration_status_cb(self, msg):        
        self.exploration_status = msg
        
    # Update Objects in DropDown Menu
    def setObjects(self, data):
        self.update_objs_needed = False
        # if self.objects_menu_text.view().isVisible():
            # return 
            
        # self.objects_menu_text.clear()

        if data != []:
            self.objects = data.objects_data
            for obj in self.objects:
                if obj.class_name not in self.objects_in_menu:
                    self.objects_in_menu.append(obj.class_name)
                    self.objects_menu_text.addItem(obj.class_name)

    #Update GUI Data
    def updateGUI(self):

        if self.tab_widget.currentIndex() == 0:
            self.updateExplorationStatusTab()
        elif self.tab_widget.currentIndex() == 1:
            self.updateSendGoalTab()
        elif self.tab_widget.currentIndex() == 2:
            self.updateObjectsTab()
        elif self.tab_widget.currentIndex() == 3:
            self.updateDataManagerTab()
        elif self.tab_widget.currentIndex() == 4:
            self.updateRosStatusTab()

    def can_update_objs(self):
        return self.update_objs_needed

    def updateExplorationStatusTab(self):
        # Update based on ROS params
        if self.exploration_status != []:
            if not self.exploration_status.active_task or self.exploration_status.finished:
                self.exp_status_text.setText("Finished")
                self.exp_status_text.setStyleSheet("QLabel {color : green; }")
            else:
                self.exp_status_text.setText("Running")
                self.exp_status_text.setStyleSheet("QLabel {color : orange; }")

            if self.exploration_status.task_id == 1:
                self.task_type_text.setText("Find/Reach Object")
            elif self.exploration_status.task_id == 2:
                self.task_type_text.setText("Inspect Object")
            else:
                self.task_type_text.setText("- - -")

            self.images_collected_text.setText(str(self.exploration_status.images_collected))

            temp_string = ""

            self.target_object_text.setText(self.exploration_status.target_object)
            if self.exploration_status.location_known:
                temp_string = f'X: {self.exploration_status.object_target_pos.x:.2f} \
                                Y: {self.exploration_status.object_target_pos.y:.2f}'
            else:
                temp_string = "Unknown"
            self.target_location_text.setText(temp_string)
            
            if self.exploration_status.is_driving:
                self.robot_status_text.setText("(Driving)")
            else:
                self.robot_status_text.setText("(Idle)")
            
            temp_string = f'X: {self.exploration_status.robot_pos.x:.2f}    \
                            Y: {self.exploration_status.robot_pos.y:.2f}'
            self.robot_pos_text.setText(temp_string)

            temp_string = f'X: {self.exploration_status.nav_target_pose.position.x:.2f}    \
                            Y: {self.exploration_status.nav_target_pose.position.y:.2f}'
            self.nav_pos_text.setText(temp_string)

    def updateSendGoalTab(self):
        # If "Others" is selected in the drop down menu --> Show Text
        select_object = self.objects_menu_text.currentText()

        if select_object == "Others":
            self.other_object_text.setVisible(True)
        else:
            self.other_object_text.setVisible(False)

    def updateObjectsTab(self):
        if self.update_objs_needed:
            self.known_objects_list.clear()

            for obj in self.objects:
                self.known_objects_list.addItem(obj.class_name)

        selected_row = self.known_objects_list.currentRow()
        if selected_row > -1:
            self.properties_window.setEnabled(True)
            temp_string = f'X: {self.objects[selected_row].centroid.x:.2f} \
                            Y: {self.objects[selected_row].centroid.y:.2f}'
            self.obj_centroid_text.setText(temp_string)
        else:
            self.properties_window.setEnabled(False)

    def updateDataManagerTab(self):
        if self.save_frame_timer != 0:
            if time.time() - self.save_frame_timer > 1.0:
                self.saved_frame.setVisible(False)

    def updateRosStatusTab(self):
        if self.exploration_status != []:
            self.fillStatusLabel(self.exploration_status.save_img_srv, self.save_img_label)
            self.fillStatusLabel(self.exploration_status.cancel_nav_srv, self.cancel_goal_label)
            self.fillStatusLabel(self.exploration_status.get_frontiers_srv, self.get_frontiers_label)
            self.fillStatusLabel(self.exploration_status.nav_to_pose_srv, self.nav_to_pose_label)
            self.fillStatusLabel(self.exploration_status.get_objects_srv, self.get_objects_label)
            self.fillStatusLabel(self.exploration_status.get_insp_goals_srv, self.get_inspect_label)
            self.fillStatusLabel(self.exploration_status.send_cand_target_srv, self.set_cand_target_label)

    def fillStatusLabel(self, status_val, status_obj):
        if status_val:
            status_obj.setText("Active")
            status_obj.setStyleSheet("QLabel {color : green; }")
        else:
            status_obj.setText("Not Active")
            status_obj.setStyleSheet("QLabel {color : red; }")