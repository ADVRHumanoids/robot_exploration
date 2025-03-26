import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from PyQt5 import QtWidgets, uic
from std_msgs.msg import String  # Change this to your message type

from exploration_manager_actions.action import RequestExploration
from exploration_manager_msgs.msg import ExplorationStatus

from action_msgs.srv import CancelGoal

class MyExplorationGUI(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(MyExplorationGUI, self).__init__()
        uic.loadUi("/home/user/data/forest_ws/src/robot_exploration/exploration_gui/ui_files/exploration_gui.ui", self)  # Load UI file
        self.node = node  # Store ROS2 node reference
        
        self.tab_widget = self.findChild(QtWidgets.QTabWidget, "tabWidget")

        #STATUS TAB
        self.exp_status_text = self.findChild(QtWidgets.QLabel, "exploration_status")
        self.target_object_text = self.findChild(QtWidgets.QLabel, "target_object")
        self.target_location_text = self.findChild(QtWidgets.QLabel, "target_location")

        self.robot_status_text = self.findChild(QtWidgets.QLabel, "robot_status")
        self.robot_pos_text = self.findChild(QtWidgets.QLabel, "robot_pos")
        self.nav_pos_text = self.findChild(QtWidgets.QLabel, "nav_pos")

        #Cancel Button
        self.cancel_expl_button = self.findChild(QtWidgets.QPushButton, "cancel_expl_button")
        self.cancel_expl_button.clicked.connect(self.cancel_exploration)

        #SEND GOAL TAB
        #Acquire objects from "Send Goal Window"
        self.objects_menu_text = self.findChild(QtWidgets.QComboBox, "objects_menu")
        self.other_object_text = self.findChild(QtWidgets.QPlainTextEdit, "other_object")
        self.obj_update_button = self.findChild(QtWidgets.QPushButton, "update_button")
        self.obj_update_button.clicked.connect(self.update_object_list)

        #Send Goal Button
        self.send_goal_button = self.findChild(QtWidgets.QPushButton, "send_goal_button")
        self.send_goal_button.clicked.connect(self.send_goal)
        
        # Parameters
        self.update_objs_needed = True
        self.objects_in_menu = ["Others"]

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

        #Cancel Request
        self.cancel_exploration_srv = self.node.create_client(CancelGoal,
                                                              '/request_exploration/_action/cancel_goal')

        while not self.cancel_exploration_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.cancel_exploration_req = CancelGoal.Request()
        
    # Callback for Update Obj Button
    def update_object_list(self):
        self.update_objs_needed = True

    def send_goal(self):
        goal_msg = RequestExploration.Goal()
        
        if self.objects_menu_text.currentText() == "Others":
            goal_msg.object_name = self.other_object_text.toPlainText()
        else:
            goal_msg.object_name = self.objects_menu_text.currentText()

        self.req_exploration_action_srv.send_goal_async(goal_msg)

        self.target_object_text.setText(goal_msg.object_name)

    def cancel_exploration(self):
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
            for obj in data.objects_data:
                if obj.class_name not in self.objects_in_menu:
                    self.objects_in_menu.append(obj.class_name)
                    self.objects_menu_text.addItem(obj.class_name)

    #Update GUI Data
    def updateGUI(self):

        if self.tab_widget.currentIndex() == 1:
            # If "Others" is selected in the drop down menu --> Show Text
            select_object = self.objects_menu_text.currentText()

            if select_object == "Others":
                self.other_object_text.setVisible(True)
            else:
                self.other_object_text.setVisible(False)
        elif self.tab_widget.currentIndex() == 0:
            # Update based on ROS params
            if self.exploration_status != []:
                if not self.exploration_status.active_task or self.exploration_status.finished:
                    self.exp_status_text.setText("Finished")
                    self.exp_status_text.setStyleSheet("QLabel {color : green; }")
                else:
                    self.exp_status_text.setText("Running")
                    self.exp_status_text.setStyleSheet("QLabel {color : orange; }")

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

    def can_update_objs(self):
        return self.update_objs_needed