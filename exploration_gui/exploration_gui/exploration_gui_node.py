import sys
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, uic
from std_msgs.msg import String  # Change this to your message type

from exploration_gui.my_exp_gui import *

from object_detection_srvs.srv import GetObjectsInfo

class ROS2QtNode(Node):
    def __init__(self, app):
        super().__init__('exploration_gui')
        self.gui = MyExplorationGUI(self)
        
        self.object_info_client = self.create_client(GetObjectsInfo,
                                                     '/get_objects_info')

        while not self.object_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.object_info_req = GetObjectsInfo.Request()
        self.object_info_req.object_class = "all"
        
        self.gui.show()
        self.app = app

        self.known_objects_data = []

    def main_loop(self):
        if self.gui.can_update_objs():
            self.process_ros_streams()

        self.process_qt_events()

    def process_ros_streams(self):
        self.object_info_req.object_class = "all"
        future = self.object_info_client.call_async(self.object_info_req)
        rclpy.spin_until_future_complete(self, future)

        self.known_objects_data = future.result()
        self.gui.setObjects(self.known_objects_data)

    def process_qt_events(self):
        self.app.processEvents()

        self.gui.updateGUI()


def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    node = ROS2QtNode(app)
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.main_loop()

    app.quit()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

