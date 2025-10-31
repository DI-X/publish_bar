import sys
import rclpy
from PySide6.QtWidgets import QApplication
from components.scripts.ros2.dynamicContentROS2 import DynamicContentRos2
from components.scripts.ros2.ros2Helper import ROSNode
from components.scripts.baseClass.publishBarBase import PublishBarBase

class PublishBarROS2(PublishBarBase):
    def __init__(self):
        self.ros_node = ROSNode()
        super().__init__()
        self.setWindowTitle("Publish Bar ROS2")

    def add_tab(self, state: dict = None, title: str = None):
        topic_name = f"robot/publish_bar_{next(self.topic_counter)}"
        tab = DynamicContentRos2(topic_name, self.ros_node)
        idx = self.tabs.addTab(tab, title or f"Tab {self.tabs.count() + 1}")
        self.tabs.setCurrentIndex(idx)
        if state:
            tab.load_state(state)

# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    win = PublishBarROS2()
    win.show()
    sys.exit(app.exec())
