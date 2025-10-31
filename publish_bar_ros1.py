import sys
import rospy
from PySide6.QtWidgets import QApplication
from components.scripts.ros1.dynamicContentROS1 import DynamicContentRos1
from components.scripts.baseClass.publishBarBase import PublishBarBase

class TabbedMain(PublishBarBase):
    def __init__(self):
        rospy.init_node('publish_bar', anonymous=True)
        super().__init__()
        self.setWindowTitle("Publish Bar ROS1")

    def add_tab(self, state: dict = None, title: str = None):
        topic_name = f"robot/publish_bar_{next(self.topic_counter)}"
        tab = DynamicContentRos1(topic_name)
        idx = self.tabs.addTab(tab, title or f"Tab {self.tabs.count() + 1}")
        self.tabs.setCurrentIndex(idx)
        if state:
            tab.load_state(state)

# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TabbedMain()
    win.show()
    sys.exit(app.exec())
