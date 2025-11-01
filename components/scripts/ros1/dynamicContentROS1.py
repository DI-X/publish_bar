
from components.scripts.ros1.ros1Helper import ROS1Publisher
from components.scripts.baseClass.dynamicContentBase import DynamicContentBase

class DynamicContentRos1(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar"):
        self.ros_node = ROS1Publisher(topic_name)
        super().__init__(topic_name)

    def update_publisher(self, topic:str, msg_type:str):
        self.ros_node.update_publisher(topic, msg_type)

    def publish(self, topic:str, val: dict):
        self.ros_node.publish(val)
