
from components.scripts.ros2.ros2Helper import ROS2Publisher
from components.scripts.baseClass.dynamicContentBase import DynamicContentBase

class DynamicContentRos2(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar", ros_node=None):
        self.ros_node = ROS2Publisher(topic_name, ros_node)
        super().__init__(topic_name)

    def update_publisher(self, topic:str, msg_type:str):
        self.ros_node.update_publisher(topic, self.msg_type)

    def publish(self, topic:str, val: dict):
        self.ros_node.publish(val)