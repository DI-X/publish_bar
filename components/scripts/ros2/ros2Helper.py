
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import JointState
from rclpy.node import Node

class ROSNode(Node):
    def __init__(self):
        super().__init__("publish_bar")

class ROS2Publisher():
    def __init__(self, topic_name="robot/publish_bar", ros_node=None):
        self.ros_node = ros_node
        self.topic_name = topic_name
        self.pub = self.ros_node.create_publisher(Float32MultiArray, self.topic_name, 1)
        self.ros_msg = Float32MultiArray()

    def update_topic(self, topic_name):
        if self.topic_name != topic_name:
            self.topic_name = topic_name
            self.pub = self.ros_node.create_publisher(Float32MultiArray, self.topic_name, 1)

    def update_msg_Type(self, msg_type):
        self.ros_node.destroy_publisher(self.pub)
        if msg_type == "Float32MultiArray":
            self.ros_msg = Float32MultiArray()
            self.pub = self.ros_node.create_publisher(Float32MultiArray, self.topic_name, 1)
        elif msg_type == "Twist":
            self.ros_msg = Twist()
            self.pub = self.ros_node.create_publisher(Twist, self.topic_name, 1)
        elif msg_type == "Vec3":
            self.ros_msg = Vector3()
            self.pub = self.ros_node.create_publisher(Vector3, self.topic_name, 1)
        elif msg_type == "Pose":
            self.ros_msg = Pose()
            self.pub = self.ros_node.create_publisher(Pose, self.topic_name, 1)
        elif msg_type == "JointState":
            self.ros_msg = JointState()
            self.pub = self.ros_node.create_publisher(JointState, self.topic_name, 1)
        else:
            raise Exception(f"Unknown ROS message type: {msg_type}")

    def publish(self, val:dict):
        if isinstance(self.ros_msg, Float32MultiArray):
            self.ros_msg.data = val["data"]
        elif isinstance(self.ros_msg, Twist):
            self.ros_msg.linear.x = val["lin"][0]
            self.ros_msg.linear.y = val["lin"][1]
            self.ros_msg.linear.z = val["lin"][2]
            self.ros_msg.angular.x = val["ang"][0]
            self.ros_msg.angular.y = val["ang"][1]
            self.ros_msg.angular.z = val["ang"][2]
        elif isinstance(self.ros_msg, Vector3):
            self.ros_msg.x = val["vals"][0]
            self.ros_msg.y = val["vals"][1]
            self.ros_msg.z = val["vals"][2]
        elif isinstance(self.ros_msg, Pose):
            self.ros_msg.orientation.w = val["ori"][0]
            self.ros_msg.orientation.x = val["ori"][1]
            self.ros_msg.orientation.y = val["ori"][2]
            self.ros_msg.orientation.z = val["ori"][3]

            self.ros_msg.position.x = val["pos"][0]
            self.ros_msg.position.y = val["pos"][1]
            self.ros_msg.position.z = val["pos"][2]
        elif isinstance(self.ros_msg, JointState):
            self.ros_msg.name = val["names"]
            self.ros_msg.position = val["pos"]
            self.ros_msg.velocity = val["vel"]
            self.ros_msg.effort = val["effort"]
        else:
            raise Exception(f"cannot publish ros msg: {self.ros_msg}")
        self.pub.publish(self.ros_msg)
