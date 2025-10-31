
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import JointState
import rospy

class ROS1Publisher():
    def __init__(self, topic_name="robot/publish_bar"):
        self.topic_name = topic_name
        self.pub = rospy.Publisher(self.topic_name, Float32MultiArray, queue_size=1)
        self.ros_msg = Float32MultiArray()
        self.msg_type = "Float32MultiArray"

    def update_publisher(self, topic_name, msg_type):
        if self.topic_name != topic_name or self.msg_type != msg_type:
            self.topic_name = topic_name
            self.msg_type = msg_type
            if msg_type == "Float32MultiArray":
                self.ros_msg = Float32MultiArray()
                self.pub = rospy.Publisher(self.topic_name, Float32MultiArray, queue_size=1)
            elif msg_type == "Twist":
                self.ros_msg = Twist()
                self.pub = rospy.Publisher(self.topic_name, Twist, queue_size=1)
            elif msg_type == "Vec3":
                self.ros_msg = Vector3()
                self.pub = rospy.Publisher(self.topic_name, Vector3, queue_size=1)
            elif msg_type == "Pose":
                self.ros_msg = Pose()
                self.pub = rospy.Publisher(self.topic_name, Pose, queue_size=1)
            elif msg_type == "JointState":
                self.ros_msg = JointState()
                self.pub = rospy.Publisher(self.topic_name, JointState, queue_size=1)
            elif msg_type == "String":
                self.ros_msg = String()
                self.pub = rospy.Publisher(self.topic_name, String, queue_size=1)
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
        elif isinstance(self.ros_msg, String):
            self.ros_msg.data = val["data"]
        else:
            raise Exception(f"cannot publish ros msg: {self.ros_msg}")
        self.pub.publish(self.ros_msg)