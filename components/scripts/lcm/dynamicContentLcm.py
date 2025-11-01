import lcm
from components.lcm_msg.lcm_std import Float32MultiArray, Twist, Pose, JointState, Vector3, String
from components.scripts.baseClass.dynamicContentBase import DynamicContentBase

class DynamicContentLcm(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar"):
        super().__init__(topic_name)
        # each tab has its own LCM and message object (you may want shared LCM; using per-tab as before)
        self.lc = lcm.LCM()
        self.lcm_msg = Float32MultiArray()

    def update_publisher(self, topic:str, msg_type:str):
        # create appropriate widget
        if msg_type == "Float32MultiArray":
            self.lcm_msg = Float32MultiArray()
        elif msg_type == "Twist":
              self.lcm_msg = Twist()
        elif msg_type == "Vec3":
            self.lcm_msg = Vector3()
        elif msg_type == "Pose":
            self.lcm_msg = Pose()
        elif msg_type == "JointState":
            self.lcm_msg = JointState()
        elif msg_type == "String":
            self.lcm_msg = String()
        else:
            self.lcm_msg = Float32MultiArray()

    def publish(self, topic:str, val: dict):
        if isinstance(self.lcm_msg, Float32MultiArray):
            self.lcm_msg.data = val["data"]
            self.lcm_msg.size = len(val["data"])
        elif isinstance(self.lcm_msg, Vector3):
            self.lcm_msg.x = val["vals"][0]
            self.lcm_msg.y = val["vals"][1]
            self.lcm_msg.z = val["vals"][2]
        elif isinstance(self.lcm_msg, Pose):
            self.lcm_msg.orientation.w = val["ori"][0]
            self.lcm_msg.orientation.x = val["ori"][1]
            self.lcm_msg.orientation.y = val["ori"][2]
            self.lcm_msg.orientation.z = val["ori"][3]
            self.lcm_msg.position.x = val["pos"][0]
            self.lcm_msg.position.y = val["pos"][1]
            self.lcm_msg.position.z = val["pos"][2]
        elif isinstance(self.lcm_msg, Twist):
            self.lcm_msg.linear.x = val["lin"][0]
            self.lcm_msg.linear.y = val["lin"][1]
            self.lcm_msg.linear.z = val["lin"][2]
            self.lcm_msg.angular.x = val["ang"][0]
            self.lcm_msg.angular.y = val["ang"][1]
            self.lcm_msg.angular.z = val["ang"][2]
        elif isinstance(self.lcm_msg, JointState):
            self.lcm_msg.position = val["pos"]
            self.lcm_msg.velocity = val["vel"]
            self.lcm_msg.effort = val["effort"]
            self.lcm_msg.name = val["names"]
            self.lcm_msg.size = len(val["names"])
        elif isinstance(self.lcm_msg, String):
            self.lcm_msg.data = val["data"]
        else:
            raise Exception(f"cannot find correct Lcm msg type: {self.lcm_msg}")

        try:
            self.lc.publish(topic, self.lcm_msg.encode())
        except:
            raise Exception(f"[Failed publish] {topic}")
