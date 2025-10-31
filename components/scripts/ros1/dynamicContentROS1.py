
from components.scripts.msgWidgets import *
from components.scripts.ros1.ros1Helper import ROS1Publisher
from components.scripts.dynamicContentBase import DynamicContentBase

class DynamicContentRos1(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar"):
        self.ros_node = ROS1Publisher(topic_name)
        super().__init__(topic_name)

    # ---------------- type switching ----------------
    def _on_type_changed(self, t):
        self.msg_type = t
        topic = self.topic_edit.text()
        # remove previous type widget
        if self.type_widget:
            # if it has clear, clear
            try:
                if hasattr(self.type_widget, "clear"):
                    self.type_widget.clear()
            except Exception:
                pass
            # remove widget from layout
            for i in reversed(range(self.dynamic_area.count())):
                w = self.dynamic_area.itemAt(i).widget()
                if w:
                    self.dynamic_area.removeWidget(w)
                    w.setParent(None)
            self.type_widget = None

        # create appropriate widget
        if t == "Float32MultiArray":
            self.type_widget = FloatArrayWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)
        elif t == "Twist":
            self.type_widget = TwistWidget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
        elif t == "Vec3":
            self.type_widget = Vec3Widget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
        elif t == "Pose":
            self.type_widget = PoseWidget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
        elif t == "JointState":
            self.type_widget = JointStateWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)
        else:
            self.type_widget = FloatArrayWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)

        self.ros_node.update_publisher(topic, t)
        # add to layout
        if self.type_widget:
            self.dynamic_area.addWidget(self.type_widget)

    def publish(self):
        topic = self.topic_edit.text()
        t = self.msg_type
        val_dict=dict()
        # Example behavior: print contents and publish Float32MultiArray via LCM
        if t == "Float32MultiArray" and isinstance(self.type_widget, FloatArrayWidget):
            values = [s.get_value() for s in self.type_widget.sliders]
            names = [s.name_label.text() for s in self.type_widget.sliders]
            print(f"[Publish] {t} -> {topic}")
            print("  Names:", names)
            print("  Values:", values)
            val_dict["data"] = values
            val_dict["names"] = names
        elif t == "Vec3" and isinstance(self.type_widget, Vec3Widget):
            vals = [s.get_value() for s in self.type_widget.group.sliders]
            print(f"[Publish] Vec3 -> {topic} : {vals}")
            val_dict["vals"] = vals
        elif t == "Pose" and isinstance(self.type_widget, PoseWidget):
            pos = [s.get_value() for s in self.type_widget.position.sliders]
            ori = [s.get_value() for s in self.type_widget.orientation.sliders]
            print(f"[Publish] Pose -> {topic}")
            print("  Pos:", pos, "Ori:", ori)
            val_dict["ori"] = ori
            val_dict["pos"] = pos
        elif t == "Twist" and isinstance(self.type_widget, TwistWidget):
            lin = [s.get_value() for s in self.type_widget.linear.sliders]
            ang = [s.get_value() for s in self.type_widget.angular.sliders]
            print(f"[Publish] Twist -> {topic}")
            print("  Linear:", lin, "  Angular:", ang)
            val_dict["lin"] = lin
            val_dict["ang"] = ang
        elif t == "JointState" and isinstance(self.type_widget, JointStateWidget):
            pos_vals = [s.get_value() for s in self.type_widget.position_sliders]
            vel_vals = [s.get_value() for s in self.type_widget.velocity_sliders]
            eff_vals = [s.get_value() for s in self.type_widget.effort_sliders]
            names = [s.name_label.text() for s in self.type_widget.position_sliders]
            print(f"[Publish] JointState -> {topic}")
            print("  Names:", names)
            print("  Position:", pos_vals)
            print("  Velocity:", vel_vals)
            print("  Effort:", eff_vals)
            val_dict["names"] = names
            val_dict["pos"] = pos_vals
            val_dict["vel"] = vel_vals
            val_dict["effort"] = eff_vals
        else:
            # generic scan for slideBar children
            values = []
            widget = self.type_widget
            if widget:
                # check common collections
                candidates = []
                if isinstance(widget, FloatArrayWidget):
                    candidates = widget.sliders
                elif isinstance(widget, Vec3Widget):
                    candidates = widget.group.sliders
                elif isinstance(widget, PoseWidget):
                    candidates = widget.position.sliders + widget.orientation.sliders
                elif isinstance(widget, TwistWidget):
                    candidates = widget.linear.sliders + widget.angular.sliders
                elif isinstance(widget, JointStateWidget):
                    candidates = widget.position_sliders + widget.velocity_sliders + widget.effort_sliders
                for s in candidates:
                    if hasattr(s, "get_value"):
                        values.append(s.get_value())
            if values:
                print(f"[Publish] {t} -> {topic} (generic):", values)
            raise Exception(f"[Publish] {t} -> {topic} (generic):")
        self.ros_node.update_publisher(self.topic_edit.text(), self.msg_type)
        self.ros_node.publish(val_dict)
