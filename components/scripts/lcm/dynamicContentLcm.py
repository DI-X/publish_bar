import lcm
from components.lcm_msg.lcm_std import Float32MultiArray, Twist, Pose, JointState, Vector3, String
from components.scripts.msgWidgets import *
from components.scripts.baseClass.dynamicContentBase import DynamicContentBase

class DynamicContentLcm(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar"):
        super().__init__(topic_name)
        # each tab has its own LCM and message object (you may want shared LCM; using per-tab as before)
        self.lc = lcm.LCM()
        self.lcm_msg = Float32MultiArray()

    # ---------------- type switching ----------------
    def _on_type_changed(self, t):
        self.msg_type = t
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
            self.lcm_msg = Float32MultiArray()
        elif t == "Twist":
            self.type_widget = TwistWidget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
            self.lcm_msg = Twist()
        elif t == "Vec3":
            self.type_widget = Vec3Widget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
            self.lcm_msg = Vector3()
        elif t == "Pose":
            self.type_widget = PoseWidget(self.slider_changed)
            self.add_slider_button.setEnabled(False)
            self.lcm_msg = Pose()
        elif t == "JointState":
            self.type_widget = JointStateWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)
            self.lcm_msg = JointState()
        elif t == "String":
            self.type_widget = StringWidget()
            self.add_slider_button.setEnabled(False)
            self.lcm_msg = String()
        else:
            self.type_widget = FloatArrayWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)
            self.lcm_msg = Float32MultiArray()
        # add to layout
        if self.type_widget:
            self.dynamic_area.addWidget(self.type_widget)

    def publish(self):
        topic = self.topic_edit.text()
        t = self.msg_type
        # Example behavior: print contents and publish Float32MultiArray via LCM
        if t == "Float32MultiArray" and isinstance(self.type_widget, FloatArrayWidget):
            values = [s.get_value() for s in self.type_widget.sliders]
            names = [s.name_label.text() for s in self.type_widget.sliders]
            print(f"[Publish] {t} -> {topic}")
            print("  Names:", names)
            print("  Values:", values)
            self.lcm_msg.data = values
            self.lcm_msg.size = len(values)
            try:
                self.lc.publish(topic, self.lcm_msg.encode())
            except :
                raise Exception(f"[msg type mismatch] {t} -> {topic}")
        elif t == "Vec3" and isinstance(self.type_widget, Vec3Widget):
            vals = [s.get_value() for s in self.type_widget.group.sliders]
            print(f"[Publish] Vec3 -> {topic} : {vals}")
            if isinstance(self.lcm_msg, Vector3):
                self.lcm_msg.x = vals[0]
                self.lcm_msg.y = vals[1]
                self.lcm_msg.z = vals[2]
            else:
                raise Exception(f"[msg type mismatch] {t} -> {topic}")
        elif t == "Pose" and isinstance(self.type_widget, PoseWidget):
            pos = [s.get_value() for s in self.type_widget.position.sliders]
            ori = [s.get_value() for s in self.type_widget.orientation.sliders]
            print(f"[Publish] Pose -> {topic}")
            print("  Pos:", pos, "Ori:", ori)
            if isinstance(self.lcm_msg, Pose):
                self.lcm_msg.orientation.w = ori[0]
                self.lcm_msg.orientation.x = ori[1]
                self.lcm_msg.orientation.y = ori[2]
                self.lcm_msg.orientation.z = ori[3]

                self.lcm_msg.position.x = pos[0]
                self.lcm_msg.position.y = pos[1]
                self.lcm_msg.position.z = pos[2]
            else:
                raise Exception(f"[msg type mismatch] {t} -> {topic}")

        elif t == "Twist" and isinstance(self.type_widget, TwistWidget):
            lin = [s.get_value() for s in self.type_widget.linear.sliders]
            ang = [s.get_value() for s in self.type_widget.angular.sliders]
            print(f"[Publish] Twist -> {topic}")
            print("  Linear:", lin, "  Angular:", ang)
            if isinstance(self.lcm_msg, Twist):
                self.lcm_msg.linear.x = lin[0]
                self.lcm_msg.linear.y = lin[1]
                self.lcm_msg.linear.z = lin[2]
                self.lcm_msg.angular.x = ang[0]
                self.lcm_msg.angular.y = ang[1]
                self.lcm_msg.angular.z = ang[2]
            else:
                raise Exception(f"[msg type mismatch] {t} -> {topic}")

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
            if isinstance(self.lcm_msg, JointState):
                self.lcm_msg.position = pos_vals
                self.lcm_msg.velocity = vel_vals
                self.lcm_msg.effort = eff_vals
                self.lcm_msg.name = names
                self.lcm_msg.size = len(names)
            else:
                raise Exception(f"[msg type mismatch] {t} -> {topic}")
        elif t == "String" and isinstance(self.type_widget, StringWidget):
            str_msg = self.type_widget.get_text()
            print(f"[Publish] String -> {topic}")
            print(f"  Message: {str_msg}")
            if isinstance(self.lcm_msg, String):
                self.lcm_msg.data = str_msg
            else:
                raise Exception(f"[msg type mismatch] {t} -> {topic}")
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

        try:
            self.lc.publish(topic, self.lcm_msg.encode())
        except:
            raise Exception(f"[Failed publish] {t} -> {topic}")
