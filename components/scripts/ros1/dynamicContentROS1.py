from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QComboBox, QSlider,
)
from components.scripts.msgWidgets import *
from components.scripts.ros1.ros1Helper import ROS1Publisher
from components.scripts.dynamicContentBase import DynamicContentBase

class DynamicContentRos1(DynamicContentBase):
    def __init__(self, topic_name="robot/publish_bar"):
        super().__init__(topic_name)
        # each tab has its own LCM and message object (you may want shared LCM; using per-tab as before)
        # self.msg_type = "Float32MultiArray"
        #
        # # layout
        # main = QVBoxLayout(self)
        #
        # # top row: topic + type
        # top = QHBoxLayout()
        # top.addWidget(QLabel("Topic:"))
        # self.topic_edit = QLineEdit(topic_name)
        # top.addWidget(self.topic_edit)
        #
        # top.addWidget(QLabel("Message Type:"))
        # self.type_box = QComboBox()
        # self.type_box.addItems(["Float32MultiArray", "Twist", "Vec3", "Pose", "JointState"])
        # self.type_box.currentTextChanged.connect(self._on_type_changed)
        # top.addWidget(self.type_box)
        # main.addLayout(top)
        #
        # # mode / freq row (per-tab)
        # mode_row = QHBoxLayout()
        # mode_row.addWidget(QLabel("Publish Mode:"))
        # self.mode_box = QComboBox()
        # self.mode_box.addItems(["As Updates", "Continuous", "Once"])
        # self.mode_box.currentTextChanged.connect(self.on_mode_changed)
        # self.mode_box.setMaximumWidth(180)
        # mode_row.addWidget(self.mode_box)
        #
        # self.freq_min_edit = QLineEdit("1"); self.freq_min_edit.setFixedWidth(50)
        # self.freq_max_edit = QLineEdit("100"); self.freq_max_edit.setFixedWidth(50)
        # self.freq_slider = QSlider(Qt.Horizontal); self.freq_slider.setRange(1, 100); self.freq_slider.setValue(10)
        # self.freq_slider.setFixedWidth(150); self.freq_slider.setTickPosition(QSlider.TicksBelow); self.freq_slider.setTickInterval(1)
        # self.freq_value_label = QLabel("10 Hz")
        #
        # self.freq_slider.valueChanged.connect(self.on_freq_slider_changed)
        # self.freq_min_edit.editingFinished.connect(self.on_freq_minmax_changed)
        # self.freq_max_edit.editingFinished.connect(self.on_freq_minmax_changed)
        #
        # mode_row.addWidget(QLabel("Min:")); mode_row.addWidget(self.freq_min_edit)
        # mode_row.addWidget(QLabel("Max:")); mode_row.addWidget(self.freq_max_edit)
        # mode_row.addWidget(self.freq_slider); mode_row.addWidget(self.freq_value_label)
        # mode_row.addStretch()
        # main.addLayout(mode_row)
        #
        # # buttons row for this tab
        # btn_row = QHBoxLayout()
        # self.add_slider_button = QPushButton("Add Slider")
        # self.add_slider_button.clicked.connect(self.add_slider)
        # self.publish_button = QPushButton("Publish")
        # self.publish_button.clicked.connect(self.manual_publish)
        # btn_row.addWidget(self.add_slider_button)
        # btn_row.addWidget(self.publish_button)
        # btn_row.addStretch()
        # main.addLayout(btn_row)
        #
        # # dynamic area for type-specific widget
        # # self.dynamic_area = QVBoxLayout()
        # # main.addLayout(self.dynamic_area)
        #
        # # --- Dynamic Area ---
        # ## no spacing no cllaps for groups
        # self.dynamic_area = QVBoxLayout()
        # self.dynamic_area.setContentsMargins(0, 0, 0, 0)
        # self.dynamic_area.setSpacing(4)
        # main.addLayout(self.dynamic_area, stretch=1)
        #
        # # internal references to type widgets (created in switch)
        # self.type_widget = None
        #
        # # timer per-tab for Continuous mode
        # self.publish_timer = QTimer()
        # self.publish_timer.timeout.connect(self.publish)
        # self.publish_timer.setInterval(int(1000 / 10))  # default 10 Hz
        self.ros_node = ROS1Publisher(topic_name)

        # # initialize UI
        # self.show_freq_controls(False)
        # self._on_type_changed(self.type_box.currentText())

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
        elif t == "Twist":
            self.type_widget = TwistWidget(self.slider_changed)
        elif t == "Vec3":
            self.type_widget = Vec3Widget(self.slider_changed)
        elif t == "Pose":
            self.type_widget = PoseWidget(self.slider_changed)
        elif t == "JointState":
            self.type_widget = JointStateWidget(self.slider_changed)
        else:
            self.type_widget = FloatArrayWidget(self.slider_changed)
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
