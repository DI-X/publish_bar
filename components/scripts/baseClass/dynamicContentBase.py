from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QComboBox, QSlider,
)
from components.scripts.msgWidgets import *

class DynamicContentBase(QWidget):
    def __init__(self, topic_name="robot/publish_bar"):
        super().__init__()
        # each tab has its own LCM and message object (you may want shared LCM; using per-tab as before)
        self.msg_type = "Float32MultiArray"

        # layout
        main = QVBoxLayout(self)

        # top row: topic + type
        top = QHBoxLayout()
        top.addWidget(QLabel("Topic:"))
        self.topic_edit = QLineEdit(topic_name)
        top.addWidget(self.topic_edit)

        top.addWidget(QLabel("Message Type:"))
        self.type_box = QComboBox()
        self.type_box.addItems(["Float32MultiArray", "Twist", "Vec3", "Pose", "JointState", "String"])
        self.type_box.currentTextChanged.connect(self._on_type_changed)
        top.addWidget(self.type_box)
        main.addLayout(top)

        # mode / freq row (per-tab)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Publish Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["As Updates", "Continuous", "Once"])
        self.mode_box.currentTextChanged.connect(self.on_mode_changed)
        self.mode_box.setMaximumWidth(180)
        mode_row.addWidget(self.mode_box)

        self.freq_min_edit = QLineEdit("1"); self.freq_min_edit.setFixedWidth(50)
        self.freq_max_edit = QLineEdit("100"); self.freq_max_edit.setFixedWidth(50)
        self.freq_slider = QSlider(Qt.Horizontal); self.freq_slider.setRange(1, 100); self.freq_slider.setValue(10)
        self.freq_slider.setFixedWidth(210); self.freq_slider.setTickPosition(QSlider.TicksBelow); self.freq_slider.setTickInterval(1)
        self.freq_value_label = QLabel("10 Hz")

        self.freq_slider.valueChanged.connect(self.on_freq_slider_changed)
        self.freq_min_edit.editingFinished.connect(self.on_freq_minmax_changed)
        self.freq_max_edit.editingFinished.connect(self.on_freq_minmax_changed)

        mode_row.addWidget(QLabel("Min:")); mode_row.addWidget(self.freq_min_edit)
        mode_row.addWidget(QLabel("Max:")); mode_row.addWidget(self.freq_max_edit)
        mode_row.addWidget(self.freq_slider); mode_row.addWidget(self.freq_value_label)
        mode_row.addStretch()
        main.addLayout(mode_row)

        # buttons row for this tab
        btn_row = QHBoxLayout()
        self.add_slider_button = QPushButton("Add Slider")
        self.add_slider_button.clicked.connect(self.add_slider)
        self.publish_button = QPushButton("Publish")
        self.publish_button.clicked.connect(self.manual_publish)
        btn_row.addWidget(self.add_slider_button)
        btn_row.addWidget(self.publish_button)
        btn_row.addStretch()
        main.addLayout(btn_row)

        # --- Dynamic Area ---
        ## no spacing no cllaps for groups
        self.dynamic_area = QVBoxLayout()
        self.dynamic_area.setContentsMargins(0, 0, 0, 0)
        self.dynamic_area.setSpacing(4)
        main.addLayout(self.dynamic_area, stretch=1)

        # internal references to type widgets (created in switch)
        self.type_widget = None

        # timer per-tab for Continuous mode
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.send_msg)
        self.publish_timer.setInterval(int(1000 / 10))  # default 10 Hz

        # initialize UI
        self.show_freq_controls(False)
        self._on_type_changed(self.type_box.currentText())
        self.on_mode_changed(self.mode_box.currentText())
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
        elif t == "String":
            self.type_widget = StringWidget()
            self.add_slider_button.setEnabled(False)
        else:
            self.type_widget = FloatArrayWidget(self.slider_changed)
            self.add_slider_button.setEnabled(True)
            # add to layout
        if self.type_widget:
            self.dynamic_area.addWidget(self.type_widget)

        topic = self.topic_edit.text()
        self.update_publisher(topic, self.msg_type)
    # ---------------- mode / freq handling ----------------
    def on_mode_changed(self, mode):
        if mode == "Continuous":
            self.publish_button.setEnabled(False)
            self.show_freq_controls(True)
            # set interval from slider
            freq = max(1, self.freq_slider.value())
            self.publish_timer.setInterval(int(1000 / freq))
            self.publish_timer.start()
        elif mode == "As Updates":
            self.publish_button.setEnabled(False)
            self.show_freq_controls(False)
            self.publish_timer.stop()
        elif mode == "Once":
            self.publish_button.setEnabled(True)
            self.show_freq_controls(False)
            self.publish_timer.stop()

    def show_freq_controls(self, show):
        for w in (self.freq_min_edit, self.freq_max_edit, self.freq_slider, self.freq_value_label):
            w.setVisible(show)

    def on_freq_slider_changed(self, v):
        self.freq_value_label.setText(f"{v} Hz")
        if self.mode_box.currentText() == "Continuous":
            self.publish_timer.setInterval(int(1000 / max(1, v)))

    def on_freq_minmax_changed(self):
        try:
            mn = int(self.freq_min_edit.text())
            mx = int(self.freq_max_edit.text())
            if mn < 1 or mx <= mn:
                return
            self.freq_slider.setRange(mn, mx)
            # ensure current slider value still valid
            if self.freq_slider.value() < mn:
                self.freq_slider.setValue(mn)
        except Exception:
            pass

    # ---------------- slider add / callbacks ----------------
    def add_slider(self):
        if isinstance(self.type_widget, FloatArrayWidget):
            self.type_widget.add_slider(f"Gain {len(self.type_widget.sliders)+1}", -10.0, 10.0, 0.0)
        elif isinstance(self.type_widget, JointStateWidget):
            self.type_widget.add_joint()

    def slider_changed(self):
        if self.mode_box.currentText() == "As Updates":
            self.send_msg()

    # ---------------- publish logic (example prints + LCM publish for FloatArray) ----------------
    def manual_publish(self):
        if self.mode_box.currentText() == "Once":
            self.send_msg()

    def update_publisher(self, topic:str, msg_type:str):
        pass

    def publish(self, topic:str, val: dict):
        pass

    def send_msg(self):
        topic = self.topic_edit.text()
        t = self.msg_type
        val_dict = dict()
        # Example behavior: print contents and publish Float32MultiArray via LCM
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
            vals = self.type_widget.group.get_slider_values()
            print(f"[Publish] Vec3 -> {topic} : {vals}")
            val_dict["vals"] = vals
        elif t == "Pose" and isinstance(self.type_widget, PoseWidget):
            pos = self.type_widget.position.get_slider_values()
            ori = self.type_widget.orientation.get_slider_values()
            print(f"[Publish] Pose -> {topic}")
            print("  Pos:", pos, "Ori:", ori)
            val_dict["ori"] = ori
            val_dict["pos"] = pos
        elif t == "Twist" and isinstance(self.type_widget, TwistWidget):
            lin = self.type_widget.linear.get_slider_values()
            ang = self.type_widget.angular.get_slider_values()
            print(f"[Publish] Twist -> {topic}")
            print("  Linear:", lin, "  Angular:", ang)
            val_dict["lin"] = lin
            val_dict["ang"] = ang
        elif t == "JointState" and isinstance(self.type_widget, JointStateWidget):
            pos_vals = self.type_widget.pos_group.get_slider_values()
            vel_vals = self.type_widget.vel_group.get_slider_values()
            eff_vals = self.type_widget.eff_group.get_slider_values()
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
        elif t == "String" and isinstance(self.type_widget, StringWidget):
            str_msg = self.type_widget.get_text()
            print(f"[Publish] String -> {topic}")
            print(f"  Message: {str_msg}")
            val_dict["data"] = str_msg
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
        self.update_publisher(topic, self.msg_type)
        self.publish(topic, val_dict)
    # ---------------- saving / loading per-tab state ----------------
    def save_state(self):
        """
        Return JSON-serializable dict describing the state of this tab.
        Contains:
          - topic_name
          - msg_type
          - publish_mode
          - freq_min, freq_max, freq_value
          - groups: list describing sliders per-type
        """
        state = {
            "topic_name": self.topic_edit.text(),
            "msg_type": self.msg_type,
            "publish_mode": self.mode_box.currentText(),
            "freq_min": self.freq_min_edit.text(),
            "freq_max": self.freq_max_edit.text(),
            "freq_value": self.freq_slider.value(),
            "groups": []
        }

        w = self.type_widget

        def slider_to_dict(s):
            return {
                "name": s.name_label.text(),
                "min": float(s.min_edit.text()) if s.min_edit.text() else 0.0,
                "max": float(s.max_edit.text()) if s.max_edit.text() else 0.0,
                "value": float(s.get_value())
            }

        if isinstance(w, FloatArrayWidget):
            state["groups"].append({
                "name": "Values",
                "sliders": [slider_to_dict(s) for s in w.sliders]
            })
        elif isinstance(w, Vec3Widget):
            state["groups"].append({
                "name": "Vec3",
                "sliders": [slider_to_dict(s) for s in w.group.sliders]
            })
        elif isinstance(w, PoseWidget):
            state["groups"].append({
                "name": "Position",
                "sliders": [slider_to_dict(s) for s in w.position.sliders]
            })
            state["groups"].append({
                "name": "Orientation",
                "sliders": [slider_to_dict(s) for s in w.orientation.sliders]
            })
        elif isinstance(w, TwistWidget):
            state["groups"].append({
                "name": "Linear",
                "sliders": [slider_to_dict(s) for s in w.linear.sliders]
            })
            state["groups"].append({
                "name": "Angular",
                "sliders": [slider_to_dict(s) for s in w.angular.sliders]
            })
        elif isinstance(w, JointStateWidget):
            # store each joint as one entry with position/velocity/effort subfields
            for i in range(len(w.position_sliders)):
                state["groups"].append({
                    "name": w.position_sliders[i].name_label.text(),
                    "position": slider_to_dict(w.position_sliders[i]),
                    "velocity": slider_to_dict(w.velocity_sliders[i]),
                    "effort": slider_to_dict(w.effort_sliders[i])
                })
        elif isinstance(w, StringWidget):
            state["groups"].append({
                "name": "String",
                "text": w.get_text()
            })

        return state

    def load_state(self, state: dict):
        """
        Given a previously-saved state dict, restore the tab to that state.
        """
        # basic fields
        self.topic_edit.setText(state.get("topic_name", "robot/publish_bar"))
        msg_type = state.get("msg_type", "Float32MultiArray")
        self.type_box.setCurrentText(msg_type)  # triggers type creation via _on_type_changed
        # restore publish mode and frequency
        self.mode_box.setCurrentText(state.get("publish_mode", "As Updates"))
        self.freq_min_edit.setText(str(state.get("freq_min", 1)))
        self.freq_max_edit.setText(str(state.get("freq_max", 100)))
        # set slider range first
        self.on_freq_minmax_changed()
        self.freq_slider.setValue(int(state.get("freq_value", 10)))
        self.on_freq_slider_changed(self.freq_slider.value())

        # now restore groups/sliders
        groups = state.get("groups", [])
        w = self.type_widget

        def restore_slider(s: slideBar, data: dict):
            if not s:
                return
            try:
                s.name_label.setText(data.get("name", s.name_label.text()))
            except Exception:
                pass
            try:
                s.min_edit.setText(str(data.get("min", s.min_edit.text())))
                s.max_edit.setText(str(data.get("max", s.max_edit.text())))
            except Exception:
                pass
            try:
                # compute slider position and update label
                s.set_value(data.get("value", s.get_value()))
            except Exception:
                pass

        if isinstance(w, FloatArrayWidget):
            w.clear()
            for g in groups:
                for sd in g.get("sliders", []):
                    w.add_slider(sd.get("name", "Gain"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
        elif isinstance(w, Vec3Widget):
            # expect only one group
            if groups:
                sliders_data = groups[0].get("sliders", [])
                # clear existing and add if counts mismatch
                existing = w.group.sliders
                if len(existing) < len(sliders_data):
                    # replace group: clear and add new
                    w.group.clear()
                    for sd in sliders_data:
                        w.group.add_slider(sd.get("name", "val"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
                else:
                    for s, sd in zip(existing, sliders_data):
                        restore_slider(s, sd)
        elif isinstance(w, PoseWidget):
            # find groups by name
            for g in groups:
                gname = g.get("name", "").lower()
                if gname == "position":
                    # ensure number of sliders matches
                    data_sliders = g.get("sliders", [])
                    if len(w.position.sliders) < len(data_sliders):
                        w.position.clear()
                        for sd in data_sliders:
                            w.position.add_slider(sd.get("name", "val"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
                    else:
                        for s, sd in zip(w.position.sliders, data_sliders):
                            restore_slider(s, sd)
                elif gname == "orientation":
                    data_sliders = g.get("sliders", [])
                    if len(w.orientation.sliders) < len(data_sliders):
                        w.orientation.clear()
                        for sd in data_sliders:
                            w.orientation.add_slider(sd.get("name", "val"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
                    else:
                        for s, sd in zip(w.orientation.sliders, data_sliders):
                            restore_slider(s, sd)
        elif isinstance(w, TwistWidget):
            for g in groups:
                gname = g.get("name", "").lower()
                if gname == "linear":
                    data_sliders = g.get("sliders", [])
                    if len(w.linear.sliders) < len(data_sliders):
                        w.linear.clear()
                        for sd in data_sliders:
                            w.linear.add_slider(sd.get("name", "val"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
                    else:
                        for s, sd in zip(w.linear.sliders, data_sliders):
                            restore_slider(s, sd)
                elif gname == "angular":
                    data_sliders = g.get("sliders", [])
                    if len(w.angular.sliders) < len(data_sliders):
                        w.angular.clear()
                        for sd in data_sliders:
                            w.angular.add_slider(sd.get("name", "val"), sd.get("min", -10.0), sd.get("max", 10.0), sd.get("value", 0.0))
                    else:
                        for s, sd in zip(w.angular.sliders, data_sliders):
                            restore_slider(s, sd)
        elif isinstance(w, JointStateWidget):
            # clear first
            w.clear()
            for joint_entry in groups:
                # each joint_entry should contain "name", "position", "velocity", "effort"
                name = joint_entry.get("name", None)
                pos = joint_entry.get("position", {})
                vel = joint_entry.get("velocity", {})
                eff = joint_entry.get("effort", {})
                w.add_joint(base_name=name,
                            min_val=pos.get("min", -10.0),
                            max_val=pos.get("max", 10.0),
                            value=pos.get("value", 0.0))
                idx = len(w.position_sliders) - 1
                # restore ranges/values and names
                try:
                    restore_slider(w.position_sliders[idx], pos)
                    restore_slider(w.velocity_sliders[idx], vel)
                    restore_slider(w.effort_sliders[idx], eff)
                    # ensure names are synchronized
                    new_name = joint_entry.get("name", w.position_sliders[idx].name_label.text())
                    w.position_sliders[idx].name_label.setText(new_name)
                    w.velocity_sliders[idx].name_label.setText(new_name)
                    w.effort_sliders[idx].name_label.setText(new_name)
                except Exception:
                    pass
        elif isinstance(w, StringWidget):
            for g in groups:
                txt = g.get("text", "")
                w.set_text(txt)
