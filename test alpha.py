# tabbed_publish_bar.py
import sys
import json
import lcm
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QFileDialog, QComboBox, QGroupBox, QSlider,
    QSpacerItem, QSizePolicy, QTabWidget, QInputDialog
)
from lcm_msg.lcm_std import FloatMultiArray


# -------------------------
# slideBar (self-contained)
# -------------------------
class slideBar(QWidget):
    def __init__(self, name="Gain", min_val=0.0, max_val=100.0, value=50.0,
                 callback=None, name_changed_callback=None):
        super().__init__()
        self.layout = QHBoxLayout()
        self.name_label = QLineEdit(str(name))
        self.name_label.setFixedWidth(90)
        self.name_label.setStyleSheet("color: brown;")

        self.min_edit = QLineEdit(str(min_val)); self.min_edit.setFixedWidth(60)
        self.max_edit = QLineEdit(str(max_val)); self.max_edit.setFixedWidth(60)

        self.value_label = QLabel("")  # updated by update_value_label()
        self.value_label.setFixedWidth(70)
        self.value_label.setStyleSheet("color: darkred;")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        # set initial slider position based on value
        try:
            pos = int((float(value) - float(min_val)) / (float(max_val) - float(min_val)) * 1000)
        except Exception:
            pos = 0
        self.slider.setValue(max(0, min(1000, pos)))

        self.layout.addWidget(self.name_label)
        self.layout.addWidget(self.min_edit)
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.max_edit)
        spacer = QSpacerItem(10, 10, QSizePolicy.Fixed, QSizePolicy.Minimum)
        self.layout.addItem(spacer)
        self.layout.addWidget(self.value_label)
        self.setLayout(self.layout)

        self.callback = callback
        self.name_changed_callback = name_changed_callback

        self.slider.valueChanged.connect(self.on_change)
        self.min_edit.editingFinished.connect(self.on_minmax_changed)
        self.max_edit.editingFinished.connect(self.on_minmax_changed)

        if self.name_changed_callback:
            self.name_label.editingFinished.connect(self.on_name_changed)

        self.update_value_label()

    def get_value(self):
        try:
            min_v = float(self.min_edit.text())
            max_v = float(self.max_edit.text())
            val = min_v + (max_v - min_v) * self.slider.value() / 1000.0
            return val
        except Exception:
            return 0.0

    def get_min(self):
        try:
            return float(self.min_edit.text())
        except Exception:
            return 0.0

    def get_max(self):
        try:
            return float(self.max_edit.text())
        except Exception:
            return 1.0

    def set_range(self, min_val, max_val):
        self.min_edit.setText(str(min_val))
        self.max_edit.setText(str(max_val))
        # update slider position to keep same absolute value if possible
        self.on_minmax_changed()

    def set_value(self, value):
        try:
            min_v = float(self.min_edit.text())
            max_v = float(self.max_edit.text())
            pos = int((float(value) - min_v) / (max_v - min_v) * 1000)
        except Exception:
            pos = 0
        self.slider.setValue(max(0, min(1000, pos)))
        self.update_value_label()

    def update_value_label(self):
        v = self.get_value()
        self.value_label.setText(f"{v:.2f}")

    def on_change(self):
        # keep value label up-to-date and call callback
        self.update_value_label()
        if self.callback:
            try:
                self.callback()
            except Exception:
                pass

    def on_minmax_changed(self):
        # adjust slider position so relative fraction remains as close as possible
        try:
            min_v = float(self.min_edit.text())
            max_v = float(self.max_edit.text())
            if max_v <= min_v:
                return
            # re-evaluate current absolute value and re-map to slider
            cur_abs = self.get_value()
            pos = int((cur_abs - min_v) / (max_v - min_v) * 1000)
            self.slider.setValue(max(0, min(1000, pos)))
            self.update_value_label()
            if self.callback:
                try:
                    self.callback()
                except Exception:
                    pass
        except Exception:
            pass

    def on_name_changed(self):
        if self.name_changed_callback:
            try:
                self.name_changed_callback(self)
            except Exception:
                pass


# -------------------------
# SliderGroup (dynamic adding)
# -------------------------
class SliderGroup(QGroupBox):
    def __init__(self, title, labels=None, slider_callback=None, name_changed_callback=None):
        super().__init__(title)
        self.setCheckable(True)
        self.setChecked(True)
        self.slider_callback = slider_callback
        self.name_changed_callback = name_changed_callback
        self.container = QVBoxLayout()
        self.sliders = []
        if labels:
            for lbl in labels:
                self.add_slider(lbl)
        self.setLayout(self.container)
        self.toggled.connect(self._on_toggled)

    def _on_toggled(self, checked):
        for s in self.sliders:
            s.setVisible(checked)

    def add_slider(self, name="Value", min_val=-10.0, max_val=10.0, value=0.0, name_changed_callback=None):
        # name_changed_callback overrides group default if provided
        callback = name_changed_callback if name_changed_callback is not None else self.name_changed_callback
        s = slideBar(name, min_val, max_val, value, callback=self.slider_callback, name_changed_callback=callback)
        self.container.addWidget(s)
        self.sliders.append(s)
        return s

    def clear(self):
        for s in self.sliders:
            try:
                s.deleteLater()
            except Exception:
                pass
        self.sliders.clear()


# -------------------------
# Type-specific widgets
# -------------------------
class Vec3Widget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout(self)
        self.group = SliderGroup("Vec3", ["x", "y", "z"], slider_callback)
        layout.addWidget(self.group)


class PoseWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout(self)
        self.position = SliderGroup("Position", ["x", "y", "z"], slider_callback)
        self.orientation = SliderGroup("Orientation", ["w", "x", "y", "z"], slider_callback)
        layout.addWidget(self.position)
        layout.addWidget(self.orientation)


class TwistWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout(self)
        self.linear = SliderGroup("Linear", ["x", "y", "z"], slider_callback)
        self.angular = SliderGroup("Angular", ["x", "y", "z"], slider_callback)
        layout.addWidget(self.linear)
        layout.addWidget(self.angular)


class FloatArrayWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.slider_callback = slider_callback
        self.sliders = []

    def add_slider(self, name="Gain", min_val=-10.0, max_val=10.0, value=0.0):
        s = slideBar(name, min_val, max_val, value, callback=self.slider_callback)
        self.layout.addWidget(s)
        self.sliders.append(s)
        return s

    def clear(self):
        for s in list(self.sliders):
            try:
                s.deleteLater()
            except Exception:
                pass
        self.sliders.clear()


class JointStateWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout(self)

        # use SliderGroup internally but with no initial labels
        self.pos_group = SliderGroup("Position", labels=None, slider_callback=slider_callback,
                                     name_changed_callback=self._on_name_changed)
        self.vel_group = SliderGroup("Velocity", labels=None, slider_callback=slider_callback,
                                     name_changed_callback=self._on_name_changed)
        self.eff_group = SliderGroup("Effort", labels=None, slider_callback=slider_callback,
                                     name_changed_callback=self._on_name_changed)

        layout.addWidget(self.pos_group)
        layout.addWidget(self.vel_group)
        layout.addWidget(self.eff_group)

        # aligned lists
        self.position_sliders = []
        self.velocity_sliders = []
        self.effort_sliders = []

    def add_joint(self, base_name=None, min_val=-10.0, max_val=10.0, value=0.0):
        idx = len(self.position_sliders)
        default_name = base_name or f"joint_{idx + 1}"

        # Important: pass the same name_changed_callback to each slideBar so changing name in one triggers sync
        pos = self.pos_group.add_slider(default_name, min_val, max_val, value, name_changed_callback=self._on_name_changed)
        vel = self.vel_group.add_slider(default_name, min_val, max_val, value, name_changed_callback=self._on_name_changed)
        eff = self.eff_group.add_slider(default_name, min_val, max_val, value, name_changed_callback=self._on_name_changed)

        self.position_sliders.append(pos)
        self.velocity_sliders.append(vel)
        self.effort_sliders.append(eff)

    def _on_name_changed(self, source_slider):
        # Locate source in any of the three lists and propagate the name to the same index in others
        lists = (self.position_sliders, self.velocity_sliders, self.effort_sliders)
        for group in lists:
            if source_slider in group:
                idx = group.index(source_slider)
                new_name = source_slider.name_label.text()
                # update same index in all lists
                for other in lists:
                    if idx < len(other):
                        target = other[idx]
                        if target is not source_slider:
                            target.name_label.setText(new_name)
                break

    def clear(self):
        for s in (self.position_sliders + self.velocity_sliders + self.effort_sliders):
            try:
                s.deleteLater()
            except Exception:
                pass
        self.position_sliders.clear()
        self.velocity_sliders.clear()
        self.effort_sliders.clear()
        self.pos_group.clear()
        self.vel_group.clear()
        self.eff_group.clear()


# -------------------------
# DynamicContent (per-tab)
# -------------------------
class DynamicContent(QWidget):
    def __init__(self):
        super().__init__()
        # each tab has its own LCM and message object (you may want shared LCM; using per-tab as before)
        self.lc = lcm.LCM()
        self.lcm_msg = FloatMultiArray()

        self.msg_type = "Float32MultiArray"

        # layout
        main = QVBoxLayout(self)
        main.setSpacing(4)
        main.setContentsMargins(10, 8, 10, 8)

        # --- Top Row: Topic + Type ---
        top = QHBoxLayout()
        top.setSpacing(6)
        top.setContentsMargins(0, 0, 0, 0)
        # top.setAlignment(Qt.AlignLeft)

        top.addWidget(QLabel("Topic:"))
        self.topic_edit = QLineEdit("robot/publish_bar")
        self.topic_edit.setFixedWidth(320)
        top.addWidget(self.topic_edit)

        top.addWidget(QLabel("Message Type:"))
        self.type_box = QComboBox()
        self.type_box.addItems(["Float32MultiArray", "Twist", "Vec3", "Pose", "JointState"])
        self.type_box.currentTextChanged.connect(self._on_type_changed)
        self.type_box.setFixedWidth(180)
        top.addWidget(self.type_box)

        top.addStretch(1)
        main.addLayout(top, stretch=0)

        # --- Publish Mode Row ---
        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        mode_row.setContentsMargins(0, 0, 0, 0)
        # mode_row.setAlignment(Qt.AlignLeft)

        mode_row.addWidget(QLabel("Publish Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["As Updates", "Continuous", "Once"])
        self.mode_box.currentTextChanged.connect(self.on_mode_changed)
        self.mode_box.setFixedWidth(150)
        mode_row.addWidget(self.mode_box)

        self.freq_min_edit = QLineEdit("1");
        self.freq_min_edit.setFixedWidth(50)
        self.freq_max_edit = QLineEdit("100");
        self.freq_max_edit.setFixedWidth(50)
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setRange(1, 100)
        self.freq_slider.setValue(10)
        self.freq_slider.setFixedWidth(250)
        self.freq_slider.setTickPosition(QSlider.TicksBelow)
        self.freq_slider.setTickInterval(1)
        self.freq_value_label = QLabel("10 Hz")

        self.freq_slider.valueChanged.connect(self.on_freq_slider_changed)
        self.freq_min_edit.editingFinished.connect(self.on_freq_minmax_changed)
        self.freq_max_edit.editingFinished.connect(self.on_freq_minmax_changed)

        mode_row.addWidget(QLabel("Min:"));
        mode_row.addWidget(self.freq_min_edit)
        mode_row.addWidget(QLabel("Max:"));
        mode_row.addWidget(self.freq_max_edit)
        mode_row.addWidget(self.freq_slider)
        mode_row.addWidget(self.freq_value_label)
        mode_row.addStretch(1)
        main.addLayout(mode_row, stretch=0)

        # --- Buttons Row ---
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        btn_row.setContentsMargins(0, 0, 0, 0)
        btn_row.setAlignment(Qt.AlignLeft)

        self.add_slider_button = QPushButton("Add Slider")
        self.add_slider_button.setFixedWidth(100)
        self.add_slider_button.clicked.connect(self.add_slider)

        self.publish_button = QPushButton("Publish")
        self.publish_button.setFixedWidth(100)
        self.publish_button.clicked.connect(self.manual_publish)

        btn_row.addWidget(self.add_slider_button)
        btn_row.addWidget(self.publish_button)
        btn_row.addStretch(1)
        main.addLayout(btn_row, stretch=0)

        # --- Dynamic Area ---
        ## no spacing no cllaps for groups
        self.dynamic_area = QVBoxLayout()
        self.dynamic_area.setContentsMargins(0, 0, 0, 0)
        self.dynamic_area.setSpacing(4)
        main.addLayout(self.dynamic_area, stretch=1)

        ## callaps and huge space
        # self.dynamic_area = QVBoxLayout()
        # main.addLayout(self.dynamic_area)

        # internal references to type widgets (created in switch)
        self.type_widget = None

        # timer per-tab for Continuous mode
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish)
        self.publish_timer.setInterval(int(1000 / 10))  # default 10 Hz

        # initialize UI
        self.show_freq_controls(False)
        self._on_type_changed(self.type_box.currentText())

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

        # add to layout
        if self.type_widget:
            self.dynamic_area.addWidget(self.type_widget)

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
            self.publish()

    # ---------------- publish logic (example prints + LCM publish for FloatArray) ----------------
    def manual_publish(self):
        if self.mode_box.currentText() == "Once":
            self.publish()

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
            except Exception:
                pass
        elif t == "Vec3" and isinstance(self.type_widget, Vec3Widget):
            vals = [s.get_value() for s in self.type_widget.group.sliders]
            print(f"[Publish] Vec3 -> {topic} : {vals}")
        elif t == "Pose" and isinstance(self.type_widget, PoseWidget):
            pos = [s.get_value() for s in self.type_widget.position.sliders]
            ori = [s.get_value() for s in self.type_widget.orientation.sliders]
            print(f"[Publish] Pose -> {topic}")
            print("  Pos:", pos, "Ori:", ori)
        elif t == "Twist" and isinstance(self.type_widget, TwistWidget):
            lin = [s.get_value() for s in self.type_widget.linear.sliders]
            ang = [s.get_value() for s in self.type_widget.angular.sliders]
            print(f"[Publish] Twist -> {topic}")
            print("  Linear:", lin, "  Angular:", ang)
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

# -------------------------
# Tabbed main window (manages multiple tabs)
# -------------------------
class TabbedMain(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tabbed Publisher GUI")
        self.resize(1000, 700)

        layout = QVBoxLayout(self)

        # control buttons
        control_row = QHBoxLayout()
        self.add_tab_btn = QPushButton("➕ Add Tab")
        self.add_tab_btn.clicked.connect(self.add_tab)
        self.save_btn = QPushButton("💾 Save All")
        self.save_btn.clicked.connect(self.save_config)
        self.load_btn = QPushButton("📂 Load All")
        self.load_btn.clicked.connect(self.load_config)
        control_row.addWidget(self.add_tab_btn)
        control_row.addWidget(self.save_btn)
        control_row.addWidget(self.load_btn)
        control_row.addStretch()
        layout.addLayout(control_row)

        # tabs
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self.close_tab)
        self.tabs.tabBarDoubleClicked.connect(self.rename_tab)
        layout.addWidget(self.tabs)

        # start with one tab
        self.add_tab()

    def add_tab(self, state: dict = None, title: str = None):
        tab = DynamicContent()
        idx = self.tabs.addTab(tab, title or f"Tab {self.tabs.count() + 1}")
        self.tabs.setCurrentIndex(idx)
        if state:
            tab.load_state(state)

    def close_tab(self, index):
        self.tabs.removeTab(index)

    def rename_tab(self, index):
        if index < 0:
            return
        old = self.tabs.tabText(index)
        new_name, ok = QInputDialog.getText(self, "Rename Tab", "Enter new name:", text=old)
        if ok and new_name.strip():
            self.tabs.setTabText(index, new_name)

    # ---------------- Save / Load all tabs ----------------
    def save_config(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save All Tabs", "", "JSON Files (*.json)")
        if not filename:
            return
        data = {"tabs": []}
        for i in range(self.tabs.count()):
            widget = self.tabs.widget(i)
            tab_name = self.tabs.tabText(i)
            try:
                state = widget.save_state()
            except Exception:
                state = {}
            data["tabs"].append({"name": tab_name, "state": state})
        with open(filename, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[Saved All Tabs] {filename}")

    def load_config(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load All Tabs", "", "JSON Files (*.json)")
        if not filename:
            return
        with open(filename, "r") as f:
            data = json.load(f)
        # clear existing tabs
        self.tabs.clear()
        for t in data.get("tabs", []):
            name = t.get("name", None)
            state = t.get("state", {})
            self.add_tab(state=state, title=name or None)
        print(f"[Loaded All Tabs] {filename}")


# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TabbedMain()
    win.show()
    sys.exit(app.exec())
