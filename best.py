import sys
import json
import lcm
from PySide6.QtCore import Qt, QTimer

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QLineEdit, QFileDialog,
    QComboBox, QGroupBox, QSpacerItem, QSizePolicy
)

from lcm_msg.lcm_std import FloatMultiArray
# from best import slideBar  # existing slideBar (assumed to support name_label and name_changed_callback)

class slideBar(QWidget):
    def __init__(self, name="Gain", min_val=0.0, max_val=100.0, value=50.0,
                 callback=None, name_changed_callback=None):
        super().__init__()
        self.layout = QHBoxLayout()
        self.name_label = QLineEdit(str(name))
        self.name_label.setFixedWidth(80)
        self.name_label.setStyleSheet("color: brown;")

        self.min_edit = QLineEdit(str(min_val))
        self.min_edit.setFixedWidth(60)
        self.max_edit = QLineEdit(str(max_val))
        self.max_edit.setFixedWidth(60)

        self.value_label = QLabel(f"{value:.2f}")
        self.value_label.setStyleSheet("color: darkred;")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(int((value - min_val) / (max_val - min_val) * 1000))

        # Layout arrangement
        self.layout.addWidget(self.name_label)
        self.layout.addWidget(self.min_edit)
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.max_edit)
        spacer = QSpacerItem(10, 10, QSizePolicy.Fixed, QSizePolicy.Minimum)
        self.layout.addItem(spacer)
        self.layout.addWidget(self.value_label)
        self.setLayout(self.layout)

        # --- Callbacks ---
        self.callback = callback
        self.name_changed_callback = name_changed_callback

        self.slider.valueChanged.connect(self.on_change)
        self.min_edit.editingFinished.connect(self.on_change)
        self.max_edit.editingFinished.connect(self.on_change)

        if self.name_changed_callback:
            self.name_label.editingFinished.connect(self.on_name_changed)

        self.update_value_label(callback)

    def get_value(self):
        min_val = float(self.min_edit.text())
        max_val = float(self.max_edit.text())
        return min_val + (max_val - min_val) * self.slider.value() / 1000.0

    def update_value_label(self, callback=None):
        value = self.get_value()
        self.value_label.setText(f"{value:.2f}")
        # if callback:
        #     callback()

    def on_change(self):
        self.update_value_label()
        if self.callback:
            self.callback()

    def on_name_changed(self):
        if self.name_changed_callback:
            self.name_changed_callback(self)


# ---------------------- Reusable Components ----------------------

class SliderGroup(QGroupBox):
    """Generic group of slideBar widgets (uses your existing slideBar)."""
    def __init__(self, title, labels, slider_callback=None, name_changed_callback=None):
        super().__init__(title)
        self.setCheckable(True)
        self.setChecked(True)
        layout = QVBoxLayout()
        self.sliders = []
        for lbl in labels:
            # Pass name_changed_callback if slideBar supports it
            s = slideBar(lbl, -10.0, 10.0, 0.0, callback=slider_callback,
                         name_changed_callback=name_changed_callback) \
                if name_changed_callback else slideBar(lbl, -10.0, 10.0, 0.0, callback=slider_callback)
            layout.addWidget(s)
            self.sliders.append(s)
        self.setLayout(layout)
        # toggle visibility of contained sliders when group is toggled
        self.toggled.connect(lambda checked: [s.setVisible(checked) for s in self.sliders])


# ---------------------- Mode Widgets ----------------------

class Vec3Widget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout()
        self.group = SliderGroup("Vector3", ["x", "y", "z"], slider_callback)
        layout.addWidget(self.group)
        self.setLayout(layout)


class PoseWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout()
        self.position = SliderGroup("Position", ["x", "y", "z"], slider_callback)
        self.orientation = SliderGroup("Orientation", ["w", "x", "y", "z"], slider_callback)
        layout.addWidget(self.position)
        layout.addWidget(self.orientation)
        self.setLayout(layout)


class TwistWidget(QWidget):
    def __init__(self, slider_callback):
        super().__init__()
        layout = QVBoxLayout()
        self.linear = SliderGroup("Linear", ["x", "y", "z"], slider_callback)
        self.angular = SliderGroup("Angular", ["x", "y", "z"], slider_callback)
        layout.addWidget(self.linear)
        layout.addWidget(self.angular)
        self.setLayout(layout)


class FloatArrayWidget(QWidget):
    """Container for Float32MultiArray sliders (dynamic add/remove)."""
    def __init__(self, slider_callback):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.slider_callback = slider_callback
        self.sliders = []

    def add_slider(self, name="Gain", min_val=-10.0, max_val=10.0, value=0.0):
        s = slideBar(name, min_val, max_val, value, callback=self.slider_callback,
                     name_changed_callback=None)
        self.layout.addWidget(s)
        self.sliders.append(s)
        return s

    def clear(self):
        while self.sliders:
            s = self.sliders.pop()
            s.deleteLater()


class JointStateWidget(QWidget):
    """
    JointState consists of three parallel groups (Position, Velocity, Effort).
    Each time we add a joint slider we create one slider per group in the same index.
    Name changes in any slider are propagated to the corresponding sliders in other groups.
    """
    def __init__(self, slider_callback):
        super().__init__()
        self.slider_callback = slider_callback

        self.pos_group = QGroupBox("Position")
        self.pos_group.setCheckable(True); self.pos_group.setChecked(True)
        self.pos_layout = QVBoxLayout(); self.pos_group.setLayout(self.pos_layout)

        self.vel_group = QGroupBox("Velocity")
        self.vel_group.setCheckable(True); self.vel_group.setChecked(True)
        self.vel_layout = QVBoxLayout(); self.vel_group.setLayout(self.vel_layout)

        self.eff_group = QGroupBox("Effort")
        self.eff_group.setCheckable(True); self.eff_group.setChecked(True)
        self.eff_layout = QVBoxLayout(); self.eff_group.setLayout(self.eff_layout)

        # Lists of sliders aligned by index: index i => position[i], velocity[i], effort[i]
        self.position_sliders = []
        self.velocity_sliders = []
        self.effort_sliders = []

        # toggle visibility when group toggled
        self.pos_group.toggled.connect(lambda checked: [s.setVisible(checked) for s in self.position_sliders])
        self.vel_group.toggled.connect(lambda checked: [s.setVisible(checked) for s in self.velocity_sliders])
        self.eff_group.toggled.connect(lambda checked: [s.setVisible(checked) for s in self.effort_sliders])

        # assemble layout
        layout = QVBoxLayout()
        layout.addWidget(self.pos_group)
        layout.addWidget(self.vel_group)
        layout.addWidget(self.eff_group)
        self.setLayout(layout)

    def add_joint(self, base_name=None, min_val=-10.0, max_val=10.0, value=0.0):
        """
        Adds one joint slider to each group. base_name is used for initial name label.
        slideBar must support a name_changed_callback argument and expose name_label attribute.
        """
        idx = len(self.position_sliders)
        default_name = base_name if base_name else f"joint_{idx + 1}"

        # Create position slider with name_changed_callback that syncs across groups
        pos_slider = slideBar(default_name, min_val, max_val, value,
                              callback=self.slider_callback,
                              name_changed_callback=self._on_name_changed)
        vel_slider = slideBar(default_name, min_val, max_val, value,
                              callback=self.slider_callback,
                              name_changed_callback=self._on_name_changed)
        eff_slider = slideBar(default_name, min_val, max_val, value,
                              callback=self.slider_callback,
                              name_changed_callback=self._on_name_changed)

        # Add to layouts and lists
        self.pos_layout.addWidget(pos_slider)
        self.vel_layout.addWidget(vel_slider)
        self.eff_layout.addWidget(eff_slider)

        self.position_sliders.append(pos_slider)
        self.velocity_sliders.append(vel_slider)
        self.effort_sliders.append(eff_slider)

    def _on_name_changed(self, source_slider):
        """
        Keep names in sync across the three groups.
        Called by slideBar via name_changed_callback with the source slider object.
        """
        # Try to find source in any of the lists
        for list_group in (self.position_sliders, self.velocity_sliders, self.effort_sliders):
            if source_slider in list_group:
                idx = list_group.index(source_slider)
                new_name = source_slider.name_label.text() if hasattr(source_slider, "name_label") else None
                if new_name is None:
                    return
                # Update all groups at that index (if present), skipping the source itself
                for other in (self.position_sliders, self.velocity_sliders, self.effort_sliders):
                    if idx < len(other):
                        slider_to_update = other[idx]
                        if slider_to_update is not source_slider and hasattr(slider_to_update, "name_label"):
                            slider_to_update.name_label.setText(new_name)
                break

    def clear(self):
        # remove widgets
        for s in (self.position_sliders + self.velocity_sliders + self.effort_sliders):
            try:
                s.deleteLater()
            except Exception:
                pass
        self.position_sliders.clear()
        self.velocity_sliders.clear()
        self.effort_sliders.clear()


# ---------------------- Dynamic Content (central) ----------------------

class DynamicContent(QWidget):
    """
    Central switching widget. Does NOT contain the type combobox anymore —
    the type combobox belongs to MainWindow and is placed next to topic name.
    """
    def __init__(self, parent):
        super().__init__()
        self.parent = parent  # MainWindow
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.current_widget = None
        # initialize default
        self.switch_type("Float32MultiArray")

    def switch_type(self, msg_type):
        # remove existing
        if self.current_widget:
            self.layout.removeWidget(self.current_widget)
            self.current_widget.deleteLater()
            self.current_widget = None

        self.parent.msg_type = msg_type
        self.parent.add_slider_button.setEnabled(msg_type in ["Float32MultiArray", "JointState"])

        if msg_type == "Vec3":
            self.current_widget = Vec3Widget(self.parent.slider_changed)
        elif msg_type == "Pose":
            self.current_widget = PoseWidget(self.parent.slider_changed)
        elif msg_type == "Twist":
            self.current_widget = TwistWidget(self.parent.slider_changed)
        elif msg_type == "JointState":
            self.current_widget = JointStateWidget(self.parent.slider_changed)
        else:  # Float32MultiArray
            self.current_widget = FloatArrayWidget(self.parent.slider_changed)

        self.layout.addWidget(self.current_widget)


# ---------------------- Main Window ----------------------

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Publish Bar LCM")
        self.resize(700, 520)

        # LCM and message
        self.lc = lcm.LCM()
        self.lcm_msg = FloatMultiArray()
        self.msg_type = "Float32MultiArray"

        # internal timer
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish)
        self.publish_timer.setInterval(100)  # 10 Hz default

        # Build UI (topic + type on same row)
        self._build_ui()

    def _build_ui(self):
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # Topic name + message type in the SAME row (fixed per your request)
        top_row = QHBoxLayout()
        top_row.addWidget(QLabel("Topic Name:"))
        self.topic_edit = QLineEdit("robot/publish_bar")
        top_row.addWidget(self.topic_edit)

        top_row.addWidget(QLabel("Message Type:"))
        self.type_box = QComboBox()
        self.type_box.addItems(["Float32MultiArray", "Twist", "Vec3", "Pose", "JointState"])
        # connect type change to dynamic content switch
        self.type_box.currentTextChanged.connect(self._on_type_changed)
        top_row.addWidget(self.type_box)

        top_row.addStretch()
        main_layout.addLayout(top_row)

        # Publish mode + frequency controls (min/max + slider + label)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Publish Mode:"))

        self.mode_box = QComboBox()
        self.mode_box.addItems(["As Updates", "Continuous", "Once"])
        self.mode_box.currentTextChanged.connect(self.on_mode_changed)
        self.mode_box.setMaximumWidth(180)
        mode_row.addWidget(self.mode_box)

        self.freq_min_edit = QLineEdit("1"); self.freq_min_edit.setFixedWidth(40)
        self.freq_max_edit = QLineEdit("100"); self.freq_max_edit.setFixedWidth(40)
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setRange(1, 100)
        self.freq_slider.setValue(10)
        self.freq_slider.setTickInterval(5)
        self.freq_slider.setTickPosition(QSlider.TicksBelow)
        self.freq_slider.setFixedWidth(150)
        self.freq_value_label = QLabel("10 Hz")

        # Connect freq controls
        self.freq_slider.valueChanged.connect(self.on_freq_changed)
        self.freq_min_edit.editingFinished.connect(self.update_freq_range)
        self.freq_max_edit.editingFinished.connect(self.update_freq_range)

        mode_row.addWidget(QLabel("Min:"))
        mode_row.addWidget(self.freq_min_edit)
        mode_row.addWidget(QLabel("Max:"))
        mode_row.addWidget(self.freq_max_edit)
        mode_row.addWidget(self.freq_slider)
        mode_row.addWidget(self.freq_value_label)
        mode_row.addStretch()
        main_layout.addLayout(mode_row)

        # Buttons row
        buttons_row = QHBoxLayout()
        self.save_button = QPushButton("Save Config"); self.save_button.setFixedWidth(120)
        self.save_button.clicked.connect(self.save_config)
        self.load_button = QPushButton("Load Config"); self.load_button.setFixedWidth(120)
        self.load_button.clicked.connect(self.load_config)
        self.add_slider_button = QPushButton("Add Slider"); self.add_slider_button.setFixedWidth(120)
        self.add_slider_button.clicked.connect(self.add_slider)
        self.publish_button = QPushButton("Publish"); self.publish_button.setFixedWidth(120)
        self.publish_button.clicked.connect(self.manual_publish)

        for b in (self.save_button, self.load_button, self.add_slider_button, self.publish_button):
            buttons_row.addWidget(b)
        buttons_row.addStretch()
        main_layout.addLayout(buttons_row)

        # Dynamic content (below the controls)
        self.dynamic_content = DynamicContent(self)
        main_layout.addWidget(self.dynamic_content)

        # Set initial visibility for frequency controls
        self.show_freq_controls(False)
        # ensure dynamic content matches type combobox initial state
        self._on_type_changed(self.type_box.currentText())

    # ---------------------- Type handling ----------------------

    def _on_type_changed(self, msg_type):
        # Pass type change through to dynamic content
        self.msg_type = msg_type
        self.dynamic_content.switch_type(msg_type)

    # ---------------------- Mode / Frequency ----------------------

    def on_mode_changed(self, mode):
        if mode == "Continuous":
            self.publish_timer.start()
            self.publish_button.setEnabled(False)
            self.show_freq_controls(True)
        elif mode == "As Updates":
            self.publish_timer.stop()
            self.publish_button.setEnabled(False)
            self.show_freq_controls(False)
        elif mode == "Once":
            self.publish_timer.stop()
            self.publish_button.setEnabled(True)
            self.show_freq_controls(False)

    def show_freq_controls(self, show):
        for w in [self.freq_min_edit, self.freq_max_edit, self.freq_slider, self.freq_value_label]:
            w.setVisible(show)

    def update_freq_range(self):
        try:
            min_val = int(self.freq_min_edit.text())
            max_val = int(self.freq_max_edit.text())
            if min_val < 1 or max_val <= min_val:
                raise ValueError
        except ValueError:
            print("[Error] Invalid min/max frequency range.")
            return
        self.freq_slider.setRange(min_val, max_val)
        self.on_freq_changed()

    def on_freq_changed(self):
        freq = self.freq_slider.value()
        self.freq_value_label.setText(f"{freq} Hz")
        # avoid division by zero; freq guaranteed >= 1 by validation
        self.publish_timer.setInterval(int(1000 / freq))

    # ---------------------- Slider events / Add slider ----------------------

    def slider_changed(self):
        """Called by slideBar when values change."""
        if self.mode_box.currentText() == "As Updates":
            self.publish()

    def add_slider(self):
        """Add slider depending on current message type."""
        msg_type = self.msg_type
        if msg_type == "Float32MultiArray":
            if isinstance(self.dynamic_content.current_widget, FloatArrayWidget):
                # name uses count+1 to match previous behavior
                name = f"Gain {len(self.dynamic_content.current_widget.sliders) + 1}"
                self.dynamic_content.current_widget.add_slider(name)
        elif msg_type == "JointState":
            # Add a joint across Position/Velocity/Effort with synchronized name callback
            js_widget = self.dynamic_content.current_widget
            if isinstance(js_widget, JointStateWidget):
                # Add joint with default name; the JointStateWidget registers name_changed callbacks
                js_widget.add_joint(base_name=f"joint_{len(js_widget.position_sliders) + 1}")

    def manual_publish(self):
        if self.mode_box.currentText() == "Once":
            self.publish()

    # ---------------------- Publish ----------------------

    def publish(self):
        topic = self.topic_edit.text()
        msg_type = self.msg_type
        print(f"[Publish] Type={msg_type} Topic={topic}")

        if msg_type == "Float32MultiArray" and isinstance(self.dynamic_content.current_widget, FloatArrayWidget):
            values = [s.get_value() for s in self.dynamic_content.current_widget.sliders]
            names = [s.name_label.text() if hasattr(s, "name_label") else "" for s in self.dynamic_content.current_widget.sliders]
            print("  Names :", names)
            print("  Values:", values)
            self.lcm_msg.data = values
            self.lcm_msg.size = len(values)
            self.lc.publish(topic, self.lcm_msg.encode())

        elif msg_type == "Vec3" and isinstance(self.dynamic_content.current_widget, Vec3Widget):
            vals = [s.get_value() for s in self.dynamic_content.current_widget.group.sliders]
            print("  Vec3 Values:", vals)

        elif msg_type == "Pose" and isinstance(self.dynamic_content.current_widget, PoseWidget):
            pos = [s.get_value() for s in self.dynamic_content.current_widget.position.sliders]
            ori = [s.get_value() for s in self.dynamic_content.current_widget.orientation.sliders]
            print("  Position:", pos)
            print("  Orientation:", ori)

        elif msg_type == "Twist" and isinstance(self.dynamic_content.current_widget, TwistWidget):
            lin = [s.get_value() for s in self.dynamic_content.current_widget.linear.sliders]
            ang = [s.get_value() for s in self.dynamic_content.current_widget.angular.sliders]
            print("  Linear :", lin)
            print("  Angular:", ang)

        elif msg_type == "JointState" and isinstance(self.dynamic_content.current_widget, JointStateWidget):
            pos_vals = [s.get_value() for s in self.dynamic_content.current_widget.position_sliders]
            vel_vals = [s.get_value() for s in self.dynamic_content.current_widget.velocity_sliders]
            eff_vals = [s.get_value() for s in self.dynamic_content.current_widget.effort_sliders]
            names = [s.name_label.text() if hasattr(s, "name_label") else "" for s in self.dynamic_content.current_widget.position_sliders]
            print("  JointState Names:", names)
            print("  Position:", pos_vals)
            print("  Velocity:", vel_vals)
            print("  Effort:", eff_vals)

        else:
            # fallback: try to print any child slideBar get_value() found
            widget = self.dynamic_content.current_widget
            values = []
            if hasattr(widget, "layout"):
                for i in range(widget.layout.count()):
                    w = widget.layout.itemAt(i).widget()
                    if hasattr(w, "get_value"):
                        values.append(w.get_value())
            if values:
                print("  Values:", values)

    # ---------------------- Save / Load Config ----------------------

    def save_config(self):
        """Save full GUI state (topic, msg_type, sliders, publish mode, frequency) to JSON."""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Config", "", "JSON Files (*.json)")
        if not filename:
            return

        def get_slider_data(s):
            """Safely extract slider info from a slideBar widget."""
            try:
                return {
                    "name": s.name_label.text(),
                    "min": float(s.min_edit.text()),
                    "max": float(s.max_edit.text()),
                    "value": s.get_value()
                }
            except Exception:
                return {"name": "unknown", "min": -10.0, "max": 10.0, "value": 0.0}

        data = {
            "topic_name": self.topic_edit.text(),
            "msg_type": self.msg_type,
            "publish_mode": self.mode_box.currentText(),
            "freq_min": self.freq_min_edit.text(),
            "freq_max": self.freq_max_edit.text(),
            "freq_value": self.freq_slider.value(),
            "groups": []
        }

        widget = self.dynamic_content.current_widget

        # --- Float32MultiArray ---
        if isinstance(widget, FloatArrayWidget):
            data["groups"].append({
                "name": "Values",
                "sliders": [get_slider_data(s) for s in widget.sliders]
            })

        # --- Vec3 ---
        elif isinstance(widget, Vec3Widget):
            data["groups"].append({
                "name": "Vec3",
                "sliders": [get_slider_data(s) for s in widget.group.sliders]
            })

        # --- Pose ---
        elif isinstance(widget, PoseWidget):
            for gname, group in [("Position", widget.position), ("Orientation", widget.orientation)]:
                data["groups"].append({
                    "name": gname,
                    "sliders": [get_slider_data(s) for s in group.sliders]
                })

        # --- Twist ---
        elif isinstance(widget, TwistWidget):
            for gname, group in [("Linear", widget.linear), ("Angular", widget.angular)]:
                data["groups"].append({
                    "name": gname,
                    "sliders": [get_slider_data(s) for s in group.sliders]
                })

        # --- JointState ---
        elif isinstance(widget, JointStateWidget):
            for i in range(len(widget.position_sliders)):
                entry = {
                    "name": widget.position_sliders[i].name_label.text(),
                    "position": get_slider_data(widget.position_sliders[i]),
                    "velocity": get_slider_data(widget.velocity_sliders[i]),
                    "effort": get_slider_data(widget.effort_sliders[i])
                }
                data["groups"].append(entry)

        # --- Save JSON file ---
        with open(filename, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[Save] Configuration saved to {filename}")

    def load_config(self):
        """Load full GUI state from JSON and restore layout and values."""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "JSON Files (*.json)")
        if not filename:
            return

        with open(filename, "r") as f:
            data = json.load(f)

        # --- Restore header info ---
        self.topic_edit.setText(data.get("topic_name", "robot/publish_bar"))
        msg_type = data.get("msg_type", "Float32MultiArray")
        self.type_box.setCurrentText(msg_type)
        self.mode_box.setCurrentText(data.get("publish_mode", "As Updates"))

        self.freq_min_edit.setText(str(data.get("freq_min", 1)))
        self.freq_max_edit.setText(str(data.get("freq_max", 100)))
        self.freq_slider.setValue(int(data.get("freq_value", 10)))
        self.update_freq_range()
        self.on_freq_changed()

        # Refresh UI
        self._on_type_changed(msg_type)
        widget = self.dynamic_content.current_widget

        # Clear old sliders
        if hasattr(widget, "clear"):
            widget.clear()

        def restore_slider(s, sdata):
            """Restore single slider values."""
            if "name" in sdata:
                s.name_label.setText(sdata["name"])
            if "min" in sdata:
                s.min_edit.setText(str(sdata["min"]))
            if "max" in sdata:
                s.max_edit.setText(str(sdata["max"]))
            if "value" in sdata:
                s.slider.setValue(int((float(sdata["value"]) - float(s.min_edit.text())) /
                                      (float(s.max_edit.text()) - float(s.min_edit.text())) * 1000))
                s.update_value_label()

        # --- Float32MultiArray ---
        if isinstance(widget, FloatArrayWidget):
            for sdata in data["groups"][0]["sliders"]:
                widget.add_slider(sdata["name"], sdata["min"], sdata["max"], sdata["value"])

        # --- Vec3 ---
        elif isinstance(widget, Vec3Widget):
            sliders_data = data["groups"][0]["sliders"]
            for s, sdata in zip(widget.group.sliders, sliders_data):
                restore_slider(s, sdata)

        # --- Pose ---
        elif isinstance(widget, PoseWidget):
            for g in data["groups"]:
                if g["name"].lower() == "position":
                    for s, sdata in zip(widget.position.sliders, g["sliders"]):
                        restore_slider(s, sdata)
                elif g["name"].lower() == "orientation":
                    for s, sdata in zip(widget.orientation.sliders, g["sliders"]):
                        restore_slider(s, sdata)

        # --- Twist ---
        elif isinstance(widget, TwistWidget):
            for g in data["groups"]:
                if g["name"].lower() == "linear":
                    for s, sdata in zip(widget.linear.sliders, g["sliders"]):
                        restore_slider(s, sdata)
                elif g["name"].lower() == "angular":
                    for s, sdata in zip(widget.angular.sliders, g["sliders"]):
                        restore_slider(s, sdata)

        # --- JointState ---
        elif isinstance(widget, JointStateWidget):
            for joint in data["groups"]:
                name = joint.get("name", "joint")
                widget.add_joint(base_name=name)
                idx = len(widget.position_sliders) - 1
                for s, sdata in zip(
                        [widget.position_sliders[idx],
                         widget.velocity_sliders[idx],
                         widget.effort_sliders[idx]],
                        [joint.get("position", {}),
                         joint.get("velocity", {}),
                         joint.get("effort", {})]
                ):
                    restore_slider(s, sdata)

        print(f"[Load] Configuration loaded from {filename}")

# ---------------------- RUN APP ----------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
