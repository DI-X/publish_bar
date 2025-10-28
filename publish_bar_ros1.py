# pip install PyQt5 PyQt5-tools
# pip install PySide6

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QLineEdit, QFileDialog
)
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QSpacerItem, QSizePolicy
import json, sys
import rospy
from std_msgs.msg import Float32MultiArray

class slideBar(QWidget):
    def __init__(self, name="Gain", min_val=0.0, max_val=100.0, value=50.0, callback=None):
        super().__init__()
        self.layout = QHBoxLayout()
        self.name_label = QLineEdit(str(name))
        self.name_label.setFixedWidth(80)
        self.name_label.setStyleSheet("color: brawn;")

        self.min_edit = QLineEdit(str(min_val))
        self.min_edit.setFixedWidth(80)

        self.max_edit = QLineEdit(str(max_val))
        self.max_edit.setFixedWidth(80)

        self.value_label = QLabel(f"{value:.2f}")
        self.value_label.setStyleSheet("color: darkred;")
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        # Make it bold
        # font = QFont()
        # font.setBold(True)
        # self.value_label.setFont(font)


        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(int((value - min_val) / (max_val - min_val) * 1000))
        self.layout.addWidget(self.name_label)
        self.layout.addWidget(self.min_edit)
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.max_edit)
        # Add horizontal spacer (stretchable space)
        spacer = QSpacerItem(10, 10, QSizePolicy.Fixed, QSizePolicy.Minimum)  # 10px space
        self.layout.addItem(spacer)
        self.layout.addWidget(self.value_label)
        self.setLayout(self.layout)

        # --- Connect signals ---
        self.callback = callback

        # --- Connect signals to update label AND call callback ---
        self.slider.valueChanged.connect(self.on_change)
        self.min_edit.editingFinished.connect(self.on_change)
        self.max_edit.editingFinished.connect(self.on_change)
        # Initial display
        self.update_value_label(callback)

    def get_value(self):
        min_val = float(self.min_edit.text())
        max_val = float(self.max_edit.text())
        return min_val + (max_val - min_val) * self.slider.value() / 1000.0

    def update_value_label(self, callback=None):
        value = self.get_value()
        self.value_label.setText(f"{value:.2f}")
        if callback:
            callback()

    def on_change(self):
        self.update_value_label()
        if self.callback:
            self.callback()

class publishBar(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Publish Bar")
        self.layout = QVBoxLayout()

        # --- Topic name horizontal layout ---
        topic_layout = QHBoxLayout()
        topic_layout.addWidget(QLabel("Topic Name:"))

        self.topic_edit = QLineEdit("robot/publish_bar")
        topic_layout.addWidget(self.topic_edit)  # editable field right next to label
        self.layout.addLayout(topic_layout)

        # --- Buttons horizontal layout ---
        buttons_layout = QHBoxLayout()

        self.save_button = QPushButton("Save Config")
        self.save_button.setFixedWidth(120)  # fixed width
        self.save_button.clicked.connect(self.save_config)
        buttons_layout.addWidget(self.save_button)

        self.load_button = QPushButton("Load Config")
        self.load_button.setFixedWidth(120)  # fixed width
        self.load_button.clicked.connect(self.load_config)
        buttons_layout.addWidget(self.load_button)

        self.add_slider_button = QPushButton("Add Slider")
        self.add_slider_button.setFixedWidth(120)
        self.add_slider_button.clicked.connect(self.add_slider)
        buttons_layout.addWidget(self.add_slider_button)

        buttons_layout.addStretch()  # push buttons to left side
        self.layout.addLayout(buttons_layout)

        self.sliders = []
        self.setLayout(self.layout)

        # --- Ros init ----
        rospy.init_node('publish_bar', anonymous=True)
        self.ros_topic_name = self.topic_edit.text()
        self.pub = rospy.Publisher(self.ros_topic_name, Float32MultiArray, queue_size=1)
        self.ros_msg = Float32MultiArray()

    def add_slider(self):
        slider = slideBar(f"Gain {len(self.sliders) + 1}", callback=self.publish)
        self.sliders.append(slider)
        self.layout.insertWidget(len(self.sliders), slider)

    def update_publisher(self):
        if self.ros_topic_name != self.topic_edit.text():
            self.ros_topic_name = self.topic_edit.text()
            self.pub = rospy.Publisher(self.ros_topic_name, Float32MultiArray, queue_size=1)

    def publish(self):
        topic = self.topic_edit.text()
        values = [s.get_value() for s in self.sliders]
        name = [s.name_label.text() for s in self.sliders]

        # --- publish lcm msg ---
        self.ros_msg.data = values
        self.update_publisher()
        self.pub.publish(self.ros_msg)

    def save_config(self):
        config = {
            "topic_name": self.topic_edit.text(),
            "sliders": []
        }
        for s in self.sliders:
            slider_data = {
                "min": float(s.min_edit.text()),
                "max": float(s.max_edit.text()),
                "value": s.get_value(),
                "name": s.name_label.text()
            }
            config["sliders"].append(slider_data)

        # Ask user for file name
        filename, _ = QFileDialog.getSaveFileName(self, "Save Config", "", "JSON Files (*.json)")
        if filename:
            with open(filename, "w") as f:
                json.dump(config, f, indent=4)
            print(f"Configuration saved to {filename}")

    def load_config(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "JSON Files (*.json)")
        if not filename:
            return

        with open(filename, "r") as f:
            config = json.load(f)

        # Clear existing sliders
        for s in self.sliders:
            self.layout.removeWidget(s)
            s.deleteLater()
        self.sliders.clear()

        # Set topic name
        self.topic_edit.setText(config.get("topic_name", "robot/pd_gains"))

        # Add sliders from config
        for s_data in config.get("sliders", []):
            slider = slideBar(
                name=s_data.get("name", "Gain"),
                min_val=s_data.get("min", 0.0),
                max_val=s_data.get("max", 100.0),
                value=s_data.get("value", 50.0),
                callback=self.publish
            )
            self.sliders.append(slider)
            self.layout.insertWidget(len(self.sliders), slider)

        print(f"Configuration loaded from {filename}")

app = QApplication(sys.argv)
window = publishBar()
window.show()
app.exec()
