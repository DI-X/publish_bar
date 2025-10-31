class DynamicContent(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.layout.setSpacing(6)
        self.layout.setContentsMargins(10, 10, 10, 10)

        # --- Topic Name and Type Row ---
        topic_layout = QHBoxLayout()
        topic_layout.setSpacing(6)
        topic_layout.setContentsMargins(0, 0, 0, 0)
        topic_layout.addWidget(QLabel("Topic Name:"))

        self.topic_edit = QLineEdit("robot/publish_bar")
        self.topic_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        topic_layout.addWidget(self.topic_edit)

        self.type_box = QComboBox()
        self.type_box.addItems(["Float32MultiArray", "Twist", "Vec3", "Pose", "JointState"])
        self.type_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.type_box.setMaximumWidth(200)
        topic_layout.addWidget(self.type_box)

        self.layout.addLayout(topic_layout)

        # --- Publish Mode + Frequency Row ---
        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(6)
        mode_layout.setContentsMargins(0, 0, 0, 0)

        mode_layout.addWidget(QLabel("Publish Mode:"))
        self.mode_box = QComboBox()
        self.mode_box.addItems(["As Updates", "Continuous", "Once"])
        self.mode_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.mode_box.setMaximumWidth(180)
        mode_layout.addWidget(self.mode_box)

        # Frequency widgets
        self.freq_min_edit = QLineEdit("1")
        self.freq_min_edit.setFixedWidth(40)
        self.freq_max_edit = QLineEdit("100")
        self.freq_max_edit.setFixedWidth(40)
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setMinimum(1)
        self.freq_slider.setMaximum(100)
        self.freq_slider.setValue(10)
        self.freq_slider.setFixedWidth(150)
        self.freq_value_label = QLabel("10 Hz")

        # Compact alignment for all
        mode_layout.addWidget(QLabel("Min:"))
        mode_layout.addWidget(self.freq_min_edit)
        mode_layout.addWidget(QLabel("Max:"))
        mode_layout.addWidget(self.freq_max_edit)
        mode_layout.addWidget(self.freq_slider)
        mode_layout.addWidget(self.freq_value_label)
        mode_layout.addStretch(1)

        self.layout.addLayout(mode_layout)

        # --- Button Row ---
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(6)
        buttons_layout.setContentsMargins(0, 0, 0, 0)

        self.save_button = QPushButton("Save Config")
        self.save_button.setFixedWidth(120)
        buttons_layout.addWidget(self.save_button)

        self.load_button = QPushButton("Load Config")
        self.load_button.setFixedWidth(120)
        buttons_layout.addWidget(self.load_button)

        self.add_slider_button = QPushButton("Add Slider")
        self.add_slider_button.setFixedWidth(120)
        buttons_layout.addWidget(self.add_slider_button)

        self.publish_button = QPushButton("Publish")
        self.publish_button.setFixedWidth(120)
        buttons_layout.addWidget(self.publish_button)

        buttons_layout.addStretch(1)
        self.layout.addLayout(buttons_layout)

        # --- Content Area (sliders/groups) ---
        self.content_layout = QVBoxLayout()
        self.content_layout.setSpacing(4)
        self.content_layout.setContentsMargins(0, 10, 0, 0)
        self.layout.addLayout(self.content_layout)

        # Make content layout stretch while upper rows stay fixed
        self.layout.addStretch(1)