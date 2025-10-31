
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QLineEdit,
    QSlider, QSpacerItem, QSizePolicy
)

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