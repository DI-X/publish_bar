from PySide6.QtWidgets import QWidget, QVBoxLayout
from components.scripts.slideBar import slideBar
from components.scripts.sliderGroup import SliderGroup
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSizePolicy, QTextEdit
)

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

class StringWidget(QWidget):
    def __init__(self):
        super().__init__()

        # Main vertical layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # Top row for label + text edit
        row = QHBoxLayout()
        row.setSpacing(8)

        # Label on the top left
        self.label = QLabel("Message:")
        self.label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        row.addWidget(self.label)

        # Multiline text edit that expands both ways
        self.text_edit = QTextEdit()
        self.text_edit.setPlaceholderText("Enter message text here...")
        self.text_edit.setMinimumHeight(120)
        self.text_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        row.addWidget(self.text_edit, stretch=1)

        # Add row to the main layout
        layout.addLayout(row)
        layout.addStretch()

    def get_text(self):
        return self.text_edit.toPlainText()

    def set_text(self, text):
        self.text_edit.setPlainText(text)
