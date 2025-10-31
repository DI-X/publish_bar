from PySide6.QtWidgets import QVBoxLayout,  QGroupBox
from components.scripts.slideBar import slideBar

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
        # for s in self.sliders:
        #     s.setVisible(checked)

        self.setVisible(checked)

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
