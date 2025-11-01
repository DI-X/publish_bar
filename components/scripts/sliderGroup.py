from PySide6.QtWidgets import QVBoxLayout,  QGroupBox, QHBoxLayout, QLabel, QComboBox
from components.scripts.slideBar import slideBar

class SliderGroup(QGroupBox):
    def __init__(self, title, labels=None, slider_callback=None,
                 name_changed_callback=None, unit_selectable=False):
        super().__init__(title)
        self.setCheckable(True)
        self.setChecked(True)
        self.slider_callback = slider_callback
        self.name_changed_callback = name_changed_callback
        self.container = QVBoxLayout()
        self.sliders = []
        self.unit_selectable = unit_selectable

        # --- Unit dropdown row ---
        if unit_selectable:
            unit_row = QHBoxLayout()
            unit_row.addWidget(QLabel("Outputs:"))
            self.unit_box = QComboBox()
            self.unit_box.addItems(["Raw values", "Deg → Rad"])
            self.unit_box.currentTextChanged.connect(self._on_unit_changed)
            unit_row.addWidget(self.unit_box)
            unit_row.addStretch()
            self.container.addLayout(unit_row)
        else:
            self.unit_box = None

        if labels:
            for lbl in labels:
                self.add_slider(lbl)

        self.setLayout(self.container)
        self.toggled.connect(self._on_toggled)

    def _on_toggled(self, checked):
        # for s in self.sliders:
        #     s.setVisible(checked)

        self.setVisible(checked)

        # -------- Unit Handling --------

    def _on_unit_changed(self, mode):
        """If switching to Deg→Rad, immediately enforce limits."""
        self._clamp_slider_ranges()

    def _clamp_slider_ranges(self):
        """Clamp all sliders to [-360, 360] if mode is Deg→Rad."""
        if not (self.unit_box and self.unit_box.currentText() == "Deg → Rad"):
            return
        for s in self.sliders:
            try:
                min_v = float(s.min_edit.text())
                max_v = float(s.max_edit.text())

                # enforce bounds
                min_v = max(min_v, -360)
                max_v = min(max_v, 360)
                if min_v >= max_v:
                    min_v, max_v = -360, 360

                # correct the text if changed
                if float(s.min_edit.text()) != min_v:
                    s.min_edit.setText(str(min_v))
                if float(s.max_edit.text()) != max_v:
                    s.max_edit.setText(str(max_v))

                s.set_range(min_v, max_v)
            except Exception:
                pass

    def add_slider(self, name="Value", min_val=-10.0, max_val=10.0,
                   value=0.0, name_changed_callback=None):
        callback = name_changed_callback if name_changed_callback is not None else self.name_changed_callback
        s = slideBar(name, min_val, max_val, value,
                     callback=self.slider_callback,
                     name_changed_callback=callback)
        self.container.addWidget(s)
        self.sliders.append(s)

        # Connect min/max edits to enforce degree clamp when necessary
        if self.unit_box:
            s.min_edit.editingFinished.connect(self._clamp_slider_ranges)
            s.max_edit.editingFinished.connect(self._clamp_slider_ranges)

        return s

    # -------- Data Extraction --------
    def get_slider_values(self):
        """Return list of current slider values with unit conversion if needed."""
        values = [s.get_value() for s in self.sliders]
        if self.unit_box and self.unit_box.currentText() == "Deg → Rad":
            import math
            values = [math.radians(v) for v in values]
        return values

    def clear(self):
        for s in self.sliders:
            try:
                s.deleteLater()
            except Exception:
                pass
        self.sliders.clear()

