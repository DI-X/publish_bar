import sys
import json
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QPushButton,
    QFileDialog, QTabWidget, QInputDialog
)
from components.scripts.msgWidgets import *
from components.scripts.ros1.dynamicContentROS1 import DynamicContentRos1
import rospy
import itertools

class TabbedMain(QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node('publish_bar', anonymous=True)
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

        self.topic_counter = itertools.count(1)
        # start with one tab
        self.add_tab()

    def add_tab(self, state: dict = None, title: str = None):
        topic_name = f"robot/publish_bar_{next(self.topic_counter)}"
        tab = DynamicContentRos1(topic_name)
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
