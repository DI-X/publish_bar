import sys
from PySide6.QtWidgets import QApplication
from components.scripts.lcm.dynamicContentLcm import DynamicContentLcm
from components.scripts.baseClass.publishBarBase import PublishBarBase

class PublishBarLcm(PublishBarBase):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Publish Bar Lcm")

    def add_tab(self, state: dict = None, title: str = None):
        topic_name = f"robot/publish_bar_{next(self.topic_counter)}"
        tab = DynamicContentLcm(topic_name)
        idx = self.tabs.addTab(tab, title or f"Tab {self.tabs.count() + 1}")
        self.tabs.setCurrentIndex(idx)
        if state:
            tab.load_state(state)

# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = PublishBarLcm()
    win.show()
    sys.exit(app.exec())
