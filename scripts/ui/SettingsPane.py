from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QGridLayout, QWidget, QLabel, QPushButton
from PyQt5.QtCore import QTimer, QPoint, Qt, QRunnable, QThreadPool, pyqtSignal, QThread


class SettingsPane(QWidget):
    toggle_pause = pyqtSignal(bool, name="toggle_pause")

    def __init__(self, parent=None):
        super(SettingsPane, self).__init__(parent)
        pause_button = QPushButton("pause", self)
        pause_button.clicked.connect(self.pause)
        #self.add(pause_button)

    def pause(self):
        print "PAUSE!"