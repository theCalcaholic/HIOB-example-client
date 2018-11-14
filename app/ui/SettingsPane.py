from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QGridLayout, QWidget, QLabel, QPushButton
from PyQt5.QtCore import QTimer, QPoint, Qt, QRunnable, QThreadPool, pyqtSignal, QThread


class SettingsPane(QWidget):


    def __init__(self, parent=None):
        super(SettingsPane, self).__init__(parent)
        pause_button = QPushButton("pause", self)
        #pause_button.clicked.connect(self.pause)
        #self.toggle_pause = pyqtSignal(bool, name="toggle_pause")
        pause_button.clicked.connect(parent.toggle_pause_stream)
        #self.add(pause_button)

    def pause(self):
        #print "PAUSE!"
        pass