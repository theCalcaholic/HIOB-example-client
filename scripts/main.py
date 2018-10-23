#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QGridLayout, QWidget, QLabel, QPushButton
from ui import App

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = App()
    sys.exit(app.exec_())