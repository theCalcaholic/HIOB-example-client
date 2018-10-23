from PyQt5.QtWidgets import QTextEdit


class LogOutput(QTextEdit):
    def __init__(self, parent=None):
        super(LogOutput, self).__init__(parent)

        self.setReadOnly(True)
        self.setLineWrapMode(self.NoWrap)
        self.insertPlainText("")