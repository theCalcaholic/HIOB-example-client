import numpy as np
from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QGridLayout, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QImage, QPainter, QColor, QPen, QCursor, QGuiApplication, QPixmap
from PyQt5.QtCore import QTimer, QPoint, Qt, QRunnable, QThreadPool, pyqtSignal, QThread
from scripts.ui import VideoStream, LogOutput
from scripts.RosWorker import RosWorker


class App(QWidget):
    test_signal = pyqtSignal()

    def __init__(self, configuration=None, parent=None):
        super(App, self).__init__(parent)

        self.title = "Hiob Demo Client"
        self.setWindowTitle(self.title)

        self.stream_widget = VideoStream(self)
        self.console = LogOutput(self)
        #btn = QPushButton("Start", self)
        #self.settings_widget = SettingsPane(self)

        grid = QGridLayout()
        #grid.addWidget(self.settings_widget, 0, 0, 3, 2)
        grid.addWidget(self.stream_widget, 0, 0, 3, 4)
        grid.addWidget(self.console, 4, 0, 1, 4)
        self.setLayout(grid)
        self.resize(600, 600)

        self.ros_service = RosWorker()
        #self.ros_service.connect_stream_widget(stream_widget)
        self.worker_thread = QThread()
        self.ros_service.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.ros_service.start_work)
        self.worker_thread.start()

        self.ros_service.frame_received.connect(self.stream_widget.load_frame)
        self.ros_service.position_received.connect(self.receive_tracking)
        self.stream_widget.stream_ready.connect(self.start_stream)


        #self.ros_service.start()

        #stream_widget.stream_ready.connect(self.ros_service.start_streaming)
        #self.test_signal.connect(self.test)
        #self.test_signal.emit()
        #QThreadPool.globalInstance().start(self.ros_service)

        self.show()

    def receive_tracking(self, tracking_result):
        self.console.append("""received result:
                position: [(x:{0.position.x}, y:{0.position.y}), (w:{0.position.w}, h:{0.position.h})]
                prediction quality: {0.predictionQuality}
                loss: {0.lostObject}\n""".format(tracking_result))
        self.stream_widget.update_position(tracking_result.position)

    def start_stream(self, pos):
        self.ros_service.start_streaming(pos)

    def closeEvent(self, event):
        #QThreadPool.globalInstance().cancel(self.ros_service)
        self.worker_thread.exit()
        event.accept()