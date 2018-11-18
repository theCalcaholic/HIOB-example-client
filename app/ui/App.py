from PyQt5.QtWidgets import QGridLayout, QWidget, QAbstractScrollArea, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, QThread, Qt
from . import VideoStream, LogOutput
from SettingsPane import SettingsPane
from app.ros import RosWorker
from threading import Timer
import rospy


class App(QWidget):
    test_signal = pyqtSignal()

    def __init__(self, configuration=None, parent=None):
        super(App, self).__init__(parent)

        self.title = "Hiob Demo Client"
        self.setWindowTitle(self.title)

        self.stream_widget = VideoStream(self)
        self.console = LogOutput(self)
        self.console.setFixedHeight(200)
        #self.console.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        #btn = QPushButton("Start", self)
        self.settings_widget = SettingsPane(self)


        grid = QGridLayout()
        #grid.setAlignment(Qt.AlignHCenter)
        grid.addWidget(self.settings_widget, 0, 0, 1, 2)
        grid.addWidget(self.stream_widget, 1, 1, 1, 1)
        grid.addWidget(self.console, 2, 0, 1, 2)
        grid.setRowMinimumHeight(0, 40)
        grid.setColumnMinimumWidth(0, 120)
        grid.setColumnStretch(0, 0)
        grid.setRowStretch(0, 0)
        grid.setRowStretch(1, 1)
        grid.setRowStretch(2, 0)

        #tracker_column = QVBoxLayout()

        #layout = QHBoxLayout()
        #layout.addChildLayout(tracker_column)
        #layout.addChildLayout(grid)

        self.setLayout(grid)
        self.resize(600, 600)

        self.ros_service = RosWorker()
        #self.ros_service.connect_stream_widget(stream_widget)
        self.worker_thread = QThread()
        self.ros_service.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.ros_service.start_work)

        self.ros_service.frame_received.connect(self.stream_widget.load_frame)
        self.ros_service.position_received.connect(self.receive_tracking)
        self.stream_widget.stream_ready.connect(self.start_stream)


        #self.ros_service.start()

        #stream_widget.stream_ready.connect(self.ros_service.start_streaming)
        #self.test_signal.connect(self.test)
        #self.test_signal.emit()
        #QThreadPool.globalInstance().start(self.ros_service)

        self.show()
        self.worker_thread.start()
        rospy.on_shutdown(self.close_app)
        #self.start_ros()

    def close_app(self):
        print("closing1...")
        self.worker_thread.exit()
        print("closing2...")
        rospy.signal_shutdown("App exiting...")

    def receive_tracking(self, tracking_result):
        self.console.append("""received result:
                position: [(x:{0.position.x}, y:{0.position.y}), (w:{0.position.w}, h:{0.position.h})]
                prediction quality: {0.predictionQuality}
                loss: {0.lostObject}\n""".format(tracking_result))
        self.stream_widget.update_position(tracking_result.position)

    def start_stream(self, pos):
        self.ros_service.start_streaming(pos)

    def toggle_pause_stream(self):
        self.ros_service.streaming_toggle_paused()

    def closeEvent(self, event):
        #QThreadPool.globalInstance().cancel(self.ros_service)
        event.accept()
        self.close_app()
