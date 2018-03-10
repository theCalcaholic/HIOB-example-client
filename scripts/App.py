#!/usr/bin/env python

import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QDialog, QTextEdit, QGridLayout, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QImage, QPainter, QColor, QPen, QCursor, QGuiApplication, QPixmap
from PyQt5.QtCore import QTimer, QPoint, Qt, QRunnable, QThreadPool, pyqtSignal, QThread
import cv2

from hiob_msgs.msg import Rect

from RosWorker import RosWorker


class LogOutput(QTextEdit):
    def __init__(self, parent=None):
        super(LogOutput, self).__init__(parent)

        self.setReadOnly(True)
        self.setLineWrapMode(self.NoWrap)
        self.insertPlainText("")


class VideoStream(QLabel):
    #frame_received = pyqtSignal(np.ndarray, name='frame_received')
    stream_ready = pyqtSignal(object, name="stream_ready")

    def __init__(self, parent=None):
        super(VideoStream, self).__init__(parent)

        self.setCursor(QCursor(Qt.CrossCursor))
        self.box_size = (40, 40)
        self.cursor_position = QPoint(0, 0)
        self.setMouseTracking(True)
        self.object_pos = None
        #self.resize(400, 400)
        #self.frame_received.connect(self.load_frame)
        self.image = None

    def load_frame(self, frame):
        cv2.cvtColor(frame, cv2.COLOR_BGR2RGB, frame)
        self.image = QImage(frame, frame.shape[1], frame.shape[0], frame.shape[1] * 3, QImage.Format_RGB888)
        #pixmap = QPixmap(self.image)
        #self.setPixmap(pixmap)
        self.update()

    def update_position(self, pos):
        self.object_pos = pos

    def mouseMoveEvent(self, event):
        self.cursor_position.setX(event.x())
        self.cursor_position.setY(event.y())
        self.update()

    def paintEvent(self, paint_event):
        if self.image is not None:
            pixmap = QPixmap(self.image)
            painter = QPainter()
            painter.begin(pixmap)
            #self.resize(self.mQImage.width(), self.mQImage.height())
            #painter.drawImage(0, 0, self.image)
            if self.object_pos is not None:
                green_pen = QPen(Qt.green)
                painter.setPen(green_pen)
                painter.drawRect(
                    self.object_pos.x,
                    self.object_pos.y,
                    self.object_pos.w,
                    self.object_pos.h)
            else:
                red_pen = QPen(Qt.red)
                painter.setPen(red_pen)
                painter.drawRect(
                    min(max(self.cursor_position.x() - int(self.box_size[0] / 2), 0), self.image.width()),
                    min(max(self.cursor_position.y() - int(self.box_size[1] / 2), 0), self.image.height()),
                    self.box_size[0], self.box_size[1])
            painter.end()
            self.setPixmap(pixmap)
        super(VideoStream, self).paintEvent(paint_event)

    def wheelEvent(self, wheel_event):
        modifiers = QGuiApplication.keyboardModifiers()
        self.box_size = (
            self.box_size[0] - wheel_event.angleDelta().y() / 30,
            self.box_size[1] - wheel_event.angleDelta().y() / 30
        )
        self.update()

    def mousePressEvent(self, event):
        print("Mouse pressed!")
        self.stream_ready.emit(
            Rect(
                min(max(self.cursor_position.x() - int(self.box_size[0] / 2), 0), self.image.width()),
                min(max(self.cursor_position.y() - int(self.box_size[1] / 2), 0), self.image.height()),
                self.box_size[0],
                self.box_size[1]))


class App(QWidget):
    test_signal = pyqtSignal()

    def __init__(self, configuration=None, parent=None):
        super(App, self).__init__(parent)

        self.title = "Hiob Demo Client"
        self.setWindowTitle(self.title)

        self.stream_widget = VideoStream(self)
        self.console = LogOutput(self)
        #btn = QPushButton("Start", self)

        grid = QGridLayout()
        grid.addWidget(self.stream_widget, 0, 0, 3, 1)
        grid.addWidget(self.console, 4, 0, 1, 2)
        self.setLayout(grid)
        self.resize(600, 400)

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = App()
    sys.exit(app.exec_())