from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QPoint, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPainter, QPen, QCursor, QPixmap
import cv2
from hiob_msgs.msg import Rect


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
        self.resize(self.image.width(), self.image.height())
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
