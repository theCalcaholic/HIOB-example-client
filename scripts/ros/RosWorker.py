#!/usr/bin/env python

import sys
import argparse
import logging
import rospy
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QRunnable, QMetaObject, Qt, Q_ARG, pyqtSignal, QThread, QObject, pyqtSlot
from hiob_msgs.msg import TrackingResult
import numpy

from NicoRosVision import CameraVision
from TrackingSubscriber import TrackingSubscriber


class RosWorker(QObject):
    #stream_ready = pyqtSignal(Rect, name='stream_ready')
    frame_received = pyqtSignal(numpy.ndarray, name='frame_received')
    position_received = pyqtSignal(object, name='position_received')

    def __init__(self, parent=None):
        #super(RosThread, self).__init__(self)
        QObject.__init__(self, parent)
        rospy.init_node('hiob_example_client', anonymous=True)
        self.subscriber = None
        self.vision_service = None

        #self.stream_ready.connect(self.start_streaming)

    #def connect_stream_widget(self, widget):
    #    print "connecting stream widget"
    #    widget.stream_ready.connect(self.start_streaming)

    def get_subscriber(self):
        return TrackingSubscriber(self.position_callback)

    def get_video_service(self, args):
        config = CameraVision.get_config()

        # Parse args
        if args.device:
            config['device'] = args.device
        if args.framerate:
            config['framerate'] = args.framerate
        if args.width:
            config['width'] = args.width
        if args.height:
            config['height'] = args.height
        if args.rostopicName:
            config['rostopicName'] = args.rostopicName

        return CameraVision(self.video_callback, config)

    def streaming_set_paused(self, pause):
        if self.vision_service is None:
            raise Exception("You must start the video stream first!")
        self.vision_service.paused = pause

    def streaming_toggle_paused(self):
        print("toggle paused")
        if self.vision_service is None:
            raise Exception("You must start the video stream first!")
        self.vision_service.paused = not self.vision_service.paused


    @pyqtSlot(object)
    def start_streaming(self, pos):
        print "starting stream"
        self.vision_service.initial_position = pos
        self.vision_service.start_stream()

    def video_callback(self, frame):
        if frame is not None:
            #height, width, byte_value = frame.shape
            #byte_value = byte_value * width
            #image = QImage(frame, width, height, byte_value, QImage.Format_RGB888)
            #self.stream_widget.mQImage = image
            #self.stream_widget.update()
            #self.stream_widget.frame_received.emit(frame)
            self.frame_received.emit(frame)
        #QMetaObject.invokeMethod(self.stream_widget,
        #                         "load_frame", Qt.AutoConnection,
        #                         Q_ARG(QImage, frame))
        #QMetaObject.invokeMethod(self.console,
        #                         "append", Qt.QueuedConnection,
        #                         Q_ARG(str, "received frame!"))

    def position_callback(self, result):
        self.position_received.emit(result)

    @pyqtSlot()
    def start_work(self):
        video_config = CameraVision.get_config()
        parser = argparse.ArgumentParser(description='NICO ROS vision interface')
        parser.add_argument('--log-level', dest='logLevel', help='Sets log level. Default: INFO', type=str, default='INFO')
        parser.add_argument('--log-file', dest='logFile', help='Path to log file. Default: NICO_VISION.log', type=str,
                            default='NICO_VISION.log')
        parser.add_argument('-d', '--device', dest='device', help='Target device. Default: %s' % video_config['device'], type=str)
        parser.add_argument('-f', '--framerate', dest='framerate',
                            help='Capture framerate. Default: %i' % video_config['framerate'], type=int)
        parser.add_argument('-W', '--width', dest='width', help='Image width. Default: %i' % video_config['width'], type=float)
        parser.add_argument('-H', '--height', dest='height', help='Image height. Default: %i' % video_config['height'],
                            type=float)
        parser.add_argument('--rostopic-name', dest='rostopicName',
                            help='Topic name for ROS. Default: %s' % video_config['rostopicName'], type=str)

        args = parser.parse_known_args()[0]
        # Set logging setting
        loggingLevel = logging.INFO
        try:
            loggingLevel = {
                'DEBUG': logging.DEBUG,
                'INFO': logging.INFO,
                'WARNING': logging.WARNING,
                'CRITICAL': logging.CRITICAL,
                'debug': logging.DEBUG,
                'info': logging.INFO,
                'warning': logging.WARNING,
                'critical': logging.CRITICAL,
            }[args.logLevel]
        except:
            sys.stderr.write('LOGGING ERROR: Unknown log level %s\n' % args.logLevel)
            pass

        logging.basicConfig(filename=args.logFile,
                            format='%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s',
                            level=loggingLevel)
        stdoutHandler = logging.StreamHandler(sys.stdout)
        stdoutHandler.setLevel(loggingLevel)
        logging_format = logging.Formatter(
            '%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s')
        stdoutHandler.setFormatter(logging_format)
        logging.getLogger().addHandler(stdoutHandler)

        self.subscriber = self.get_subscriber()
        self.vision_service = self.get_video_service(args)

        self.subscriber.start()
        #self.vision_service.connect_device()
        self.vision_service.start_capture()
        rospy.spin()
        self.vision_service.stop_stream()
        self.subscriber.stop()
