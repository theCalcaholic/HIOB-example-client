#!/usr/bin/env python

import logging
import argparse
import rospy
import sensor_msgs.msg
import cv_bridge
import nicovision.VideoDevice
import cv2
import sys
from hiob_msgs.msg import Rect
from hiob_msgs.msg import FrameWithGroundTruth
from PIL import Image
import io
import time

print("OpenCV version: {}".format(cv2.__version__))


class NicoRosVision():
    """
    The NicoRosVision class exposes a camera stream over ROS
    """

    @staticmethod
    def get_config():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            'device': '',
            'framerate': 30,
            'width': 640,
            'height': 480,
            'rostopicName': '/nico/vision'
        }

    def __init__(self, callback=None, config=None):
        """
        The NicoRosVision enables the sending of a camera image through ROS

        :param config: Configuration dict
        :type config: dict
        """
        self._device = None
        self._stream_running = False
        self._config = config
        if config is None:
            self._config = NicoRosVision.get_config()
        self._bridge = cv_bridge.CvBridge()
        self._external_callback = callback
        self.initial_position = None
        self.transmitting_pos = 0
        self.frame_count = 0

        logging.info('-- Init NicoRosMotion --')

        logging.debug('Init ROS publisher')
        self._publisher = rospy.Publisher(self._config['rostopicName'] + '/videoStream', FrameWithGroundTruth, queue_size = 1)

        logging.info('-- All done --')
        pass

    def connect_device(self):
        if self._device is not None:
            logging.warning('A video device is already connected')
            return
        self._device = nicovision.VideoDevice.VideoDevice.fromDevice(self._config['device'])
        if self._device is None:
            logging.error('Can not initialise device - is the device name correct and not ambiguous?')
            return
        self._device.addCallback(self._callback)
        #self._device.setFrameRate(self._config['framerate'])
        self._device.setResolution(self._config['width'], self._config['height'])
        self._device.open()

    def disconnect_device(self):
        if self._device is None:
            logging.warning('No video device connected')
            return
        self._device.close()
        self._device = None

    def start_stream(self):
        """
        Starts the stream
        """
        if self._device is None:
            self.connect_device()
        if self._stream_running:
            logging.warning('Stream already running')
            return
        self.transmitting_pos = 10
        self._stream_running = True

    def stop_stream(self):
        """
        Stops the stream
        """
        self.disconnect_device()
        if not self._stream_running:
            logging.warning('Stream not running')
            return
        self._stream_running = False

    def is_running(self):
        """
        Returns true if stream is currently running

        :return: true if running
        :rtype: bool
        """
        return self._stream_running

    def _callback(self, rval, frame):
        """
        Callback for device

        :param rval: rval
        :param frame: frame
        """
        if frame is None:
            return

        self.frame_count += 1
        if self._external_callback is not None:
            self._external_callback(frame)

        if self._stream_running:
            cmd = ''
            #compressed_img = self._bridge.cv2_to_compressed_imgmsg(frame, 'png')
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            compressed_img = self._bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            """test = Image.frombytes("RGB", (compressed_img.width, compressed_img.height), compressed_img.data)

            if test.mode != "RGB":
                print("NOT RGB!")
                test = test.convert("RGB")
            test.show()
            time.sleep(3.0)"""
            ground_truth = Rect(0, 0, 40, 40)
            if self.initial_position and self.transmitting_pos > 0:
                ground_truth = self.initial_position
                cmd = 'start'
                self.transmitting_pos -= 1
            msg = FrameWithGroundTruth(compressed_img, cmd, ground_truth, "{}".format(self.frame_count))
            self._publisher.publish(msg)


if __name__ == '__main__':
    config = NicoRosVision.get_config()

    parser = argparse.ArgumentParser(description='NICO ROS vision interface')
    parser.add_argument('--log-level', dest='logLevel', help='Sets log level. Default: INFO', type=str, default='INFO')
    parser.add_argument('--log-file', dest='logFile', help='Path to log file. Default: NICO_VISION.log', type=str, default='NICO_VISION.log')
    parser.add_argument('-d', '--device', dest='device', help='Target device. Default: %s' % config['device'], type=str)
    parser.add_argument('-f', '--framerate', dest='framerate', help='Capture framerate. Default: %i' % config['framerate'], type=int)
    parser.add_argument('-W', '--width', dest='width', help='Image width. Default: %i' % config['width'], type=float)
    parser.add_argument('-H', '--height', dest='height', help='Image height. Default: %i' % config['height'], type=float)
    parser.add_argument('--rostopic-name', dest='rostopicName', help='Topic name for ROS. Default: %s' % config['rostopicName'], type=str)

    args = parser.parse_known_args()[0]

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

    rospy.init_node('hiob_example_client', anonymous=True)
    vision = NicoRosVision(config)

    vision.start_stream()
    rospy.spin()
    vision.stop_stream()
