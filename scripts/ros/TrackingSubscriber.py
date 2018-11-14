import rospy
import hiob_msgs.msg


class TrackingSubscriber:
    def __init__(self, callback=None):
        self.running = False
        self.results = []
        self.external_callback = callback

    def start(self):
        self.running = True
        print "subscribing..."
        rospy.Subscriber('/hiob/object', hiob_msgs.msg.TrackingResult, self.receive_message)

    def receive_message(self, tracking_result):
        print """received result:
        position: [(x:{0.position.x}, y:{0.position.y}), (w:{0.position.w}, h:{0.position.h})]
        prediction quality: {0.predictionQuality}
        loss: {0.lostObject}\n""".format(tracking_result)
        self.results.append(tracking_result)
        if self.external_callback:
            self.external_callback(tracking_result)


    def stop(self):
        self.running = False


if __name__ == '__main__':
    rospy.init_node('hiob_example_client', anonymous=True)
    subscriber = TrackingSubscriber()
    subscriber.start()
    rospy.spin()
    subscriber.stop()
