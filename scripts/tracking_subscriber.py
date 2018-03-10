import rospy
import hiob_ros.msg


class TrackingSubscriber:
    def __init__(self):
        self.running = False
        self.results = []

    def subscribe(self):
        self.running = True
        print "subscribing..."
        rospy.init_node('hiob_example_client', anonymous=True)
        rospy.Subscriber('/core/objects/0', hiob_ros.msg.TrackingResult, self.receive_message)

    def receive_message(self, tracking_result):
        self.results.append(tracking_result)
        print """received result:
        position: [(x:{0.position.x}, y:{0.position.y}), (w:{0.position.w}, h:{0.position.h})]
        prediction quality: {0.predictionQuality}
        loss: {0.lostObject}\n""".format(tracking_result)

    def stop(self):
        self.running = False


if __name__ == '__main__':
    subscriber = TrackingSubscriber()
    subscriber.subscribe()
    rospy.spin()
    subscriber.stop()
