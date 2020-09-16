#!/usr/bin/env python
import rospy
import rostopic

import std_msgs


class BIT:
    def __init__(self, topics, max_publish_rate=1):

        # this will listen to publishing rate
        self.topic_hz = dict()

        # subscriber per topic
        self.subscribers = dict()

        for topic_name in topics:
            # poll
            self.topic_hz[topic_name] = rostopic.ROSTopicHz(-1)

            # subscribe
            self.subscribers[topic_name] = rospy.Subscriber(topic_name, rospy.AnyMsg,
                                                            self.topic_hz[topic_name].callback_hz,
                                                            callback_args=topic_name)

        # a publishing rate higher than this will cause an error report
        self.min_publish_rate = max_publish_rate

        # publisher for bit
        self.bit_pub = rospy.Publisher('/wizzy/bit_state', std_msgs.msg.String, queue_size=10)

    def status(self):
        rospy.sleep(1)

        # aggregate topic statuses
        topic_status = []

        for topic_name in self.subscribers.keys():
            # get publish rate
            hz = self.topic_hz[topic_name].get_hz(topic_name)

            # if we got nothing
            if hz is None:
                rospy.logwarn('topic {} not published'.format(topic_name))
                topic_status.append(False)
            elif hz[0] < self.min_publish_rate:
                rospy.logwarn('topic {} publish rate invalid'.format(topic_name))
                topic_status.append(False)
            else:
                rospy.logdebug('topic {} publish rate {}'.format(topic_name, hz[0]))
                topic_status.append(True)

        if any([not x for x in topic_status]):
            self.bit_pub.publish('false')

        else:
            self.bit_pub.publish('true')


if __name__ == '__main__':

    # start this node
    rospy.init_node('bit', log_level=rospy.WARN)

    # topics to look for
    topics = ['/wizzy/obstacle_list', '/wizzy/lidar_proc']

    # create test object
    bit = BIT(topics=topics)

    # test frequency
    rate = rospy.Rate(1.0/8)

    while not rospy.is_shutdown():
        # publish status
        bit.status()

        # go back to sleep
        rate.sleep()
