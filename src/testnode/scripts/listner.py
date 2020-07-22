#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data:String):
    """The callback funciton is used by the rospy
    access the data from the network

    :data: TODO
    :returns: TODO

    """
    # rospy.loginfo(f'{rospy.get_caller_id()} I hered {data.data}')
    rospy.loginfo("test")
    rospy.loginfo(data.data)


def listner():
    """ The listren of the messages

    :arg1: TODO
    :returns: TODO

    """
    rospy.init_node('listner')
    rospy.loginfo("Subscriber to chatter")
    rospy.Subscriber("chatter", String, callback)
    rospy.loginfo("spin")
    rospy.spin()


if __name__ == "__main__":
    listner()
