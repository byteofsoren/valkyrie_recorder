#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    """Tarker function
    :returns: TODO

    """
    pub = rospy.Publisher('chatter',String,queue_size=10)
    ropy.init_node('talker', anonymous=False)
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        hello_str = f'Hello time={rospy.get_time()}'
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
    try:
       talker()
    except rospy.ROSException as e:
        raise e
