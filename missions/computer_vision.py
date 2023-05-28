import rospy
from std_msgs.msg import String


rospy.init_node('test_node', anonymous=False)

publisher = rospy.Publisher('/vision/request', String, queue_size=10)

def subscriber():
    rospy.Subscriber('/vision/request', String, callback)
    rospy.spin()

def callback(message):
    print(message)

subscriber()

vision_request = "aruco"
publisher.publish(vision_request)