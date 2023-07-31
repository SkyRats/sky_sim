import rospy
from mavros_msgs.msg import State, LandingTarget, PositionTarget
from mavros_msgs.srv import CommandTOL

class MAV():
    def __init__(self):
        # Msgs
        self.drone_state = State()
        self.landing_target = LandingTarget()
        # Services
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    def takeoff(self, height):
        rospy.loginfo("TAKING OFF...")
        self.set_mode("GUIDED")
        self.arm()
        rospy.wait_for_service('/mavros/cmd/takeoff', 10)

        rospy.sleep(1)
        response = self.takeoff_srv(altitude=height)
        
        if response.success:
            rospy.loginfo("Takeoff completed!")
            return
        else:
            rospy.loginfo("Takeoff failed!")
            return
        
    def set_landing_target(self, target_num, angle, distance, size, target_type):
        self.landing_target.target_num = target_num
        self.landing_target.frame = PositionTarget.FRAME_LOCAL_NED
        self.landing_target.angle = angle
        self.landing_target.distance = distance
        self.landing_target.size = size
        self.landing_target.type = target_type

if __name__ == '__main__':
    rospy.init_node('mavbase2')
    mav = MAV()
    mav.takeoff(0.5)
    rospy.spin()

