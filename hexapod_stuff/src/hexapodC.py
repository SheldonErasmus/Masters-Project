import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class HexapodC:

    def __init__(self, ns='/simple_hexapod/'):
        self.ns = ns
        self.joints = None
        self.angles = None

        self.sub_joints = rospy.Subscriber(ns+'joint_states', JointState, self.cb_joints, queue_size=1)
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)
            rospy.loginfo('waiting for joints')
        rospy.loginfo('joints populated') 

        rospy.loginfo('Creating joint command publishers')
        self.pub_angles = {}
        for j in self.joints:
            pub = rospy.Publisher(ns+j+'_position_controller/command',Float64,queue_size=1)
            self.pub_angles[j] = pub 

        rospy.sleep(1)

        self._pub_cmd_vel = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size=1)

    def set_walk_velocity(self, x, y, t):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        self._pub_cmd_vel.publish(msg)

    def cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                rospy.logerror('Invalid joint name "' + j + '"')
                continue
            self.pub_angles[j].publish(v)