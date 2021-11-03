
import rospy
from hexapodC import HexapodC
from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node("quicktest")

    mode_pub = rospy.Publisher('/mode_selected',Float32,queue_size=1)
    mode_pub.publish(data=-1)
    rospy.sleep(0.01)
    mode_pub.publish(data=2)

    robot = HexapodC()
    rospy.sleep(1)
    robot.set_walk_velocity(0.08,0.08,0)
    #rospy.sleep(10)
    #robot.set_path_var(Ss=150)
    #while not rospy.is_shutdown():
    #    rospy.sleep(1)