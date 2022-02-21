
import rospy
from marvelmind_nav.msg import hedge_pos_ang,marvelmind_waypoint

addr=0; ts=0; Xpos=0; Ypos=0; Zpos=0; Ang=0; Flag=0
index=0; total_items=0; movement_type=0; param1=0; param2=0; param3=0

def hedge_pos_ang_callback(msg):
    global addr, ts, Xpos, Ypos, Zpos, Ang, Flag

    addr = msg.address
    ts = msg.timestamp_ms
    Xpos = msg.x_m
    Ypos = msg.y_m
    Zpos = msg.z_m
    Ang = msg.angle
    Flag = msg.flags
    print(addr, ts, Xpos, Ypos, Zpos, Ang, Flag)

def marvelmind_waypoint_callback(msg):
    global index, total_items, movement_type, param1, param2, param3

    index = msg.item_index
    total_items = msg.total_items
    movement_type = msg.movement_type
    param1 = msg.param1
    param2 = msg.param2
    param3 = msg.param3
    print(index, total_items, movement_type, param1, param2, param3)

rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
rospy.Subscriber("marvelmind_waypoint", marvelmind_waypoint, marvelmind_waypoint_callback, queue_size=1) 

if __name__ == "__main__":

    rospy.init_node("MarvelM")

    while not rospy.is_shutdown():
        pass
        #print(addr, ts, Xpos, Ypos, Zpos, Ang, Flag)
        #print(index, total_items, movement_type, param1, param2, param3)

