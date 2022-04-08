import signal
from xbox360controller import Xbox360Controller
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from my_message.msg import PathVar_n_cmdVel

import sys
sys.path.append("/home/devlon/catkin_ws/src/simu_hexapod_stuff/src")
from hexapodC import HexapodC

flag_a = 0; flag_y = 0; flag_x = 0; flag_b = 0; flag_LB = 0; flag_RB = 0;flag_cam=0;flag_way=0
mode = 0; mode_selected = -1
start = 0
axX =0.0; axY = 0.0; flag_Lstick = 0
vx = 0.0; vy = 0.0; totV = 0.0

robot = HexapodC()
rospy.sleep(1)

bh = HexapodC.BH; turnAng = 0.0; FootH = HexapodC.Fh; StepH = HexapodC.Sh
vx_way = 0; vy_way = 0;z_way = 0

def vel_path_cb(msg):
    global FootH,StepH,flag_cam,vx_way,z_way,flag_way
    if msg.Name == 'Camera':
        flag_cam = 1
        FootH = msg.path_var.Fh
        StepH = msg.path_var.Sh
    if msg.Name == 'Waypoint':
        flag_way = 1
        vx_way = msg.linear.x
        z_way = msg.angular.z



rospy.init_node('XboxController')

teleop_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
mode_pub = rospy.Publisher('/mode_selected',Float32,queue_size=1)

rospy.Subscriber('/simple_hexapod/changed_vel_path_var',PathVar_n_cmdVel,vel_path_cb,queue_size=1)

def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def on_A_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_a
    flag_a = 1
def on_A_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_a
    flag_a = 0
def on_Y_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_y
    flag_y = 1
def on_Y_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_y
    flag_y = 0

def on_X_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_x
    flag_x = 1
def on_X_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_x
    flag_x = 0
def on_B_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_b
    flag_b = 1
def on_B_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_b
    flag_b = 0

def on_LB_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_LB
    flag_LB = 1
def on_LB_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_LB
    flag_LB = 0

def on_RB_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global flag_RB
    flag_RB = 1
def on_RB_button_released(button):
    #print('Button {0} was released'.format(button.name))
    global flag_RB
    flag_RB = 0

def on_mode_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global mode
    mode = mode + 1
    if mode > 2:
        mode = 0
    print(mode)
def on_mode_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_select_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global mode_selected
    if totV == 0.0:
        mode_selected = mode
        print('mode selected: {0}'.format(mode_selected))
        mode_pub.publish(data=mode_selected)
def on_select_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_start_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global start
    if start == 0:
        start = 1
        mode_pub.publish(data=-1)
def on_start_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_Laxis_moved(axis):
    global axX, axY, flag_Lstick
    axX = -axis.y
    axY = axis.x
    flag_Lstick = 1
    

try:
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        # Button A events
        controller.button_a.when_pressed = on_A_button_pressed
        controller.button_a.when_released = on_A_button_released
        controller.button_y.when_pressed = on_Y_button_pressed
        controller.button_y.when_released = on_Y_button_released

        controller.button_x.when_pressed = on_X_button_pressed
        controller.button_x.when_released = on_X_button_released
        controller.button_b.when_pressed = on_B_button_pressed
        controller.button_b.when_released = on_B_button_released

        controller.button_trigger_l.when_pressed = on_LB_button_pressed
        controller.button_trigger_l.when_released = on_LB_button_released

        controller.button_trigger_r.when_pressed = on_RB_button_pressed
        controller.button_trigger_r.when_released = on_RB_button_released

        controller.button_mode.when_pressed = on_mode_button_pressed
        controller.button_mode.when_released = on_mode_button_released

        controller.button_select.when_pressed = on_select_button_pressed
        controller.button_select.when_released = on_select_button_released

        controller.button_start.when_pressed = on_start_button_pressed
        controller.button_start.when_released = on_start_button_released


        # Left and right axis move event
        controller.axis_l.when_moved = on_Laxis_moved
        #controller.axis_r.when_moved = on_axis_moved

        #signal.pause()
        while not rospy.is_shutdown():
            
            if start == 1:
                if mode_selected == 0:
                    msg = Twist()

                    if flag_a == 1 and flag_LB != 1 and flag_RB != 1:
                        msg.linear.x = -0.5
                        teleop_pub.publish(msg)
                    if flag_y == 1 and flag_LB != 1 and flag_RB != 1:
                        msg.linear.x = 0.5
                        teleop_pub.publish(msg)

                    if flag_x == 1 and flag_LB != 1 and flag_RB != 1:
                        msg.linear.y = 0.5
                        teleop_pub.publish(msg)
                    if flag_b == 1 and flag_LB != 1 and flag_RB != 1:
                        msg.linear.y = -0.5
                        teleop_pub.publish(msg)

                    if flag_a == 1 and flag_LB == 1 and flag_RB != 1:
                        msg.linear.z = -0.5
                        teleop_pub.publish(msg)
                    if flag_y == 1 and flag_LB == 1 and flag_RB != 1:
                        msg.linear.z = 0.5
                        teleop_pub.publish(msg)

                    if flag_x == 1 and flag_LB == 1 and flag_RB != 1:
                        msg.angular.z = 0.5
                        teleop_pub.publish(msg)
                    if flag_b == 1 and flag_LB == 1 and flag_RB != 1:
                        msg.angular.z = -0.5
                        teleop_pub.publish(msg)

                    if flag_a == 1 and flag_RB == 1 and flag_LB != 1:
                        msg.linear.y = -0.6
                        teleop_pub.publish(msg)
                    if flag_y == 1 and flag_RB == 1 and flag_LB != 1:
                        msg.linear.y = 0.6
                        teleop_pub.publish(msg)

                    if flag_x == 1 and flag_RB == 1 and flag_LB != 1:
                        msg.linear.x = 0.6
                        teleop_pub.publish(msg)
                    if flag_b == 1 and flag_RB == 1 and flag_LB != 1:
                        msg.linear.x = -0.6
                        teleop_pub.publish(msg)

                elif mode_selected == 1:
                    pass

                elif mode_selected == 2:
                    """if flag_Lstick == 1:
                        if axX>=0.3:
                            axX = _map(axX,0.3, 1.0, 0.0, 0.1)
                        elif axX<=-0.3:
                            axX = _map(axX,-0.3, -1.0, 0.0, -0.1)
                        else:
                            axX = 0.0

                        if axY>=0.3:
                            axY = _map(axY,0.3, 1.0, 0.0, 0.1)
                        elif axY<=-0.3:
                            axY = _map(axY,-0.3, -1.0, 0.0, -0.1)
                        else:
                            axY = 0.0

                        totV = (axX**2+axY**2)**(1/2)

                        print('vx: {0} vy: {1} V: {2}'.format(axX, axY, totV))
                        robot.set_walk_velocity(axX,axY,0)
                        flag_Lstick = 0"""

                    if flag_y == 1 or flag_a == 1 or flag_b == 1 or flag_x == 1:
                        if flag_LB == 1:
                            if flag_y == 1:
                                flag_y = 0
                                bh = bh + 1
                            elif flag_a == 1:
                                flag_a = 0
                                bh = bh - 1
                            if flag_b == 1:
                                flag_b = 0
                                turnAng = turnAng + 0.25
                            elif flag_x == 1:
                                flag_x = 0
                                turnAng = turnAng - 0.25

                            if turnAng > 30:
                                turnAng = 30
                            elif turnAng < -30:
                                turnAng = -30

                            turnAng = round(turnAng,2)

                            print('body height: {0} Turn Ang: {1}'.format(bh, turnAng))
                            robot.set_walk_velocity(vx,vy,turnAng)
                            robot.set_path_var(BH = bh)

                        else:
                            if flag_y == 1:
                                flag_y = 0
                                vx = vx + 0.0050
                            elif flag_a == 1:
                                flag_a = 0
                                vx = vx - 0.0050
                            if flag_b == 1:
                                flag_b = 0
                                vy = vy + 0.0050
                            elif flag_x == 1:
                                flag_x = 0
                                vy = vy - 0.0050
                                
                            if vx > 0.1:
                                vx = 0.1
                            elif vx < -0.1:
                                vx = -0.1

                            vymax = (0.1**2-vx**2)**(1/2)

                            if vy > vymax:
                                vy = vymax
                            elif vy < -vymax:
                                vy = -vymax

                            vx = round(vx,3)
                            vy = round(vy,3)

                            totV = (vx**2+vy**2)**(1/2)
                            print('vx: {0} vy: {1} V: {2}'.format(vx, vy, totV))
                            robot.set_walk_velocity(vx,vy,0)

                    if flag_cam == 1:
                        robot.set_path_var(Sh = StepH, Fh = FootH)
                        flag_cam = 0

                    if flag_way == 1:
                        flag_way = 0
                        robot.set_walk_velocity(vx_way,vy_way,z_way)

            rospy.sleep(0.01)
except KeyboardInterrupt:
    pass