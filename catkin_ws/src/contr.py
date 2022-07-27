from xbox360controller import Xbox360Controller
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from my_message.msg import PathVar_n_cmdVel
from numpy import isnan

import sys
sys.path.append("/home/devlon/catkin_ws/src/simu_hexapod_stuff/src")
from hexapodC import HexapodC

flag_a = 0; flag_y = 0; flag_x = 0; flag_b = 0; flag_LB = 0; flag_RB = 0;flag_cam=0;flag_way=0
mode = 0; mode_selected = -1; IMU_toggle = 0; CAM_toggle = 0; NAVmode = 0; NAVmode_selected = 0  
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
        vx_way = vx if isnan(msg.linear.x) else msg.linear.x
        z_way = msg.angular.z

def stopwalking():
    robot.set_walk_velocity(0,0,0)
    IMU_toggle_pub.publish(0)
    rospy.sleep(1)

rospy.init_node('XboxController')

teleop_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
mode_pub = rospy.Publisher('/mode_selected',Float32,queue_size=1)
IMU_toggle_pub = rospy.Publisher('/IMU_toggle',Float32,queue_size=1)
Nav_pub = rospy.Publisher('/Nav_mode',Float32,queue_size=1)

rospy.Subscriber('/simple_hexapod/changed_vel_path_var',PathVar_n_cmdVel,vel_path_cb,queue_size=1)

rospy.on_shutdown(stopwalking)

def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def print_stuff():
    if NAVmode_selected == 0:
        print('\n\t NAVmode_selected: Nav off')
    elif NAVmode_selected == 1:
        print('\t NAVmode_selected: Heading controll on')
    elif NAVmode_selected == 2:
        print('\t NAVmode_selected: Waypoint nav on')

    if IMU_toggle == 0:
        print('\t\t IMU toggle: off')
    elif IMU_toggle == 1:
        print('\t\t IMU toggle: on')

    if CAM_toggle == 0:
        print('\t\t Cam toggle: off')
    elif CAM_toggle == 1:
        print('\t\t Cam toggle: on')   
    
    if flag_way == 1:
        print('\t\t vx: {0} vy: {1} V: {2} body height: {3} Turn Ang: {4} \n\t\t Foot height: {5} Step height: {6}'.format(vx_way, vy_way, (vx_way**2+vy_way**2)**(1/2), bh, z_way, FootH, StepH))
    else:
        print('\t\t vx: {0} vy: {1} V: {2} body height: {3} Turn Ang: {4} \n\t\t Foot height: {5} Step height: {6}'.format(vx, vy, totV, bh, turnAng, FootH, StepH))

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
    print("mode {0}" .format(mode))
def on_mode_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_select_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global mode_selected, NAVmode_selected, NAVmode
    if mode_selected != mode:
        if totV == 0.0 and IMU_toggle == 0 and start == 1 and NAVmode_selected == 0:
            mode_selected = mode
            NAVmode = 0
            print('mode selected: {0}'.format(mode_selected))
            mode_pub.publish(data=mode_selected)
    if mode_selected == 2:
        NAVmode_selected = NAVmode
        Nav_pub.publish(data=NAVmode_selected)
        print_stuff()#print('\t NAVmode selected: {0}'.format(NAVmode_selected))
def on_select_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_start_button_pressed(button):
    #print('Button {0} was pressed'.format(button.name))
    global start
    if start == 0:
        start = 1
        mode_pub.publish(data=-1)
        robot.set_walk_velocity(0,0,0)
        IMU_toggle_pub.publish(-1)
def on_start_button_released(button):
    #print('Button {0} was released'.format(button.name))
    pass

def on_Laxis_moved(axis):
    global axX, axY, flag_Lstick
    axX = -axis.y
    axY = axis.x
    flag_Lstick = 1

def on_left_stick_pressed(stick):
    global IMU_toggle
    IMU_toggle = 1 if IMU_toggle == 0 else 0
    IMU_toggle_pub.publish(IMU_toggle)
    print_stuff()
def on_left_stick_released(stick):
    pass

def on_right_stick_pressed(stick):
    global CAM_toggle
    CAM_toggle = 1 if CAM_toggle == 0 else 0
    print_stuff()
    if CAM_toggle == 0:
        robot.set_path_var(Sh = [50, 50, 50, 50, 50, 50], Fh = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
def on_right_stick_released(stick):
    pass

def on_dpad_moved(axis):
    global NAVmode
    if mode_selected == 2:
        if axis.y == 1:
            NAVmode = NAVmode + 1
            if NAVmode > 2: NAVmode = 0
        elif axis.y == -1:
            NAVmode = NAVmode - 1
            if NAVmode < 0: NAVmode = 2
        print("\t NAVmode {0} " .format(NAVmode))

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

        controller.button_thumb_l.when_pressed = on_left_stick_pressed
        controller.button_thumb_l.when_released = on_left_stick_released

        controller.button_thumb_r.when_pressed = on_right_stick_pressed
        controller.button_thumb_r.when_released = on_right_stick_released

        controller.hat.when_moved = on_dpad_moved

        # Left and right axis move event
        controller.axis_l.when_moved = on_Laxis_moved
        #controller.axis_r.when_moved = on_axis_moved

        #signal.pause()
        while not rospy.is_shutdown() and controller._event_thread.is_alive():
            
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
                                turnAng = turnAng + 15
                            elif flag_x == 1:
                                flag_x = 0
                                turnAng = turnAng - 0.25

                            if turnAng > 30:
                                turnAng = 30
                            elif turnAng < -30:
                                turnAng = -30

                            turnAng = round(turnAng,2)

                            print_stuff()#print('body height: {0} Turn Ang: {1}'.format(bh, turnAng))
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
                            print_stuff()#print('vx: {0} vy: {1} V: {2}'.format(vx, vy, totV))
                            robot.set_walk_velocity(vx,vy,turnAng)

                    if flag_cam == 1 and CAM_toggle == 1:
                        robot.set_path_var(Sh = StepH, Fh = FootH)
                        print_stuff()
                        flag_cam = 0

                    if flag_way == 1:
                        robot.set_walk_velocity(vx_way,vy_way,z_way)
                        print_stuff()
                        flag_way = 0

            rospy.sleep(0.01)
except KeyboardInterrupt:
    pass
except:
    print("no")