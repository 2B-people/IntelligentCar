#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from roborts_msgs.msg import TwistAccel

import sys, select, termios, tty

msg = """
Control The Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

-/+ : increase/decrease max speeds by 10%
</> : increase/decrease only linear speed by 10%
{/} : increase/decrease only angular speed by 10%
space key, s : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'e':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        'q':(1,1),
        'x':(-1,0),
        'c':(-1,1),
        'z':(-1,-1),
           }

speedBindings={
        '=':(1.1,1.1),
        '-':(.9,.9),
        ']':(1.1,1),
        '[':(.9,1),
        '.':(1,1.1),
        ',':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robot_teleop')
    pub1 = rospy.Publisher('car/cmd_vel', Twist, queue_size=5)
    pub2 = rospy.Publisher('cmd_vel_acc', TwistAccel, queue_size=8)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 's' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist1 = Twist()
            twist2 = TwistAccel()

            twist1.linear.x = control_speed; twist1.linear.y = 0; twist1.linear.z = 0
            twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = control_turn

            twist2.twist.linear.x = control_speed;twist2.twist.linear.y = 0; twist2.twist.linear.z = 0
            twist2.twist.angular.x = 0; twist2.twist.angular.y = 0; twist2.twist.angular.z = control_turn
            twist2.accel.linear.x = 0.3; twist2.accel.angular.x = 0.3; 
            pub1.publish(twist1)
            pub2.publish(twist2)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e

    finally:
        twist1 = Twist()
        twist2 = TwistAccel()
            
        twist1.linear.x = control_speed; twist1.linear.y = 0; twist1.linear.z = 0
        twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = control_turn

        twist2.twist.linear.x = 0;twist2.twist.linear.y = 0; twist2.twist.linear.z = 0
        twist2.twist.angular.x = 0; twist2.twist.angular.y = 0; twist2.twist.angular.z = 0
        twist2.accel.linear.x = 0; twist2.accel.angular.x = 0; 
        pub1.publish(twist1)
        pub2.publish(twist2)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)