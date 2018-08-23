#!/usr/bin/env python
import rospy
import tf

from moveit_planner.UR5Arm import *

from moveit_msgs.msg import *
from moveit_msgs.srv import *

def main(args):
    arm = UR5Arm()
    arm.__init__()
    rospy.sleep()

if __name__ == '__main__':
    main(sys.argv)
