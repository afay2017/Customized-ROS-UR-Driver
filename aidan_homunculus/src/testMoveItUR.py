#!/usr/bin/env python
import rospy
import tf

from moveit_planner.UR5Arm import *

from moveit_msgs.msg import *
from moveit_msgs.srv import *

class GraspTest():
    def __init__(self):
        # publishers and subscribers


        #self.listener = tf.TransformListener()

        #rospy.sleep(2)

        #try:
        #    (t, q) = self.listener.lookupTransform('/world', '/camera_rgb_optical_frame', rospy.Time(0))
        #    # better way to do this
        #    self.wTc_pose = Pose()
        #    self.wTc_pose.position.x = t[0]
        #    self.wTc_pose.position.y = t[1]
        #    self.wTc_pose.position.z = t[2]

        #    self.wTc_pose.orientation.x = q[0]
        #    self.wTc_pose.orientation.y = q[1]
        #    self.wTc_pose.orientation.z = q[2]
        #    self.wTc_pose.orientation.w = q[3]

        #    print self.wTc_pose
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         #   rospy.logerr('There is no wTc available')

        #self._pub_ps = rospy.Publisher('planning_scene', PlanningScene)
        #rospy.wait_for_service('/get_planning_scene', 10.0)
        #self._srv_gps = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)


    def run(self):
        print "starting run"

        # move to position to spot objects
        #success = self.arm.MoveToSpotLoc()

        #if not success:
         #   print "Error moving to spot location!"
         #   return False

        #print('##### press any key to proceed #####')
        #raw_input()

        #success = self.arm.MoveToBin('right')

def main(args):
    rospy.init_node('grasping_demo', anonymous=True)
    arm = UR5Arm()
    arm.__init__()
    g = GraspTest()
    shutdown = False

    try:
        while not rospy.is_shutdown():
            no_error = g.run()
            # no_error = g.run_cluttered()
            # shutdown = not no_error
            print('##### press any key to proceed #####')
            raw_input()
    except KeyboardInterrupt:
        print "grasping_demo Shutting down"

if __name__ == '__main__':
    main(sys.argv)
