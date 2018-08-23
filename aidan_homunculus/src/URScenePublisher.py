#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def add_scene(self, timeout=4):

    cart_name = self.box_name
    scene = self.scene

    cart_pose = geometry_msgs.msg.PoseStamped()
    cart_pose.header.frame_id = "/world"
    cart_pose.pose.position.x = 0.4
    cart_pose.pose.position.y = 0.0
    cart_pose.pose.position.z = -0.1
    cart_pose.pose.orientation.w = 1.0
    cart_name = "Cart"
    scene.add_box(cart_name, cart_pose, size=(1, 0.51, 0.2))

    pole_pose = geometry_msgs.msg.PoseStamped()
    pole_pose.header.frame_id = "/world"
    pole_pose.pose.position.x = 0.36
    pole_pose.pose.position.y = -0.25
    pole_pose.pose.position.z = 0.55
    pole_pose.pose.orientation.w = 1.0
    pole_name = "Pole"
    scene.add_box(pole_name, pole_pose, size=(0.2, 0.6, 1.2))

    self.box_name=cart_name



  def attach_gripper(self, timeout=4):


    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    gripper_pose = geometry_msgs.msg.PoseStamped()
    gripper_pose.header.frame_id = "ee_link"
    gripper_pose.pose.position.x = .13
    gripper_pose.pose.position.y = 0
    gripper_pose.pose.position.z = 0
    gripper_pose.pose.orientation.w = 1.0
    gripper_name = "Gripper"
    scene.add_box(gripper_name, gripper_pose, size=(0.26, 0.2, 0.1))
    rospy.sleep(1)
    grasping_group = 'endeffector'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, gripper_name, touch_links=touch_links)



  def detach_gripper(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link


    scene.remove_attached_object(eef_link, name=box_name)

  def remove_gripper(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    scene.remove_world_object(box_name)



def main():
  try:
    publisher = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)
    publisher.add_scene()
    rospy.sleep(1)
    publisher.attach_gripper()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

