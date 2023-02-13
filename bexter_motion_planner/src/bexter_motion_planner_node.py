#!/usr/bin/env python
## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import interfaces.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import pdb
## END_SUB_TUTORIAL

POSITION_TOLEREANCE = 0.01

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

class BexterMoveGroup(object):
  _feedback = interfaces.msg.TargetFeedback()
  _result = interfaces.msg.TargetResult()

  def __init__(self):
    super(BexterMoveGroup, self).__init__()
    self._action_name = "bexter_action"
    moveit_commander.roscpp_initialize(sys.argv)
    self._as = actionlib.SimpleActionServer(self._action_name, interfaces.msg.TargetAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
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

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  
  def execute_cb(self, goal):
    # Get end effector pose from goal
    rospy.loginfo("Recieved target: [%.2f, %.2f, %.2f]", goal.position[0], goal.position[1], goal.position[2])
    curr_position = self.group.get_current_pose().pose.position
    rospy.loginfo("Current Position: [%.2f, %.2f, %.2f]", curr_position.x, curr_position.y, curr_position.z)
    position = goal.position
    self.group.set_position_target(position)
    self.group.set_goal_position_tolerance(POSITION_TOLEREANCE)
    self.group.set_planning_time(10)
    plan = self.group.plan()
    # Check if plan contains points
    
    if plan[0]:
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = self.group.get_current_state()
      display_trajectory.trajectory.append(plan[1])
      # Publish
      self.display_trajectory_publisher.publish(display_trajectory);
      result = self.group.execute(plan_msg = plan[1], wait = False)
      while self.get_error(position) > POSITION_TOLEREANCE:
        # Update feedback using error
        self._feedback.error = self.error
        self._as.publish_feedback(self._feedback)
      self.group.stop()
      self._result.status = True
      rospy.loginfo("Reached Target!")
      self._as.set_succeeded(self._result) 

    else:
      rospy.logerr("Trajectory is empty. Planning was unsuccessful.") 
      self._as.set_aborted(False, "Trajectory is empty. Planning was unsuccessful.")



  def get_error(self, position):
    robot_pose = self.group.get_current_pose().pose.position
    self.error = math.sqrt(pow(robot_pose.x-position[0],2)+pow(robot_pose.y-position[1],2)+pow(robot_pose.z-position[2],2))
    return self.error
    
def main():
  try:
    rospy.init_node("bextermovegroup")
    tutorial = BexterMoveGroup()
    rospy.spin()
  except rospy.ROSInterruptException:
    return

if __name__ == '__main__':
  main()