#!/usr/bin/env python
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
from moveit_msgs.msg import ExecuteTrajectoryActionResult
## END_SUB_TUTORIAL

POSITION_TOLEREANCE = 0.02
PENDING             = 0
ACTIVE              = 1
PREEMPTED           = 2
SUCCEEDED           = 3
ABORTED             = 4
REJECTED            = 5
PREEMPTING          = 6
RECALLING           = 7
RECALLED            = 8
LOST                = 9  

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

  def __init__(self, rviz_mode=True):
    super(BexterMoveGroup, self).__init__()
    self._action_name = "bexter_action"
    moveit_commander.roscpp_initialize(sys.argv)
    self._as = actionlib.SimpleActionServer(self._action_name, interfaces.msg.TargetAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self._rviz_mode = rviz_mode
    self._result_subscriber = rospy.Subscriber("/execute_trajectory/result", ExecuteTrajectoryActionResult, self.update_result)
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
    group.set_max_acceleration_scaling_factor(0.8)
    group.set_max_velocity_scaling_factor(0.8)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    if rviz_mode:
      display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.display_trajectory_publisher = display_trajectory_publisher


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
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.execute_status = -1
    self.error = 0

  def update_result(self, msg):
    self.execute_status=msg.status.status;
    #ROS_INFO("%i execute status",self.execute_status)
    rospy.loginfo("Action Result: %d", self.execute_status)

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
      self.execute_status = -1
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = self.group.get_current_state()
      display_trajectory.trajectory.append(plan[1])
      # Publish
      if self._rviz_mode:
        self.display_trajectory_publisher.publish(display_trajectory);
      result = self.group.execute(plan_msg = plan[1], wait = False)
      while self.execute_status < 2:
        # Update feedback using error
        self._feedback.error = self.get_error(position)
        self._as.publish_feedback(self._feedback)
      self.group.stop()
      # match self.execute_status:
      #   case ABORTED:
      #     self._as.set_aborted(False, "Unsucessful during plan/execute")
      #   case SUCCEEDED:
      #     self._result.status = True
      #     rospy.loginfo("Reached Target!")
      #     self._as.set_succeeded(self._result)
      #   case PREEMPTED:
      #     self._as.set_preempted(text="Preempted this action") 
      if self.execute_status == ABORTED:
        self._as.set_aborted(False, "Unsucessful during plan/execute")
      elif self.execute_status == SUCCEEDED:
        self._result.status = True
        rospy.loginfo("Reached Target!")
        self._as.set_succeeded(self._result)
      elif self.execute_status == PREEMPTED:
        self._as.set_preempted(text="Preempted this action") 
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
    rviz_mode = rospy.get_param('/bextermovegroup/rviz_mode')
    tutorial = BexterMoveGroup(rviz_mode)
    rospy.spin()
  except rospy.ROSInterruptException:
    return

if __name__ == '__main__':
  main()