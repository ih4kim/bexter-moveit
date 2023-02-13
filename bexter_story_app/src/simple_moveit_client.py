

#! /usr/bin/env python

import rospy
import interfaces.msg
import sys
# Brings in the SimpleActionClient
import actionlib
import actionlib_msgs.msg._GoalStatus



def bexter_client(x, y, z):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('bexter_action', interfaces.msg.TargetAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = interfaces.msg.TargetGoal(position=[x,y,z])

    # Sends the goal to the action server.
    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
    rospy.sleep(1)
    # Waits for the server to finish performing the action.
    while client.get_state() == actionlib_msgs.msg._GoalStatus.GoalStatus.ACTIVE:
        rospy.loginfo("Server Active")

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def done_cb(status, result):
    if status == actionlib_msgs.msg._GoalStatus.GoalStatus.SUCCEEDED:
        rospy.loginfo("Action completed successfully")
        rospy.loginfo("Action returned: %d", result.status)
    elif status == actionlib_msgs.msg._GoalStatus.GoalStatus.ABORTED:
        rospy.logerr("Action not completed successfully with id: %d", status)
        
def feedback_cb(feedback):
    rospy.loginfo("Error to target: %.2f", feedback.error)


# if __name__ == '__main__':
#     try:
#         # Initializes a rospy node so that the SimpleActionClient can
#         # publish and subscribe over ROS.
#         rospy.init_node('fibonacci_client_py')
#         result = fibonacci_client()
#         print("Result:", ', '.join([str(n) for n in result.sequence]))
#     except rospy.ROSInterruptException:
#         print("program interrupted before completion", file=sys.stderr)

if __name__ == "__main__":
    if len(sys.argv) == 4:
        rospy.init_node("bextersimpleclient")
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        print("Requesting position: [%.2f, %.2f, %.2f]"%(x, y, z))
        print("Target reached: %d"% (bexter_client(x,y,z).status))
   