

#! /usr/bin/env python

import rospy
import interfaces.msg
import sys
# Brings in the SimpleActionClient
import actionlib
import actionlib_msgs.msg._GoalStatus
import time
from std_msgs.msg import Float32
import numpy as np
# def bexter_client(x, y, z):
#     # Creates the SimpleActionClient, passing the type of the action
#     # (FibonacciAction) to the constructor.
#     client = actionlib.SimpleActionClient('bexter_action', interfaces.msg.TargetAction)

#     # Waits until the action server has started up and started
#     # listening for goals.
#     client.wait_for_server()

#     # Creates a goal to send to the action server.
#     goal = interfaces.msg.TargetGoal(position=[x,y,z], reset=True, target_time=-1)

#     # Sends the goal to the action server.
#     client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
#     rospy.sleep(1)
#     # Waits for the server to finish performing the action.
#     while client.get_state() == actionlib_msgs.msg._GoalStatus.GoalStatus.ACTIVE:
#         rospy.loginfo("Server Active")

#     # Prints out the result of executing the action
#     return client.get_result()  # A FibonacciResult

# def done_cb(status, result):
#     if status == actionlib_msgs.msg._GoalStatus.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Action completed successfully")
#         rospy.loginfo("Action returned: %d", result.status)
#     elif status == actionlib_msgs.msg._GoalStatus.GoalStatus.ABORTED:
#         rospy.logerr("Action not completed successfully with id: %d", status)
        
# def feedback_cb(feedback):
#     rospy.loginfo("Error to target: %.2f", feedback.error)

ICON_HEIGHT = 4
ICON_WIDTH = 5

class BexterUnitTest(object):

    def __init__(self):
        self._icon_cooridinate = np.zeros((ICON_HEIGHT,ICON_WIDTH,3))
        # y_axis = [-0.282, -0.250, -0.214, -0.182, -0.153, -0.127, -0.100, -0.066]
        # x_axis = [0.182, 0.214, 0.246, 0.275, 0.308, 0.337]

        # y_axis = [-0.281, -0.209, -0.135, -0.074]
        # x_axis = [0.2, 0.27, 0.330]
        # 1- L 4 - R 5 - R, 1 - L, 2- TL, 4 - R, 1 - TL, 2 -TTL, 4 - TR, 5 - T
        y_axis = [-0.30, -0.235, -0.176, -0.11, -0.05]
        x_axis = [0.20, 0.25, 0.31, 0.355]
        for i, x_value in enumerate(x_axis):
            for j, y_value in enumerate(y_axis):
                self._icon_cooridinate[i][j] = [x_value, y_value, 0.02]
       
        # Flatten array and make array of points
        self._first_test_points = np.reshape(self._icon_cooridinate, (ICON_HEIGHT*ICON_WIDTH, 3)).tolist()
        self._first_test_points_ts = []
        # First test points will have 10 seconds between icons
        for i in range(0, ICON_HEIGHT*ICON_WIDTH):
            self._first_test_points_ts.append((i+1)*6)
        print("Running tests on these points: ")
        print(self._first_test_points)
        self._client = actionlib.SimpleActionClient('bexter_action', interfaces.msg.TargetAction)
        self._client.wait_for_server()
        self._curr_time_pub = rospy.Publisher("current_time", Float32, queue_size=10)
        rospy.loginfo("Connected to action server!")

    def run_test(self):
        self._doing_action = False
        start_time = time.time()
        while self.get_elapsed_time(start_time) < self._first_test_points_ts[-1]:
            if self._doing_action and self._first_test_points_ts[0] < self._elapsed_time:
                rospy.logwarn("Currently doing action! Missing target from client!")
                self._first_test_points_ts.pop(0)
                self._first_test_points.pop(0)
            elif not self._doing_action:
                position = self._first_test_points.pop(0)
                target_time = self._first_test_points_ts.pop(0)
                goal = interfaces.msg.TargetGoal(position=position, reset=False, target_time=target_time)
                self._doing_action = True
                self._client.send_goal(goal, done_cb=self.done_cb)
                if len(self._first_test_points) == 0:
                    break

        while self._doing_action:
            continue
    
    def done_cb(self, status, result):
        if status == actionlib_msgs.msg._GoalStatus.GoalStatus.SUCCEEDED:
            rospy.loginfo("Action completed successfully")
            rospy.loginfo("Action returned: %d", result.status)
        elif status == actionlib_msgs.msg._GoalStatus.GoalStatus.ABORTED:
            rospy.logerr("Action not completed successfully with id: %d", status)
        self._doing_action = False

    def get_elapsed_time(self, start_time):
        elapsed_time = time.time()-start_time
        self._curr_time_pub.publish(elapsed_time)
        self._elapsed_time = elapsed_time
        return elapsed_time


# if __name__ == "__main__":
#     if len(sys.argv) == 4:
#         rospy.init_node("bextersimpleclient")
#         x = float(sys.argv[1])
#         y = float(sys.argv[2])
#         z = float(sys.argv[3])
#         print("Requesting position: [%.2f, %.2f, %.2f]"%(x, y, z))
#         print("Target reached: %d"% (bexter_client(x,y,z).status))
   
if __name__ == "__main__":
    rospy.init_node("bextersimpleclient")
    test = BexterUnitTest()
    test.run_test()
    rospy.spin()
