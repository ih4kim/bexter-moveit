#! /usr/bin/env python

import rospy
import actionlib
from interfaces.msg import ReadStoryAction, ReadStoryResult
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class StoryReader(object):
    def __init__(self):
        # Create service to get path
        rospy.init_node("story_reader_node")
        #self._service = rospy.Service('story_reader', PathToStoryData, self.read_story)
        self._action_name = "story_reader_action"
        self._as = actionlib.SimpleActionServer(self._action_name, ReadStoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._result = ReadStoryResult()
        self._sound_client = SoundClient(blocking=True)
    
    def execute_cb(self, goal):
        path_to_wav = goal.path
        if self._sound_client._playing:
            self._sound_client.stopAll()
        volume = 1
        self._sound_client.playWave(path_to_wav, volume)
        self._result.status = True
        rospy.loginfo("Finished wav recording!", logger_name="story_reader_node")
        self._as.set_succeeded(self._result) 

if __name__ == "__main__":
    story_reader = StoryReader()
    rospy.spin()




