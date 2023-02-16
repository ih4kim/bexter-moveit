#! /usr/bin/env python

import rospy
from interfaces.srv import PathToStoryData, PathToStoryDataResponse
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class StoryReader(object):
    def __init__(self):
        # Create service to get path
        rospy.init_node("story_reader_node")
        self._service = rospy.Service('story_reader', PathToStoryData, self.read_story)
        self._sound_client = SoundClient()
    
    def read_story(self, req):
        file_path = req.path
        if self._sound_client._playing:
            self._sound_client.stopAll()
        # Play sound
        volume = 1
        self._sound_client.playWave(file_path, volume)
        return PathToStoryDataResponse(True)

if __name__ == "__main__":
    story_reader = StoryReader()
    rospy.spin()




