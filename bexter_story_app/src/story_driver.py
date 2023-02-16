#! /usr/bin/env python

import rospy
from interfaces.srv import PathToStoryData, PathToStoryDataResponse
from interfaces.msg import TargetAction
import actionlib
import actionlib_msgs.msg._GoalStatus
import json
import os 
import glob
import time

JSON_NAME = "story_metadata.json"
IMAGE_FOLDER = "storyphotos"
RECORDING_FOLDER = "voicerecording"


class StoryDriver(object):
    def __init__(self):
        rospy.init_node("story_driver")

        # Connect to motion planning node
        self._arm_client = actionlib.SimpleActionClient('bexter_action', TargetAction)
        rospy.loginfo("Waiting to connect to arm server...")
        self._arm_client.wait_for_server()
        rospy.loginfo("Connected to arm client!")

        # TODO: Connect to vision node that determines coorindates of icons

        # Connect to story shower and reader node
        rospy.wait_for_service('story_reader')
        rospy.loginfo("Waiting to connect to story reader...")
        self._story_read_client = rospy.ServiceProxy('story_reader', PathToStoryData)
        rospy.loginfo("Connected to story reader!")

        rospy.wait_for_service('story_shower')
        rospy.loginfo("Waiting to connect to story shower...")
        self._story_show_client = rospy.ServiceProxy('story_shower', PathToStoryData)
        rospy.loginfo("Connected to story shower!")

        # Create server that recieves story path
        self._story_service = rospy.Service('story_driver', PathToStoryData, self.story_driver_cb)


    def story_driver_cb(self, req):
        path = req.path
        # Check if meta data exists
        metadata_path = os.path.join(path, JSON_NAME)
        if not os.path.isfile(metadata_path):
            rospy.logerr("Story metadata not found!")
            return PathToStoryDataResponse(False)
        metadata = json.load(metadata_path)
        duration = metadata["length"]
        page_timestamp = metadata["page_timestamp"]
        # Get .wav file
        recorder_folder = os.path.join(path, RECORDING_FOLDER)
        voice_files = glob.glob1(recorder_folder,"*.wav")
        if len(voice_files) > 1:
            rospy.logerr("Multiple WAV files found!")
            return PathToStoryDataResponse(False)
        if len(voice_files) == 0:
            rospy.logerr("No WAV files found!")
            return PathToStoryDataResponse(False)
        
        # Send wav file to play
        voice_file_path = os.path.join(path, RECORDING_FOLDER, voice_files[0])
        rospy.loginfo("Found WAV file: %s", voice_file_path)

        status = self._story_read_client(os.path.abspath(voice_file_path))
        if not status:
            rospy.logerr("Story Reader returned error!")
            return PathToStoryDataResponse(False)
        
        # Now that story is playing, set timer and manage 
        self._start_time = time.time()
        page_count = 0
        while self.get_elapsed_time() < duration:
            # check if time passed to flip image
            if len(page_timestamp) > 0 and self._elapsed_time > page_timestamp[0]:
                # send page location
                file_name = "{}.png".format(page_count)
                page_path = os.path.join(path, IMAGE_FOLDER, file_name)
                status = self._story_show_client(page_path)
                if not status:
                    rospy.logerr("Story Shower returned error!")
                    return PathToStoryDataResponse(False)
                page_count += 1
                page_timestamp.pop(0)
            
        rospy.loginfo("Finished Story!")
        # Send kill message
        status = self._story_show_client("-1")
        return PathToStoryDataResponse(True)
            
    def get_elapsed_time(self):
        self._elapsed_time = time.time()-self._start_time
        return self._elapsed_time


if __name__ == "__main__":
    story_driver = StoryDriver()
    rospy.spin()


