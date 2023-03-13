#! /usr/bin/env python

import rospy
from interfaces.srv import PathToStoryData, PathToStoryDataResponse
import interfaces.msg
from interfaces.msg import TargetAction, ReadStoryAction
import actionlib
import actionlib_msgs.msg._GoalStatus
import json
import os 
import glob
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

JSON_NAME = "story_metadata.json"
IMAGE_FOLDER = "storyphotos"
RECORDING_FOLDER = "voicerecording"

WIDTH = 1280
HEIGHT = 720

class StoryDriver(object):
    def __init__(self):
        rospy.init_node("story_driver")

        # Connect to motion planning node
        rospy.loginfo("Waiting to connect to arm server...")
        self._arm_client = actionlib.SimpleActionClient('bexter_action', TargetAction)
        self._arm_client.wait_for_server()
        rospy.loginfo("Connected to arm client!")

        # TODO: Connect to vision node that determines coorindates of icons

        # Connect to story shower and reader node
        rospy.loginfo("Waiting to connect to story reader...")
        self._story_read_client = actionlib.SimpleActionClient('story_reader_action', ReadStoryAction)
        self._story_read_client.wait_for_server()
        rospy.loginfo("Connected to story reader!")

        # rospy.wait_for_service('story_shower')
        # rospy.loginfo("Waiting to connect to story shower...")
        # self._story_show_client = rospy.ServiceProxy('story_shower', PathToStoryData)
        # rospy.loginfo("Connected to story shower!")

        # Create server that recieves story path
        self._story_service = rospy.Service('story_driver', PathToStoryData, self.story_driver_cb)
        self._image_pub = rospy.Publisher("story_image",Image, queue_size=10)
        self._bridge = CvBridge()
        self._finished_story = False
        self.logger_name = "story_driver"
        rospy.loginfo("Ready to read story!", logger_name=self.logger_name)
        self.image_message = self._bridge.cv2_to_imgmsg(np.zeros((HEIGHT, WIDTH, 3), dtype = np.uint8), encoding="rgb8")
        self.display_image()

        

    def display_image(self):
        r = rospy.Rate(3) # 10hz
        while not rospy.is_shutdown():
            self._image_pub.publish(self.image_message)
            r.sleep()

    def get_icons(self):
        #TODO: detection code
        self.aac_icon_location = {
            "Dog": [0.271, 0.106, 0.021],
            "Cat": [0.168, -0.110, 0.015]
        }

    def story_driver_cb(self, req):
        # First scan the page such that it can determine all the locations
        # For now, hard-coded locations for testing
        self.get_icons()
        path = req.path
        # Check if meta data exists
        metadata_path = os.path.join(path, JSON_NAME)
        if not os.path.isfile(metadata_path):
            rospy.logerr("Story metadata not found!")
            return PathToStoryDataResponse(False)
        with open(metadata_path) as f:
            metadata = json.load(f)
        story_photo_timestamp = metadata["storyPhotoTimes"]
        story_keywords = metadata["transcriptOfKeywords"]
        story_keywords_timestamp = metadata["transcriptOfKeyWordsTimes"]

        # Check if transcript of keywords exists in the dict
        unique_keywords = set(story_keywords)
        detected_words = set(self.aac_icon_location.keys())
        if not unique_keywords.issubset(detected_words):
            rospy.logerr("Keywords for this storybook not all detected!", logger_name=self.logger_name)
            return PathToStoryDataResponse(False)

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

        read_story_goal = interfaces.msg.ReadStoryGoal(path=voice_file_path)
        self._finished_story = False
        self._story_read_client.send_goal(read_story_goal, done_cb=self.story_read_cb)

        # status = self._story_read_client(os.path.abspath(voice_file_path))
        # if not status:
        #     rospy.logerr("Story Reader returned error!")
        #     return PathToStoryDataResponse(False)
        
        # Now that story is playing, set timer and manage 
        self._start_time = time.time()
        page_count = 0
        dim = (WIDTH, HEIGHT)
        while not self._finished_story:
            self.get_elapsed_time()
            # check if time passed to flip image
            if len(story_photo_timestamp) > 0 and self._elapsed_time > story_photo_timestamp[0]:
                # send page location
                file_name = "{}.png".format(page_count)
                page_path = os.path.join(path, IMAGE_FOLDER, file_name)
                cv2_img = cv2.imread(page_path)
                cv2_img = cv2.resize(cv2_img, dim, interpolation = cv2.INTER_AREA)
                self.image_message = self._bridge.cv2_to_imgmsg(cv2_img, encoding="rgb8")
                page_count += 1
                story_photo_timestamp.pop(0)
            if len(story_keywords_timestamp) > 0 and self._elapsed_time > story_keywords_timestamp[0]:
                # get keyword location
                position = self.aac_icon_location[story_keywords[0]]
                bexter_goal = interfaces.msg.TargetGoal(position=position)
                # check if bexter still active
                if self._arm_client.get_state() == actionlib_msgs.msg._GoalStatus.GoalStatus.ACTIVE:
                    # cancel the current goal
                    rospy.logwarn("Cancelling current icon, arm too slow!", logger_name=self.logger_name)
                    self._arm_client.cancel_goal()
                self._arm_client.send_goal(bexter_goal, done_cb=self.bexter_arm_cb)
                story_keywords_timestamp.pop(0)
                story_keywords.pop(0)
            
        #Set robot back to original position
        bexter_goal = interfaces.msg.TargetGoal(position=[0.258, -0.141, 0.046])
        self._arm_client.send_goal(bexter_goal, done_cb=self.bexter_arm_cb)
        rospy.loginfo("Finished Story!")
        return PathToStoryDataResponse(True)
    
    def story_read_cb(self, status, result):
        rospy.loginfo("Story Reader finished!", logger_name=self.logger_name)
        self._finished_story = result.status

    def bexter_arm_cb(self, status, result):
        rospy.loginfo("Arm reached icon!", logger_name=self.logger_name)


    def get_elapsed_time(self):
        self._elapsed_time = time.time()-self._start_time
        return self._elapsed_time


if __name__ == "__main__":
    story_driver = StoryDriver()
    rospy.spin()


