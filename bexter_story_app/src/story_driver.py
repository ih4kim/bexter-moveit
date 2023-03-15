#! /usr/bin/env python

import rospy
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
import vlc


JSON_NAME = "story_json.json"
IMAGE_FOLDER = "storyphotos"
RECORDING_FOLDER = "voicerecording"
COVER_FOLDER = "coverphoto"
COVER_FOLDER_IMG = "coverphoto.png"

WIDTH = 1280
HEIGHT = 720
SUCCEEDED = 3


class StoryDriver(object):
    def __init__(self):
        rospy.init_node("story_driver")

        # Connect to motion planning node
        rospy.loginfo("Waiting to connect to arm server...")
        self._arm_client = actionlib.SimpleActionClient('bexter_action', TargetAction)
        self._arm_client.wait_for_server()
        rospy.loginfo("Connected to arm client!")

        # TODO: Connect to vision node that determines coorindates of icons
        # Create server that recieves story path
        self._story_action_server = actionlib.SimpleActionServer("start_bexter_action", ReadStoryAction, execute_cb=self.story_driver_cb, auto_start = False)
        self._story_action_server.start()
        self._bexter_result = interfaces.msg.ReadStoryResult()
        self._image_pub = rospy.Publisher("story_image",Image, queue_size=10)
        self._bridge = CvBridge()
        self.logger_name = "story_driver"
        rospy.loginfo("Ready to read story!", logger_name=self.logger_name)
        self.black_image_msg = self._bridge.cv2_to_imgmsg(np.zeros((HEIGHT, WIDTH, 3), dtype = np.uint8), encoding="rgb8")
        self.image_message = self.black_image_msg
        self.display_image()

    def display_image(self):
        r = rospy.Rate(3) # 10hz
        while not rospy.is_shutdown():
            self._image_pub.publish(self.image_message)
            r.sleep()

    def get_icons(self):
        #TODO: detection code
        self.aac_icon_location = {
            "like": [0.271, 0.106, 0.021],
            "yellow": [0.168, -0.110, 0.015],
            "pink": [0.271, 0.106, 0.021],
            "green": [0.168, -0.110, 0.015],
            "red": [0.271, 0.106, 0.021],
            "purple": [0.168, -0.110, 0.015],
            "white": [0.271, 0.106, 0.021],
            "see": [0.168, -0.110, 0.015],
            "blue": [0.271, 0.106, 0.021],
            "black": [0.168, -0.110, 0.015],
            "what": [0.271, 0.106, 0.021],
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
        story_photo_timestamp = json.loads(metadata["storyPhotoTimes"])
        story_keywords = json.loads(metadata["transcriptOfKeywords"])
        story_keywords_timestamp = json.loads(metadata["transcriptOfKeywordTimes"])

        # Check if transcript of keywords exists in the dict
        unique_keywords = set(story_keywords)
        detected_words = set(self.aac_icon_location.keys())
        if not unique_keywords.issubset(detected_words):
            rospy.logerr("Keywords for this storybook not all detected!", logger_name=self.logger_name)
            return PathToStoryDataResponse(False)

        # Get .mp3 file
        recorder_folder = os.path.join(path, RECORDING_FOLDER)
        voice_files = glob.glob1(recorder_folder,"*.mp3")
        if len(voice_files) > 1:
            rospy.logerr("Multiple MP3 files found!")
            return PathToStoryDataResponse(False)
        if len(voice_files) == 0:
            rospy.logerr("No MP3 files found!")
            return PathToStoryDataResponse(False)
        
        # Send mp3 file to play
        voice_file_path = os.path.join(path, RECORDING_FOLDER, voice_files[0])
        rospy.loginfo("Found MP3 file: %s", voice_file_path)

        # Display cover photo
        dim = (WIDTH, HEIGHT)
        cv2_img = cv2.imread(os.path.join(path, COVER_FOLDER, COVER_FOLDER_IMG))
        cv2_img = cv2.resize(cv2_img, dim, interpolation = cv2.INTER_AREA)
        self.image_message = self._bridge.cv2_to_imgmsg(cv2_img, encoding="rgb8")

        # Get list of images from the image photo
        story_photos = sorted(os.listdir(os.path.join(path, IMAGE_FOLDER)))

        # Start recording
        player = vlc.MediaPlayer(voice_file_path)
        # wait until starts playing
        player.play()
        while not player.is_playing():
            continue
        # Now that story is playing, set timer and manage

        self._start_time = time.time()
        while player.is_playing() and not self._story_action_server.is_preempt_requested():
            elapsed_time = self.get_elapsed_time()
            # check if time passed to flip image
            if len(story_photo_timestamp) > 0 and elapsed_time > int(story_photo_timestamp[0]):
                # send page location
                page_path = os.path.join(path, IMAGE_FOLDER, story_photos.pop(0))
                cv2_img = cv2.imread(page_path)
                cv2_img = cv2.resize(cv2_img, dim, interpolation = cv2.INTER_AREA)
                self.image_message = self._bridge.cv2_to_imgmsg(cv2_img, encoding="rgb8")
                story_photo_timestamp.pop(0)
            if len(story_keywords_timestamp) > 0 and elapsed_time > int(story_keywords_timestamp[0]):
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
        # Change image to black default screen
        self.image_message = self.black_image_msg

        if self._story_action_server.is_preempt_requested():
            rospy.logwarn("Preempt request from app!", logger_name=self.logger_name)
            player.stop()
            # stop arm
            if self._arm_client.get_state() == actionlib_msgs.msg._GoalStatus.GoalStatus.ACTIVE:
                # cancel the current goal
                rospy.logwarn("Cancelling arm due to preempt from ap", logger_name=self.logger_name)
                self._arm_client.cancel_goal()
            bexter_goal = interfaces.msg.TargetGoal(position=[0.258, -0.141, 0.046])
            self._arm_client.send_goal(bexter_goal, done_cb=self.bexter_arm_cb)
            self._story_action_server.set_preempted(text="Preempted this story")
            return

        #Set robot back to original position
        bexter_goal = interfaces.msg.TargetGoal(position=[0.258, -0.141, 0.046])
        self._arm_client.send_goal(bexter_goal, done_cb=self.bexter_arm_cb)
        rospy.loginfo("Finished Story!")
        self._bexter_result.status = True
        self._story_action_server.set_succeeded(self._bexter_result)
    
    def story_read_cb(self, status, result):
        rospy.loginfo("Story Reader finished!", logger_name=self.logger_name)
        self._finished_story = result.status

    def bexter_arm_cb(self, status, result):
        rospy.loginfo("Arm reached icon!", logger_name=self.logger_name)

    def update_read_result(self, msg):
        self._reading_story = msg.status.status

    def get_elapsed_time(self):
        self._elapsed_time = time.time()-self._start_time
        return self._elapsed_time


if __name__ == "__main__":
    story_driver = StoryDriver()
    rospy.spin()


