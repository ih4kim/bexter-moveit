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
from std_msgs.msg import Float32
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

ICON_WIDTH = 8
ICON_HEIGHT = 6

class StoryDriver(object):
    def __init__(self):
        rospy.init_node("story_driver")

        # Connect to motion planning node
        rospy.loginfo("Waiting to connect to arm server...")
        self._arm_client = actionlib.SimpleActionClient('bexter_action', TargetAction)
        self._arm_client.wait_for_server()
        rospy.loginfo("Connected to arm client!")
        # (0,0) is top left when facing bexter
        self._icon_cooridinate = np.zeros((ICON_HEIGHT,ICON_WIDTH,3))
        # y_axis = [-0.282, -0.250, -0.214, -0.182, -0.153, -0.127, -0.100, -0.066]
        # x_axis = [0.182, 0.214, 0.246, 0.275, 0.313, 0.337]
        y_axis = [-0.281, -0.209, -0.135, -0.074]
        x_axis = [0.2, 0.27, 0.330]
        for i, x_value in enumerate(x_axis):
            for j, y_value in enumerate(y_axis):
                self._icon_cooridinate[i][j] = [x_value, y_value, 0.01]
        
        print(self._icon_cooridinate)


        # TODO: Connect to vision node that determines coorindates of icons
        # Create server that recieves story path
        self._story_action_server = actionlib.SimpleActionServer("start_bexter_action", ReadStoryAction, execute_cb=self.story_driver_cb, auto_start = False)
        self._story_action_server.start()
        self._bexter_result = interfaces.msg.ReadStoryResult()
        self._image_pub = rospy.Publisher("story_image",Image, queue_size=10)
        self._curr_time_pub = rospy.Publisher("current_time", Float32, queue_size=10)
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
            # "like": [0, 3],
            # "red": [4, 0],
            # "blue": [4, 3],
            # "green": [4, 2],
            # "purple": [4, 4],
            # "white": [4, 7],
            # "see": [0, 6],
            # "blue": [4, 3],
            # "black": [4, 6],
            # "what": [2, 0],
            # "pink": [4, 5],
            # "yellow": [4, 1],
            "like": [0, 2],
            "red": [2, 0],
            "blue": [2, 2],
            "green": [2, 3],
            "purple": [1, 0],
            "white": [1, 3],
            "see": [0, 3],
            "black": [1, 2],
            "what": [0, 1],
            "pink": [1, 1],
            "yellow": [2, 1],
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
        total_words = len(story_keywords)
        # Check if transcript of keywords exists in the dict
        unique_keywords = set(story_keywords)
        print(unique_keywords)
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
        self.image_message = self._bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
        self._failed_attempt = 0
        # Get list of images from the image photo
        story_photos = sorted(os.listdir(os.path.join(path, IMAGE_FOLDER)))
        bexter_goal = interfaces.msg.TargetGoal(position=[0.272, -0.17, 0.075], reset=True, target_time=-1)
        self._target_reached = False
        self._arm_client.send_goal(bexter_goal, done_cb=self.bexter_arm_cb)
        while not self._target_reached:
            continue
            
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
                self.image_message = self._bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")
                story_photo_timestamp.pop(0)
            # Continusouly feed story word and timestamp to action when 
            if self._target_reached and len(story_keywords_timestamp)>0:
                # Get position
                self._target_reached = False
                y_grid, x_grid = self.aac_icon_location[story_keywords[0]]
                position = self._icon_cooridinate[y_grid][x_grid].tolist()
                bexter_goal = interfaces.msg.TargetGoal(position=position, reset=False, target_time=int(story_keywords_timestamp[0]))
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
        successful_count = total_words-self._failed_attempt
        success_rate = successful_count/total_words
        rospy.loginfo("Successful attempts: %d/%d = %.2f", successful_count, total_words, success_rate)
        self._bexter_result.status = True
        self._story_action_server.set_succeeded(self._bexter_result)
    
    def story_read_cb(self, status, result):
        rospy.loginfo("Story Reader finished!", logger_name=self.logger_name)
        self._finished_story = result.status

    def bexter_arm_cb(self, status, result):
        if status == actionlib_msgs.msg._GoalStatus.GoalStatus.SUCCEEDED:
            rospy.loginfo("Action completed successfully")
            rospy.loginfo("Action returned: %d", result.status)
        elif status == actionlib_msgs.msg._GoalStatus.GoalStatus.ABORTED:
            rospy.logerr("Action aborted with id: %d", status)
            self._failed_attempt += 1
        elif status == actionlib_msgs.msg._GoalStatus.GoalStatus.PREEMPTED:
            rospy.logerr("Action preempted with id: %d", status)
            self._failed_attempt += 1
        self._target_reached = True

    def update_read_result(self, msg):
        self._reading_story = msg.status.status

    def get_elapsed_time(self):
        self._elapsed_time = time.time()-self._start_time
        self._curr_time_pub.publish(self._elapsed_time)
        return self._elapsed_time


if __name__ == "__main__":
    story_driver = StoryDriver()
    rospy.spin()


