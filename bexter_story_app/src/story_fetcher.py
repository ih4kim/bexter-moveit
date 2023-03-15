#! /usr/bin/env python

import rospy
import os
import json
from dotenv import load_dotenv
import time
import asyncio
import aiohttp
import socketio
import shutil
import actionlib
import actionlib_msgs.msg._GoalStatus
import interfaces.msg
from interfaces.msg import ReadStoryAction

# Hardcode location on harddrive
base_dir = '/home/ubuntu/Documents/story_folder'


class StoryFetcher(object):
    def __init__(self):
        rospy.init_node("story_fetcher_node")
        self.logger_name = "story_fetcher_node"
        self._sio = socketio.Client()
        self._base_url = "http://3.134.99.13:5000/"#os.getenv("BASE_URL")
        rospy.loginfo("Connecting to start_bexter_action", logger_name=self.logger_name)
        self._bexter_client = actionlib.SimpleActionClient('start_bexter_action', ReadStoryAction)
        self._bexter_client.wait_for_server()
        rospy.loginfo("Connected to start_bexter_action!", logger_name=self.logger_name)
        self.setup()

    def setup(self):
        self.call_backs()
        self._sio.connect(self._base_url)
        self._sio.emit('join', 33)
        rospy.loginfo("successfully joined socket 33")

    def call_backs(self):
        @self._sio.on('play')
        def on_message(data):
            if data.get("command") == "play":
                id = data.get("storyId")
                asyncio.run(self.main(id))
                rospy.loginfo("Finished getting story, passing story to action in folder %s", base_dir)
                goal = interfaces.msg.ReadStoryGoal(path=base_dir)
                self._bexter_client.send_goal(goal, done_cb=self.story_fetch_cb)
            else:
                if self._bexter_client.get_state() == actionlib_msgs.msg._GoalStatus.GoalStatus.ACTIVE:
                    # cancel the current goal
                    rospy.logwarn("Cancel request from user!", logger_name=self.logger_name)
                    self._bexter_client.cancel_goal()
    
    def story_fetch_cb(self, status, result):
        rospy.loginfo("Bexter finished!", logger_name=self.logger_name)

    async def main(self, id):
        start_time = time.perf_counter()
        url = self._base_url + "stories/robots/" + str(id)
        headers = {"x-auth-token": "33"}

        #Fetch Data
        async with aiohttp.ClientSession(headers=headers) as session:
            async with session.get(url) as response:
                status_code = response.status;
                data = await response.json()

        print("Status Code: ", status_code)

        if (status_code == 400 or status_code == 500):
            print(data["msg"])
        else:
            base_folder = base_dir

            # Delete existing files
            if os.path.exists(base_folder + "/coverphoto"):
                shutil.rmtree(base_folder + "/coverphoto")
            if os.path.exists(base_folder + "/storyphotos"):
                shutil.rmtree(base_folder + "/storyphotos")
            if os.path.exists(base_folder + "/voicerecording"):
                shutil.rmtree(base_folder + "/voicerecording")

            # Create new files
            os.mkdir(base_folder + "/coverphoto")
            os.mkdir(base_folder + "/storyphotos")
            os.mkdir(base_folder + "/voicerecording")

            # Data extraction  
            coverPhoto = data.get("coverPhoto")
            voiceRecording = data.get("voiceRecording")
            storyPhotos = data.get("storyPhotos")
            storyPhotoTimes = data.get("storyPhotoTimes")
            transcriptOfKeywords = data.get("transcriptOfKeywords")
            transcriptOfKeywordTimes = data.get("transcriptOfKeywordTimes")

            # Define storage paths
            cover_photo_path = base_folder + "/coverphoto/coverphoto.png"
            compressed_voice_path = base_folder + "/voicerecording/compressed_voicerecording.mp3"
            decompressed_voice_path = base_folder + '/voicerecording/voicerecording.wav'
            story_photo_dir = base_folder + "/storyphotos/"

            # Download files
            async with aiohttp.ClientSession() as session:
                tasks = [
                    self.download_file(session, coverPhoto, cover_photo_path),
                    self.download_voice_recording(session, voiceRecording, compressed_voice_path),
                    *[self.download_file(session, storyPhoto, story_photo_dir + f"{i+1:02d}.png") for i, storyPhoto in enumerate(storyPhotos)]
                ]
                await asyncio.gather(*tasks)

            end_time = time.perf_counter()
            print(f"The code fetched in {end_time - start_time:0.4f} seconds")

            # Convert to tuples for json storage
            storyPhotoTimes = tuple([time for time in data["storyPhotoTimes"]])
            transcriptOfKeywords = tuple([keyword for keyword in data["transcriptOfKeywords"]])
            transcriptOfKeywordTimes = tuple(time for time in data["transcriptOfKeywordTimes"])

            # Store data in JSON
            story_json = {
                "storyPhotoTimes": json.dumps(storyPhotoTimes),
                "transcriptOfKeywords": json.dumps(transcriptOfKeywords),
                "transcriptOfKeywordTimes": json.dumps(transcriptOfKeywordTimes)
            }
            json_file_path = os.path.join(base_folder, "story_json.json")
            with open(json_file_path, "w") as outfile:
                json.dump(story_json, outfile)

            end_time = time.perf_counter()
            print(f"The code ran in {end_time - start_time:0.4f} seconds")

    async def download_file(self, session, url, path):
        async with session.get(url) as response:
            content = await response.read()
            with open(path, 'wb') as f:
                f.write(content)

    async def download_voice_recording(self, session, url, path):
        async with session.get(url) as response:
            with open(path, 'wb') as f:
                async for chunk in response.content.iter_chunked(1024):
                    if chunk:
                        f.write(chunk) 
    

if __name__ == "__main__":
    story_fetcher = StoryFetcher()
    rospy.spin()