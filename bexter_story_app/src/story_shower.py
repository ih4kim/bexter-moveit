#! /usr/bin/env python

import rospy
import cv2
import os
from interfaces.srv import PathToStoryData, PathToStoryDataResponse

class StoryShower(object):
    def __init__(self):
        rospy.init_node("story_shower_node")
        # Create story shower service server
        self._service = rospy.Service('story_shower', PathToStoryData, self.display_image)
        self._img = None
        self._window_name = "Storybook"
        cv2.namedWindow(self._window_name,cv2.WINDOW_NORMAL)
        # Have it loop until kill
        while True:
            if self._img is None and cv2.getWindowProperty(self._window_name,cv2.WND_PROP_VISIBLE) > 0:
                cv2.destroyAllWindows
            else:
                cv2.imshow(self._window_name, self._img)
                cv2.waitKey(1)


    def display_image(self, req):
        image_path = req.path
        # Kill signal
        if image_path == "-1":
            self._img = None
            return PathToStoryDataResponse(True)

        # Check if file exists
        if not os.path.isfile(req.path):
            rospy.logerr("%s is not a valid file!", req.path)
            return PathToStoryDataResponse(False) 
        
        self._img = cv2.imread(req.path)
        return PathToStoryDataResponse(True)
    
if __name__ == "__main__":
    story_shower = StoryShower()
    rospy.spin()