import cv2

img = "/home/billyk/Documents/get_story_robot/storyphotos/01.png"
img = cv2.imread(img)
print("Test")
if img is None:
    print("TEST")

while True:
    cv2.imshow("test", img)
    cv2.waitKey(1)