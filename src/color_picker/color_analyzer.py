import cv2
import pixel_picker as pp
import os


if __name__=="__main__":
    impath = "/home/sanaria/vs_ws/src/ur_visual_servo/src/color_picker/usbcam1.png"
    img = cv2.imread(impath, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for i in range(10):
        pts = pp.PointPickerOne(img)
        print(hsv[pts[1],pts[0],:])
    