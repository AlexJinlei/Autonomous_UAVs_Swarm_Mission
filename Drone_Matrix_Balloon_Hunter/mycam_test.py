# v2.1

import os
import sys
import time
import numpy as np
import threading

from PIL import Image
import cv2

from fcntl import ioctl
from cStringIO import StringIO
import mmap
import ctypes

from MyPythonModule.v4l2_device import Camera
from MyPythonModule import ObjectDetection as od


#========================= main ===========================

'''
Pixel Format : MJPG
discrete 1920x1080 ( 30 fps) # Good. FPS=10.24. decode_time=58ms. Full hardward resolution. Largest view angle.
discrete 1280x1024 ( 30 fps) # Good. FPS=15.66. decode_time=40ms. View angel is slightly larger than 320X240
discrete 1280x720  ( 60 fps) # Good. FPS=23.23. decode_time=27ms. View angel is equal to 320X240
discrete 1024x768  ( 30 fps) # Dark. FPS=26.50. decode_time=22ms. View angel is slightly larger than 320X240
discrete  800x600  ( 60 fps) # Good. FPS=38.64. decode_time=17ms. Very Narrow.
discrete  640x480  (120 fps) # Dark. FPS=58.74. decode_time=11ms. View angel is equal to 320X240
discrete  320x240  (120 fps) # Good. FPS=99.14. decode_time= 3ms. 

Pixel Format : YUYV
discrete 1920x1080 (  6 fps) # FPS= 4.97. decode_time=12 ms.
discrete 1280x1024 (  6 fps) # FPS= 4.97. decode_time=8.0ms. Narrow.
discrete 1280x720  (  9 fps) # FPS= 9.21. decode_time=6.5ms.
discrete 1024x768  (  6 fps) # FPS= 4.96. decode_time=5.5ms. Dark. 
discrete  800x600  ( 20 fps) # FPS=19.90. decode_time=2.7ms. Narrow.
discrete  640x480  ( 30 fps) # FPS=29.83. decode_time=1.8ms. View angel is smaller than 320x240.
discrete  320x240  ( 30 fps) # FPS=29.82. decode_time=0.8ms. View angel is equal to 1920x1080, white ballence differ too much.
'''


# Open camera.
print('\n')
cameraL = Camera('/dev/video0')
cameraR = Camera('/dev/video1')
cameraL.open()
cameraR.open()


# Initialize video device.
resolution = (1920,1080)
#resolution = (1280,720)
#resolution = (640,480)
#resolution = (320,240)

pixelformat = 'MJPG'  # 'YUYV' or 'MJPG'.
cameraL.init_device(resolution=resolution, pixel_format=pixelformat, exposure='AUTO', white_balance='AUTO')
cameraR.init_device(resolution=resolution, pixel_format=pixelformat, exposure='AUTO', white_balance='AUTO')


print('\n')


# Initialize memory map.
cameraL.init_mmap()
cameraR.init_mmap()
# Start streaming.
cameraL.stream_on()
cameraR.stream_on()

frame_width, frame_height = cameraL.get_frame_width_height()


'''
colorspace = 'BGR'
color_range_lower = np.array([0, 0, 100], dtype = 'uint8')
color_range_upper = np.array([85, 85, 255], dtype = 'uint8')
'''

# In opencv, HSV values:
# Hue range        = [0,179]
# Saturation range = [0,255]
# Value range      = [0,255].

# Red color range.
red_color_range_lower = (170, 128, 55) # HSV color.
red_color_range_upper = (7, 255, 255) # HSV color.
red_color_range_pair = (red_color_range_lower, red_color_range_upper)

# Yellow color range.
yellow_color_range_lower = (23, 55, 55) # HSV color.
yellow_color_range_upper = (35, 255, 255) # HSV color.
yellow_color_range_pair = (yellow_color_range_lower, yellow_color_range_upper)


# Start detecting.
act_detecting = od.Thread_detect_balloon(cameraL, cameraR, red_color_range_pair)
act_detecting.start()


#max_counts = 100000
#counts = 1
#t1 = time.time()

time.sleep(1)

path = os.getcwd() + '/saved_frames_haha/'
print(path)

'''
try:
    act_detecting.start_capturing(folderPath=path, isPlotOriginalFrame=True, displayOrSave='both')
    time.sleep(10)
except KeyboardInterrupt:
    print('\n"Ctl-C" is pressed!')
    act_detecting.stop_capturing()
    pass    
'''

act_detecting.start_video_recording(framePath=path, isPlotOriginalFrame=False, displayOrSave='save')
for i in xrange(100):
    print(act_detecting.locate_balloon())
    time.sleep(0.1)

#t2 = time.time()
#print('FPS = {}'.format(counts*1.0/(t2 - t1)))


# Stop detecting thread.
act_detecting.stop_video_recording()
act_detecting.stop()

# Turn off stream.
cameraL.stream_off()
cameraR.stream_off()

# Turn off camera.
cameraL.close()
cameraR.close()

exit()


