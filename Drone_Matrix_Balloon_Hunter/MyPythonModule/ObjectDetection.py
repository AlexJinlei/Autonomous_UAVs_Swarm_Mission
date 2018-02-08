import os
import numpy as np
import threading
import cv2
import time
from collections import deque
from MyPythonModule import DroneControlFunction as dcf
import __builtin__ # __builtin.variables can cross multiple files (for imported functions).

########################################################################################

def find_shape_center_and_radius(binary_image):
    contours = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if contours: # contours is a list. The boolean value of an empty list is False.
    	max_contour = max(contours, key=cv2.contourArea) # key is a function applied to contours.
        ((center_x, center_y), radius) = cv2.minEnclosingCircle(max_contour)
        max_contour_center = (int(round(center_x)),int(round(center_y)))
        radius = int(round(radius))
        return (max_contour_center, radius, contours)
    else:
        return (None, None, contours)

########################################################################################

def calculateDisparity(framePair):

    min_disp = 0
    num_disp = 3
    block_size = 10
    isSGBM = 0
    
    gray_l = cv2.cvtColor(framePair[0], cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(framePair[1], cv2.COLOR_BGR2GRAY)
    
    '''
    imgTwin = np.hstack((frame_l, frame_r))
    imgTwin = cv2.resize(imgTwin,(800,300))
    cv2.imshow('frame_l  frame_r',imgTwin)
    '''

    if(isSGBM):
        stereo = cv2.StereoSGBM_create(minDisparity=min_disp, numDisparities=16*num_disp,\
        blockSize=2*block_size+1)  
    else:
        stereo = cv2.StereoBM_create(numDisparities=16*num_disp,blockSize=2*block_size+1)

    # Compute disparity.    
    disparity = stereo.compute(gray_l, gray_r)

    if 1: # post process for plot.
        # Convert to plot value.
        disparity = disparity/16 #Scale to (0~255) int16
        #print(disparity.min(), disparity.max())                
        #disparity = disparity.astype(np.float16)/disparity.max().astype(np.float16)*255.0
        disparity = disparity.astype(np.float16)/55.0*255.0
        #print(disparity.min(), disparity.max())                
        disparity = disparity.astype(np.uint8)
        #print(disparity.min(), disparity.max())
        #disparity = cv2.medianBlur(disparity, 5)

    return disparity

########################################################################################

def find_shape_center(binary_image):
    M = cv2.moments(binary_image)
    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
    return center

########################################################################################

def find_max_contour(binary_image):
    # cv2.findContours will change original data, so we pass a copy.
    contours = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] # cv2.CHAIN_APPROX_NONE
    if contours: # contours is a list. The boolean value of an empty list is False.
    	max_contour = max(contours, key=cv2.contourArea) # key is a function applied to contours.
        return max_contour
    else:
        return None

########################################################################################

def min_enclosing_circle(contour):
    try:
        ((center_x, center_y), radius) = cv2.minEnclosingCircle(contour)
        return (int(round(center_x)), int(round(center_y))), int(round(radius))
    except Exception:
        return ((None, None), None)
        
########################################################################################

def least_square_circle_fit(contour):
    # Frome contour get x and y coordinates.
    x = contour[:,:,0]
    y = contour[:,:,1]
    # coordinates of the barycenter
    x_m = np.mean(x)
    y_m = np.mean(y)
    # calculation of the reduced coordinates
    u = x - x_m
    v = y - y_m
    # linear system defining the center in reduced coordinates (uc, vc):
    #    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
    #    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
    Suv  = np.sum(u*v)
    Suu  = np.sum(u**2)
    Svv  = np.sum(v**2)
    Suuv = np.sum(u**2 * v)
    Suvv = np.sum(u * v**2)
    Suuu = np.sum(u**3)
    Svvv = np.sum(v**3)
    # Solving the linear system
    A = np.array([ [ Suu, Suv ], [Suv, Svv]])
    B = np.array([ Suuu + Suvv, Svvv + Suuv ])/2.0
    try:
        uc, vc = np.linalg.solve(A, B)
    except Exception:
        return ((None, None), None)
    xc = x_m + uc
    yc = y_m + vc
    # Calculation of all distances from the center (xc, yc)
    Ri      = np.sqrt((x-xc)**2 + (y-yc)**2)
    R       = np.mean(Ri)
    #residu  = np.sum((Ri-R)**2)
    #residu2 = np.sum((Ri**2-R**2)**2)
    return (int(xc), int(yc)), int(R)

########################################################################################

# Get the region of specified color in a given image.
# Return a mask in which the desired color region is True and the other region is False.
# With blurring, it takes 0.00187092208862s for a 360X240 frame.
def extract_colored_shape(image, color_range_lower, color_range_upper, colorspace='HSV', floodFill=False):
    # color_range_lower, color_range_upper are tuples, cv2.inRange only takes numpy array with dtype='uint8' as parameters. 
    # So we need to convert tuples to numpy array.
    color_range_lower = np.array(color_range_lower, dtype='uint8')
    color_range_upper = np.array(color_range_upper, dtype='uint8')
    if (colorspace == 'HSV'):
        # Blur the input image.
        #image = cv2.medianBlur(image,5) # Take lots of time, about 0.0014245s for 360X240 frame.
        # Convert color space from BGR to HSV()
        #t1 = time.time()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #t2 = time.time()
        #print('convert time = ',(t2-t1)*1000.0)
        if (color_range_lower[0] > color_range_upper[0]): # Hue range crosses 0 degree.
            #t1 = time.time()
            # Split to two ranges.
            color_range1_lower = np.copy(color_range_lower)
            color_range1_upper = np.copy(color_range_upper)
            color_range1_upper[0] = 180
            color_range2_lower = np.copy(color_range_lower)
            color_range2_upper = np.copy(color_range_upper)
            color_range2_lower[0] = 0
            #t2 = time.time()
            #print('split time = ',(t2-t1)*1000.0)
            mask1 = cv2.inRange(image, color_range1_lower, color_range1_upper)
            mask2 = cv2.inRange(image, color_range2_lower, color_range2_upper)
            mask = mask1 | mask2
            #t2 = time.time()
            #print('find range time: ', (t2-t1)*1000.0)
        else:
            # Find all color pixels in given color range.
            mask = cv2.inRange(image, color_range_lower, color_range_upper)
    elif (colorspace == 'BGR'):
        mask = cv2.inRange(image, color_range_lower, color_range_upper)
    else:
        print('Please specify "BGR" or "HSV" colorspace.')
        return None
    
    if floodFill:
        # Make a copy for mask.
        mask_floodFilled = mask.copy()
        # Make a flood fill mask for mask_floodFilled. The size needs to be 2 pixels larger than the image.
        height_mask_floodFilled, width_mask_floodFilled = mask_floodFilled.shape
        fillMask_for_mask_floodFilled = np.zeros((height_mask_floodFilled+2, width_mask_floodFilled+2), np.uint8)
        # Flood fill mask_floodFilled with white color.
        cv2.floodFill(mask_floodFilled, fillMask_for_mask_floodFilled, (0,0), 255); 
        # Combine the mask and inversed mask_floodFilled to get the foreground.
        foreground = mask | cv2.bitwise_not(mask_floodFilled)
        # erode the foreground to remove isolated islands.
        foreground = cv2.erode(foreground, None, iterations=1)
        return foreground
    else:
        mask = cv2.erode(mask, None, iterations=1)
        return mask
    
########################################################################################

# Calculate Distance frome disparity value.
# focal lenth and distance are in meters. disparity is in pixels. 
def disparity2distance(disparity, **kwargs):
	if 'baseLine' not in kwargs:
		print('Please specify baseLine!')
		return None
	if 'focalLength' not in kwargs:
		print('Please specify focalLengh!')
		return None
	if disparity != 0:
		return 1.0 * kwargs['baseLine'] * kwargs['focalLength'] / disparity
	else:
		return np.inf

######################################################################################## s
'''
class ControlFlag_bool:
    def __init__(self, lock, bool_value=False):
        ControlFlag_bool.value = bool_value
        self.lock = lock
    def set_true(self):
        with self.lock:
            ControlFlag_bool.value = True
    def set_false(self):
        with self.lock:
            ControlFlag_bool.value = False
'''

# A subclass within Thread_detect_balloon class.
class ControlFlag_bool:
    def __init__(self, bool_value=False):
        self.value = bool_value
        self._lock = threading.Lock()
    def set_true(self):
        with self._lock:
            self.value = True
    def set_false(self):
        with self._lock:
            self.value = False

########################################################################################

class Thread_detect_balloon(threading.Thread):
    def __init__(self, cameraL, cameraR, color_range_pair):
        threading.Thread.__init__(self)
        self._cameraL = cameraL
        self._cameraR = cameraR
        self._color_range_pair = color_range_pair
        self._isRunning_Thread_detect_balloon = ControlFlag_bool(bool_value=False)
        self._isStartedVideoRecording = None
        self._result_queue = deque(maxlen=1)
        self._isPlotOriginalFrame = None
        self._isPlotForeground    = None
        self._isPlotDetected      = None
        self._isPlotEncloseCircle = None
        self._isPlotFittedCircle  = None
        self._displayOrSave       = None
    
    '''
    # A subclass within Thread_detect_balloon class.
    class _ControlFlag_bool:
        def __init__(self, bool_value=False):
            self.value = bool_value
            self._lock = threading.Lock()
        def set_true(self):
            with self._lock:
                self.value = True
        def set_false(self):
            with self._lock:
                self.value = False
    '''
    
    def get_result(self): # Get an element in deque without deleting it. 
        while True:
            try:
                return self._result_queue[-1] # Get the right most result.
            except Exception:
                #print('Exception happened, wait 1 milisecond.')
                cv2.waitKey(1)
    
    def locate_balloon(self):
        result = self.get_result()
        centerL_x_fit = result[2][0][0]
        centerL_y_fit = result[2][0][1]
        radiusL_fit   = result[2][0][2]
        centerR_x_fit = result[2][1][0]
        centerR_y_fit = result[2][1][1]
        radiusR_fit   = result[2][1][2]
        centerL_x_enclose = result[3][0][0]
        centerL_y_enclose = result[3][0][1]
        radiusL_enclose   = result[3][0][2]
        centerR_x_enclose = result[3][1][0]
        centerR_y_enclose = result[3][1][1]
        radiusR_enclose   = result[3][1][2]
        if (radiusL_fit is not None)&(radiusR_fit is not None)&(radiusL_enclose is not None)&(radiusR_enclose is not None):
            if (abs(centerL_y_fit - centerR_y_fit) < abs(centerL_y_enclose - centerR_y_enclose)):
                return result[2]
            else:
                return result[3]
        else:
            return ((None, None, None),(None, None, None))
    
    def locate_balloon_fit(self):
        result = self.get_result()
        centerL_x_fit = result[2][0][0]
        centerL_y_fit = result[2][0][1]
        radiusL_fit   = result[2][0][2]
        centerR_x_fit = result[2][1][0]
        centerR_y_fit = result[2][1][1]
        radiusR_fit   = result[2][1][2]
        if (radiusL_fit is not None)&(radiusR_fit is not None):
            return result[2]
        else:
            return ((None, None, None),(None, None, None))
    
    def locate_balloon_enc(self):
        result = self.get_result()
        centerL_x_enclose = result[3][0][0]
        centerL_y_enclose = result[3][0][1]
        radiusL_enclose   = result[3][0][2]
        centerR_x_enclose = result[3][1][0]
        centerR_y_enclose = result[3][1][1]
        radiusR_enclose   = result[3][1][2]
        if (radiusL_enclose is not None)&(radiusR_enclose is not None):
            return result[3]
        else:
            return ((None, None, None),(None, None, None))
    
    def start_video_recording(self, isPlotOriginalFrame=True,
                              isPlotForeground=True,
                              isPlotDetected=True,
                              isPlotEncloseCircle=True,
                              isPlotFittedCircle=True,
                              displayOrSave='save',
                              framePath='./saved_frames/'):
        # Instantialize Thread_record_video class.
        self._video_recording = self.Thread_record_video(self._result_queue,
                                isPlotOriginalFrame=isPlotOriginalFrame,
                                isPlotForeground=isPlotForeground,
                                isPlotDetected=isPlotDetected,
                                isPlotEncloseCircle=isPlotEncloseCircle,
                                isPlotFittedCircle=isPlotFittedCircle,
                                displayOrSave=displayOrSave,
                                framePath=framePath)
        # Start video_recording instance.
        self._video_recording.start()
        self._isStartedVideoRecording = True
        print('Video recording is started!')
        
    def stop_video_recording(self):
        if self._isStartedVideoRecording:
            self._video_recording.stop()
            self._isStartedVideoRecording = False
            print('Video recording is stopped!')
        else:
            print('Do not need to stop video recording, it is not started!')
    
    def start(self):
        self._isRunning_Thread_detect_balloon.set_true()
        threading.Thread.start(self)
        print('{} - Thread_detect_balloon is started!\n'.format(time.ctime()))
        
    def stop(self):
        self._isRunning_Thread_detect_balloon.set_false()
        print('{} - Thread_detect_balloon is stopped!\n'.format(time.ctime()))
    
    def run(self): # Capture frames and save to deque.
        color_range_lower = self._color_range_pair[0]
        color_range_upper = self._color_range_pair[1]
        while self._isRunning_Thread_detect_balloon.value:
            #print('{} - detect_balloon is running.'.format(time.ctime()))
            # Grab.
            self._cameraL.grab()
            self._cameraR.grab()
            # Retrieve.
            raw_frameL = self._cameraL.retrieve()
            raw_frameR = self._cameraR.retrieve()
            # Decode.
            #frameL = cameraL.decode_MJPG(raw_frameL, color=1) # Decode raw jpeg image.
            frameL = self._cameraL.decode_MJPG_downsample(raw_frameL, (480, 270), color=1)
            frameR = self._cameraR.decode_MJPG_downsample(raw_frameR, (480, 270), color=1) # Decode raw jpeg image.
            # If both frames are captured and decoded successfully.
            if (frameL is not None)&(frameR is not None):
                # Extract the shape with desired color, get a binary mask. Do not need to check if None.
                foregroundL = extract_colored_shape(frameL, color_range_lower, color_range_upper, colorspace='HSV')
                foregroundR = extract_colored_shape(frameR, color_range_lower, color_range_upper, colorspace='HSV')
                # Find max contour of the foreground.
                max_contourL = find_max_contour(foregroundL)
                max_contourR = find_max_contour(foregroundR)
                if (max_contourL is not None)&(max_contourR is not None):
                    # Find min enclosing circle.
                    (centerL_x_enclose, centerL_y_enclose), radiusL_enclose = min_enclosing_circle(max_contourL)
                    (centerR_x_enclose, centerR_y_enclose), radiusR_enclose = min_enclosing_circle(max_contourR)
                    # Put result into tuple.            
                    enclose_circle = ((centerL_x_enclose, centerL_y_enclose, radiusL_enclose), (centerR_x_enclose, centerR_y_enclose, radiusR_enclose))
                    # Find least square fitted circle.
                    (centerL_x_fit, centerL_y_fit), radiusL_fit = least_square_circle_fit(max_contourL)
                    (centerR_x_fit, centerR_y_fit), radiusR_fit = least_square_circle_fit(max_contourR)
                    # Put result into tuple.            
                    fit_circle = ((centerL_x_fit, centerL_y_fit, radiusL_fit), (centerR_x_fit, centerR_y_fit, radiusR_fit))
                    # Put results into tuple.
                    result = ((frameL, frameR), (foregroundL, foregroundR), enclose_circle, fit_circle)
                else:
                    enclose_circle = ((None, None, None), (None, None, None))		
                    fit_circle = ((None, None, None), (None, None, None))
                    result = ((frameL, frameR), (foregroundL, foregroundR), enclose_circle, fit_circle)
            else:
                result = ((None, None), (None, None), ((None, None, None), (None, None, None)), ((None, None, None), (None, None, None)))
            # Put result into deque.
            self._result_queue.append(result)
    
    # A subclass within Thread_detect_balloon class.
    class Thread_record_video(threading.Thread):
        def __init__(self, frameQueue,
                           isPlotOriginalFrame = True,
                           isPlotForeground    = True,
                           isPlotDetected      = True,
                           isPlotEncloseCircle = True,
                           isPlotFittedCircle  = True,
                           displayOrSave       = 'save',
                           framePath = './saved_frames/'):
            # Save parameter into class variables, so other functions can use it.
            self._frameQueue          = frameQueue
            self._isPlotOriginalFrame = isPlotOriginalFrame
            self._isPlotForeground    = isPlotForeground
            self._isPlotDetected      = isPlotDetected
            self._isPlotEncloseCircle = isPlotEncloseCircle
            self._isPlotFittedCircle  = isPlotFittedCircle
            self._displayOrSave       = displayOrSave
            self._framePath           = framePath
            self._isRunningThread_record_video = ControlFlag_bool(bool_value=False)
            # Use the original __init__ function from threading.Thread class.
            threading.Thread.__init__(self)
        # Get result from self._frameQueue
        def _get_result(self): # Get an element in deque without deleting it. 
            while True:
                try:
                    return self._frameQueue[-1] # Get the right most result.
                except Exception:
                    #print('Exception happened, wait 1 milisecond.')
                    cv2.waitKey(1)
        # Start Thread_record_video
        def start(self):
            self._isRunningThread_record_video.set_true()
            threading.Thread.start(self)
            print('{} - Thread_record_video is started!\n'.format(time.ctime()))
        # Stop Thread_record_video
        def stop(self):
            self._isRunningThread_record_video.set_false()
            print('{} - Thread_record_video is stopped!\n'.format(time.ctime()))
        # Main function in Thread_record_video class.
        def run(self):
            # Create folder to save frames.
            if (self._displayOrSave == 'save')|(self._displayOrSave == 'both'):
                if not os.path.exists(self._framePath):
                    os.makedirs(self._framePath)
            frame_counts = 1
            t1 = time.time()
            while (self._isRunningThread_record_video.value): # If .value is true.
                # Get result from deque.
                result = self._get_result()
                # Get time stamp.
                current_time_in_seconds = time.time()
                ms_str = str(int(current_time_in_seconds*1000%1000))
                while (len(ms_str) < 3):
                    ms_str = ms_str + '0'
                time_stamp_save = '{}.{}'.format(time.strftime('%Y%m%d_%H-%M-%S'), ms_str)
                time_stamp_display = '{}.{}'.format(time.strftime('%Y-%m-%d %H:%M:%S'), ms_str)
                #print('Time : {}'.format(time_stamp_display))
                frameL = result[0][0] # Left original frame.
                frameR = result[0][1] # Right original frame.
                foregroundL = result[1][0] # Left extract foregroundL.
                foregroundR = result[1][1] # Right extract foregroundR.
                print('{} - fit (x, y, r): {}'.format(time_stamp_display, result[2]))
                print('{} - enc (x, y, r): {}'.format(time_stamp_display, result[3]))
                if self._isPlotOriginalFrame & (frameL is not None) & (frameR is not None):
                    frameL = result[0][0]
                    frameR = result[0][1]
                    frame_pair = np.hstack((frameL, frameR))
                    cv2.putText(frame_pair,time_stamp_display,(18,28), 2, 1,(0,0,0),2)
                    cv2.putText(frame_pair,time_stamp_display,(18,28), 2, 1,(255,255,255),1)
                    if self._displayOrSave == 'display':
                        cv2.imshow('original', frame_pair)
                    elif self._displayOrSave == 'save': # Save frame.
                        cv2.imwrite(self._framePath + '/' + time_stamp_save+'_original.jpg', frame_pair)
                    elif self._displayOrSave == 'both': # Display and save frame.
                        cv2.imshow('original', frame_pair)
                        cv2.imwrite(self._framePath + '/' + time_stamp_save+'_original.jpg', frame_pair)                
                if self._isPlotForeground & (foregroundL is not None) & (foregroundR is not None):
                    foreground_pair = np.hstack((foregroundL, foregroundR))
                    cv2.putText(foreground_pair,time_stamp_display,(18,28), 2, 1,(0,0,0),2)
                    cv2.putText(foreground_pair,time_stamp_display,(18,28), 2, 1,(255,255,255),1)
                    if self._displayOrSave == 'display':
                        cv2.imshow('foreground', foreground_pair)
                    elif self._displayOrSave == 'save': # Save frame.
                        cv2.imwrite(self._framePath + time_stamp_save+'_foreground.jpg', foreground_pair)
                    elif self._displayOrSave == 'both': # Display and save frame.
                        cv2.imshow('foreground', foreground_pair)
                        cv2.imwrite(self._framePath + time_stamp_save+'_foreground.jpg', foreground_pair)
                if self._isPlotDetected & (frameL is not None) & (frameR is not None):
                    output_l = frameL#.copy()
                    output_r = frameR#.copy()
                    if self._isPlotEncloseCircle:
                        centerL_x_enclose = result[3][0][0]
                        centerL_y_enclose = result[3][0][1]
                        radiusL_enclose   = result[3][0][2]
                        centerR_x_enclose = result[3][1][0]
                        centerR_y_enclose = result[3][1][1]
                        radiusR_enclose   = result[3][1][2]
                        # Plot left frame.
                        if all(x is not None for x in [centerL_x_enclose, centerL_y_enclose, radiusL_enclose]):
                            centerL_x_enclose = int(round(centerL_x_enclose))
                            centerL_y_enclose = int(round(centerL_y_enclose))
                            radiusL_enclose   = int(round(radiusL_enclose))
                            cv2.circle(output_l, (centerL_x_enclose, centerL_y_enclose), radiusL_enclose, (255, 255, 255), 2)
                            cv2.rectangle(output_l, (centerL_x_enclose - 2, centerL_y_enclose - 2), (centerL_x_enclose + 2, centerL_y_enclose + 2), (255, 255, 255), -1)
                        if all(x is not None for x in [centerR_x_enclose, centerR_y_enclose, radiusR_enclose]):
                            centerR_x_enclose = int(round(centerR_x_enclose))
                            centerR_y_enclose = int(round(centerR_y_enclose))
                            radiusR_enclose   = int(round(radiusR_enclose))
                            cv2.circle(output_r, (centerR_x_enclose, centerR_y_enclose), radiusR_enclose, (255, 255, 255), 2)
                            cv2.rectangle(output_r, (centerR_x_enclose - 2, centerR_y_enclose - 2), (centerR_x_enclose + 2, centerR_y_enclose + 2), (255, 255, 255), -1)
                    if self._isPlotFittedCircle:
                        centerL_x_fit = result[2][0][0]
                        centerL_y_fit = result[2][0][1]
                        radiusL_fit   = result[2][0][2]
                        centerR_x_fit = result[2][1][0]
                        centerR_y_fit = result[2][1][1]
                        radiusR_fit   = result[2][1][2]
                        # Plot left frame.
                        if all(x is not None for x in [centerL_x_fit, centerL_y_fit, radiusL_fit]):
                            centerL_x_fit = int(round(centerL_x_fit))
                            centerL_y_fit = int(round(centerL_y_fit))
                            radiusL_fit   = int(round(radiusL_fit))
                            cv2.circle(output_l, (centerL_x_fit, centerL_y_fit), radiusL_fit, (0, 255, 0), 2) # circle.
                            cv2.rectangle(output_l, (centerL_x_fit - 2, centerL_y_fit - 2), (centerL_x_fit + 2, centerL_y_fit + 2), (0, 255, 0), -1) # center.
                        # Plot right frame.
                        if all(x is not None for x in [centerR_x_fit, centerR_y_fit, radiusR_fit]):
                            centerR_x_fit = int(round(centerR_x_fit))
                            centerR_y_fit = int(round(centerR_y_fit))
                            radiusR_fit   = int(round(radiusR_fit))
                            cv2.circle(output_r, (centerR_x_fit, centerR_y_fit), radiusR_fit, (0, 255, 0), 2)
                            cv2.rectangle(output_r, (centerR_x_fit - 2, centerR_y_fit - 2), (centerR_x_fit + 2, centerR_y_fit + 2), (0, 255, 0), -1)
                    detected_frame_pair = np.hstack((output_l, output_r))
                    cv2.putText(detected_frame_pair,time_stamp_display,(18,28), 2, 1,(0,0,0),2)
                    cv2.putText(detected_frame_pair,time_stamp_display,(18,28), 2, 1,(255,255,255),1)
                    if self._displayOrSave == 'display':
                        cv2.imshow('detected', detected_frame_pair)
                    elif self._displayOrSave == 'save': # Save frame.
                        cv2.imwrite(self._framePath + time_stamp_save+'_detected.jpg', detected_frame_pair)
                    elif self._displayOrSave == 'both': # Display and save frame.    
                        cv2.imshow('detected', detected_frame_pair)
                        cv2.imwrite(self._framePath + time_stamp_save+'_detected.jpg', detected_frame_pair)
                frame_counts += 1
                cv2.waitKey(1) # This serves all cv2.imshow(). cv2.imshow() only work with cv2.waitKey().
            # When while loop is finished. Calculate frames per second.
            t2 = time.time()
            print('FPS = {}\n'.format(frame_counts*1.0/(t2 - t1)))
            # Close all windows.
            cv2.destroyAllWindows()

########################################################################################

class Balloon_destroyer:

    def __init__(self, vehicle, cameraL, cameraR, color_range_pair, time_out, max_altitude, max_radius, framePath):
        self._time_out = time_out
        self._vehicle = vehicle
        self._home_lat = None
        self._home_lon = None
        self._home_alt = None
        self._start_time = None
        self._isFoundBalloon = False
        self._isMissionCompleted = False
        self._max_altitude = max_altitude
        self._max_radius = max_radius
        self._framePath = framePath
        self._L_x = None
        self._L_y = None
        self._L_r = None
        self._R_x = None
        self._R_y = None
        self._R_r = None
        # Have to instantialize Thread_detect_balloon first.
        self._detect_balloon = Thread_detect_balloon(cameraL, cameraR, color_range_pair)
        print('{} - Class Balloon_destroyer is instantialized.\n'.format(time.ctime()))
    
    def get_sonar_distance(self):
        return self._vehicle.rangefinder.distance
        
    def get_vision_distance(self, x_diff):
        b = 4.4843131825975853e-06
        k = 1.6119424896362898e-05
        visionDistance = disparity2distance(x_diff, baseLine=0.14, focalLength=4.35e-3)
        corrected = (visionDistance - b) * 1.0 / k # Liner correction.
        return corrected
    
    def get_distance_to_home(self):
        # Get current GPS coordinate.
        current_lat = self._vehicle.location.global_relative_frame.lat
        current_lon = self._vehicle.location.global_relative_frame.lon
        #current_alt = self._vehicle.location.global_relative_frame.alt
        
        # Calculate distance between two gps coordinates.
        return dcf.distance_between_two_gps_coord((current_lat,current_lon), (self._home_lat,self._home_lon))
    
    def approach_balloon_sonar(self):
        print('{} - Executing approach_balloon()...\n'.format(time.ctime()))
        rtv = True
        while ((self.get_sonar_distance() > 2) & (self.get_distance_to_home() < self._max_radius) & rtv):
            # Every time approaching balloon, adjust the balloon position in view field.
            rtv = self.make_balloon_in_view_center()
            # Vx: forward from the current vehicle position
            # Vy: to the right
            # Vz: down (forward, right and down are "positive" values).
            dcf.send_body_frame_velocity_once(self._vehicle, 0.5, 0, 0) # Fly forward at 0.5 m/s
            print('{} - Sonar distance = {}.\n'.format(time.ctime(), self.get_sonar_distance()))
            time.sleep(1) # Sleep 1 second.
        # Stop moving when distance less than 2 meter.
        dcf.air_break(self._vehicle)
        print('{} - Function approach_balloon() is finished.\n'.format(time.ctime()))
        return rtv
        
    def approach_balloon_old(self): # By vision.
        print('{} - Executing approach_balloon()...\n'.format(time.ctime()))
        rtv = True
        counter = 10
        while True:
            # Get detecting result.
            ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon_enc()
            if (self._L_r is not None)&(self._R_r is not None):
                r_mean = (self._L_r + self._R_r) // 2
                if ((r_mean < 45) & (self.get_distance_to_home() < self._max_radius) & rtv):
                    # Every time approaching balloon, adjust the balloon position in view field.
                    rtv = self.make_balloon_in_view_center() # rtv is False when balloon is not detected or lost.
                    # Vx: forward from the current vehicle position
                    # Vy: to the right
                    # Vz: down (forward, right and down are "positive" values).
                    dcf.send_body_frame_velocity_once(self._vehicle, 0.5, 0, 0) # Fly forward at 0.5 m/s
                    print('{} - Sonar distance = {}.\n'.format(time.ctime(), self.get_sonar_distance()))
                    time.sleep(0.5) # Sleep 0.5 second.
                else:
                    # Stop moving.
                    dcf.air_break(self._vehicle)
                    print('{} - Function approach_balloon() is finished.\n'.format(time.ctime()))
                    return rtv
            else:
                if (counter > 0):
                    time.sleep(0.1)
                    counter -= 1
                else:
                    # Stop moving.
                    dcf.air_break(self._vehicle)
                    print('{} - Function approach_balloon() is finished. Balloon lost.\n'.format(time.ctime()))
                    return False

    def approach_balloon(self): # By vision.
        print('{} - Executing approach_balloon()...\n'.format(time.ctime()))
        counter = 10
        while True:
            # Check distance to home.
            if (self.get_distance_to_home() > self._max_radius):
                # Stop moving.
                dcf.air_break(self._vehicle)
                print('{} - Function approach_balloon() is stopped. Drone exceeds distance limitation.\n'.format(time.ctime()))
                return False
            # Get detecting result.
            ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon_enc()
            if (self._L_r is not None)&(self._R_r is not None): # Balloon is detected.
                if ((self._L_r + self._R_r)//2 < 45): # If the radius of the balloon is less than 45 pixels.
                    # Every time approaching balloon, adjust the balloon position in view field.
                    if (self.make_balloon_in_view_center()): # Return value is False if self.make_balloon_in_view_center() fails.
                        # Send velocity command to drone.
                        # Vx: forward from the current vehicle position
                        # Vy: to the right
                        # Vz: down (forward, right and down are "positive" values).
                        dcf.send_body_frame_velocity_once(self._vehicle, 0.5, 0, 0) # Fly forward at 0.5 m/s
                        print('{} - Getting close to balloon...'.format(time.ctime()))
                        print('{} - Sonar distance = {}.\n'.format(time.ctime(), self.get_sonar_distance()))
                        time.sleep(0.5) # Sleep 0.5 second.
                    else: # Balloon is lost while executing self.make_balloon_in_view_center().
                        # Stop moving.
                        dcf.air_break(self._vehicle)
                        print('{} - Function approach_balloon() is stopped. Balloon is lost while making it in view field center.')
                        return False
                else: # Drone is getting close enough to balloon.
                    # Stop moving.
                    dcf.air_break(self._vehicle)
                    print('{} - Function approach_balloon() is finished. Drone is close to the balloon.'.format(time.ctime()))
                    print('{} - Sonar distance = {}.\n'.format(time.ctime(), self.get_sonar_distance()))
                    return True
            else: # Balloon is not detected. Try to detect it again.
                if (counter > 0): # If not exceed the maximum trial times.
                    time.sleep(0.1)
                    counter -= 1
                else: # Exceeds the maximum trial times.
                    # Stop moving.
                    dcf.air_break(self._vehicle)
                    print('{} - Function approach_balloon() is finished. Balloon is lost when drone is approaching to it.\n'.format(time.ctime()))
                    return False
    
    def make_balloon_in_view_center_x(self):
        # In 70 pixel box, 205<x<275, 100<y<170.
        # In 30 pixel box, 225<x<255, 120<y<150.
        print('{} - Executing make_balloon_in_view_center_x()...\n'.format(time.ctime()))
        # Get latest balloon location.
        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
        # If balloon is detected.
        if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
            # If balloon is already in 70 pixel box, do nothing.
            if(((self._L_x + self._R_x)//2 > 205) and ((self._L_x + self._R_x)//2<275)):
                dcf.air_break(self._vehicle)
                print('{} - Do not need adjust x! Balloon is already in the 70 pixel center box.\n'.format(time.ctime()))
                return True
            else: # If balloon is outside the 70 pixel center box.
                # Adjust vehicle to make balloon appear in 30 pixel center box.
                for temp in xrange(150): # 150*0.1s = 15s at most.
                    # Check if time out.
                    if ((time.time()-self._start_time) >= self._time_out):
                        dcf.air_break(self._vehicle)
                        print('{} - Time out! Function make_balloon_in_view_center_x() is stopped.\n'.format(time.ctime()))
                        return False
                    # Get latest balloon location.
                    ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                    if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
                        # Adjust yaw.
                        if ((self._L_x + self._R_x)//2<225): # In left view field, drone turn left.
                            # Rotate drone 1 degree to the left.
                            print('{} - Rotate vehicle 1 degree to the left.\n'.format(time.ctime()))
                            dcf.send_body_frame_yaw_once(self._vehicle, 1, -1) # turn_direction -1 : counter clockwise
                            time.sleep(0.1) # Upon testing, the turning speed is 30 degree/second.
                        elif ((self._L_x + self._R_x)//2>255):  # In right view field, drone turn right.
                            # Rotate drone 1 degree to the right.
                            print('{} - Rotate vehicle 1 degree to the right.\n'.format(time.ctime()))
                            dcf.send_body_frame_yaw_once(self._vehicle, 1, 1) # turn_direction 1 : clockwise
                            time.sleep(0.1) # Upon testing, the turning speed is 30 degree/second.
                        else: # In center
                            dcf.air_break(self._vehicle)
                            print('{} - Done! Function make_balloon_in_view_center_x() is finished.\n'.format(time.ctime()))
                            return True
                    else: # Balloon lost.
                        dcf.air_break(self._vehicle)
                        self._isFoundBalloon = False # Balloon lost.
                        print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
        else: # Balloon is not detected.
            dcf.air_break(self._vehicle)
            self._isFoundBalloon = False # Balloon lost.
            print('{} - Balloon lost! Function make_balloon_in_view_center_x() is stopped.\n'.format(time.ctime()))
            return False
    
    def make_balloon_in_view_center_x_finer(self):
        # In 70 pixel box, 205<x<275, 100<y<170.
        # In 50 pixel box, 215<x<265, 110<y<160.
        # In 30 pixel box, 225<x<255, 120<y<150.
        print('{} - Executing make_balloon_in_view_center_x()...\n'.format(time.ctime()))
        # Get latest balloon location.
        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
        # If balloon is detected.
        if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
            # If balloon is already in 50 pixel box, do nothing.
            if(((self._L_x + self._R_x)//2 > 215) and ((self._L_x + self._R_x)//2<265)):
                dcf.air_break(self._vehicle)
                print('{} - Do not need adjust x! Balloon is already in the 70 pixel center box.\n'.format(time.ctime()))
                return True
            else: # If balloon is outside the 50 pixel center box.
                # Adjust vehicle to make balloon appear in 30 pixel center box.
                for temp in xrange(150): # 150*0.1s = 15s at most.
                    # Check if time out.
                    if ((time.time()-self._start_time) >= self._time_out):
                        dcf.air_break(self._vehicle)
                        print('{} - Time out! Function make_balloon_in_view_center_x() is stopped.\n'.format(time.ctime()))
                        return False
                    # Get latest balloon location.
                    ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                    if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
                        # Adjust yaw.
                        if ((self._L_x + self._R_x)//2<225): # In left view field, drone turn left.
                            # Rotate drone 1 degree to the left.
                            print('{} - Rotate vehicle 1 degree to the left.\n'.format(time.ctime()))
                            dcf.send_body_frame_yaw_once(self._vehicle, 1, -1) # turn_direction -1 : counter clockwise
                            time.sleep(0.1) # Upon testing, the turning speed is 30 degree/second.
                        elif ((self._L_x + self._R_x)//2>255):  # In right view field, drone turn right.
                            # Rotate drone 1 degree to the right.
                            print('{} - Rotate vehicle 1 degree to the right.\n'.format(time.ctime()))
                            dcf.send_body_frame_yaw_once(self._vehicle, 1, 1) # turn_direction 1 : clockwise
                            time.sleep(0.1) # Upon testing, the turning speed is 30 degree/second.
                        else: # In center
                            dcf.air_break(self._vehicle)
                            print('{} - Done! Function make_balloon_in_view_center_x() is finished.\n'.format(time.ctime()))
                            return True
                    else: # Balloon lost.
                        dcf.air_break(self._vehicle)
                        self._isFoundBalloon = False # Balloon lost.
                        print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
        else: # Balloon is not detected.
            dcf.air_break(self._vehicle)
            self._isFoundBalloon = False # Balloon lost.
            print('{} - Balloon lost! Function make_balloon_in_view_center_x() is stopped.\n'.format(time.ctime()))
            return False
    
    def make_balloon_in_view_center_y(self):
        # In 70 pixel box, 205<x<275, 100<y<170.
        # In 30 pixel box, 225<x<255, 120<y<150.
        print('{} - Executing make_balloon_in_view_center_y()...\n'.format(time.ctime()))
        # Get latest balloon location.
        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
        # If balloon is detected.
        if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
            # If balloon is already in 70 pixel box, do nothing.
            if(((self._L_y + self._R_y)//2 > 100) and ((self._L_y + self._R_y)//2<170)):
                dcf.air_break(self._vehicle)
                print('{} - Do not need adjust y! Balloon is already in the 70 pixel center box.\n'.format(time.ctime()))
                return True
            else: # If balloon is outside the 70 pixel center box.
                # Adjust vehicle to make balloon appear in 30 pixel center box.
                for temp in xrange(150): # 150*0.1s = 15s at most.
                    # Check if time out.
                    if ((time.time()-self._start_time) >= self._time_out):
                        dcf.air_break(self._vehicle)
                        print('{} - Time out! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
                    # Get latest balloon location.
                    ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                    if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
                        # Adjust altitude.
                        y_mean = (self._L_y + self._R_y)//2
                        print('{} - y_mean = {}'.format(time.ctime(), y_mean))
                        if (y_mean<120): # Above center, drone moves up.
                            # Vx: forward from the current vehicle position
                            # Vy: to the right
                            # Vz: down (forward, right and down are "positive" values).
                            print('{} - y_mean<120, move vehicle up.\n'.format(time.ctime()))
                            dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, -0.5) # Fly up at 0.5 m/s
                            time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 0.1 s = 5cm
                        elif (y_mean>150): # Below center, drone moves down.
                            print('{} - y_mean>150, move vehicle down.\n'.format(time.ctime()))
                            dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, 0.5) # Fly down at 0.5 m/s
                            time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 0.1 s = 5cm
                        else: # In center
                            dcf.air_break(self._vehicle)
                            print('{} - Done! 120<y_mean<150. Function make_balloon_in_view_center_y() is finished.\n'.format(time.ctime()))
                            return True
                    else: # Balloon lost.
                        dcf.air_break(self._vehicle)
                        self._isFoundBalloon = False # Balloon lost.
                        print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
        else: # Balloon is not detected.
            dcf.air_break(self._vehicle)
            self._isFoundBalloon = False # Balloon lost.
            print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
            return False
    
    def make_balloon_in_view_center_y_finer(self):
        # In 70 pixel box, 205<x<275, 100<y<170.
        # In 50 pixel box, 215<x<265, 110<y<160.
        # In 30 pixel box, 225<x<255, 120<y<150.
        print('{} - Executing make_balloon_in_view_center_y()...\n'.format(time.ctime()))
        # Get latest balloon location.
        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
        # If balloon is detected.
        if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
            # If balloon is already in 50 pixel box, do nothing.
            if(((self._L_y + self._R_y)//2 > 110) and ((self._L_y + self._R_y)//2<160)):
                dcf.air_break(self._vehicle)
                print('{} - Do not need adjust y! Balloon is already in the 70 pixel center box.\n'.format(time.ctime()))
                return True
            else: # If balloon is outside the 70 pixel center box.
                # Adjust vehicle to make balloon appear in 30 pixel center box.
                for temp in xrange(150): # 150*0.1s = 15s at most.
                    # Check if time out.
                    if ((time.time()-self._start_time) >= self._time_out):
                        dcf.air_break(self._vehicle)
                        print('{} - Time out! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
                    # Get latest balloon location.
                    ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                    if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Detected.
                        # Adjust altitude.
                        y_mean = (self._L_y + self._R_y)//2
                        print('{} - y_mean = {}'.format(time.ctime(), y_mean))
                        if (y_mean<120): # Above center, drone moves up.
                            # Vx: forward from the current vehicle position
                            # Vy: to the right
                            # Vz: down (forward, right and down are "positive" values).
                            print('{} - y_mean<120, move vehicle up.\n'.format(time.ctime()))
                            dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, -0.5) # Fly up at 0.5 m/s
                            time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 0.1 s = 5cm
                        elif (y_mean>150): # Below center, drone moves down.
                            print('{} - y_mean>150, move vehicle down.\n'.format(time.ctime()))
                            dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, 0.5) # Fly down at 0.5 m/s
                            time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 0.1 s = 5cm
                        else: # In center
                            dcf.air_break(self._vehicle)
                            print('{} - Done! 120<y_mean<150. Function make_balloon_in_view_center_y() is finished.\n'.format(time.ctime()))
                            return True
                    else: # Balloon lost.
                        dcf.air_break(self._vehicle)
                        self._isFoundBalloon = False # Balloon lost.
                        print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
                        return False
        else: # Balloon is not detected.
            dcf.air_break(self._vehicle)
            self._isFoundBalloon = False # Balloon lost.
            print('{} - Balloon lost! Function make_balloon_in_view_center_y() is stopped.\n'.format(time.ctime()))
            return False
    
    def make_balloon_in_view_center(self):
        rtv_y = self.make_balloon_in_view_center_y() # Adjust altitude.
        rtv_x = self.make_balloon_in_view_center_x() # Adjust yaw.
        return (rtv_x and rtv_y)

    def make_balloon_in_view_center_finer(self):
        rtv_y = self.make_balloon_in_view_center_y_finer() # Adjust altitude.
        rtv_x = self.make_balloon_in_view_center_x_finer() # Adjust yaw.
        return (rtv_x and rtv_y)

    def rotate_to_search_balloon(self):
        # Remember current heading.
        initial_heading = self._vehicle.heading
        # Degree to turn.
        degree_to_turn = 360
        print('{} - Executing rotate_to_search_balloon()...\n'.format(time.ctime()))
        while True:
            # Check if time out.
            if ((time.time()-self._start_time) >= self._time_out):
                dcf.air_break(self._vehicle)
                print('{} - Time out! Function rotate_to_search_balloon() is stopped.\n'.format(time.ctime()))
                return False
            # Get detecting result.
            ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
            # If not detected, rotate drone.
            if not all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)): # Not detected.
                dcf.send_body_frame_yaw_once(self._vehicle, 1, 1) # turn_direction 1 : clockwise
                time.sleep(0.1) # Upon testing, the turning speed is 30 degree/second.
                degree_to_turn -= 1
                if (degree_to_turn <= 0):
                    # Restore initial heading.
                    print('{} - Restore initial heading.\n'.format(time.ctime()))
                    dcf.set_yaw(self._vehicle, initial_heading, 0)
                    dcf.air_break(self._vehicle)
                    print('{} - 360 degree rotation is done! function rotate_to_search_balloon() is finished.\n'.format(time.ctime()))
                    return False
            else: # Detected.
                dcf.air_break(self._vehicle)
                # Change found state to True.
                self._isFoundBalloon = True
                print('{} - Balloon is found! function rotate_to_search_balloon() is finished.\n'.format(time.ctime()))
                return True
                
    def find_and_destroy_balloon(self): # This is the main function in this class.
        print('{} - Executing find_and_destroy_balloon()...\n'.format(time.ctime()))
        # Before taking off, get home gps coordiante.
        self._home_lat = self._vehicle.location.global_relative_frame.lat
        self._home_lon = self._vehicle.location.global_relative_frame.lon
        self._home_alt = self._vehicle.location.global_relative_frame.alt
        # Start continuously_detecting.
        self._detect_balloon.start() # Cameras start capturing frames and save results to deque.
        # Start video recording.
        print('{} - Start video recording...\n'.format(time.ctime()))
        self._detect_balloon.start_video_recording(isPlotOriginalFrame=False, displayOrSave='save', framePath=self._framePath)
        # Wait for 1 second to let camera warm up.
        time.sleep(1)
        # Counter
        count_move_down = 3
        # Get start time.
        self._start_time = time.time()
        # Drone take off. Looking for balloon while taking off.
        while True:
            
            # Check if time out.
            if ((time.time()-self._start_time) >= self._time_out):
                dcf.air_break(self._vehicle)
                print('{} - Time out! Function find_and_destroy_balloon() is stopped.\n'.format(time.ctime()))
                print('{} - Mission Failed! Balloon is not found!.\n'.format(time.ctime()))
                time.sleep(1)
                # Stop video recording.
                self._detect_balloon.stop_video_recording()
                # Stop continuously detection.
                self._detect_balloon.stop()
                # Return home.
                dcf.return_to_launch(self._vehicle)
                return False
            
            # If drone altitude is lower than max_altitude, move up.
            if (vehicle.location.global_relative_frame.alt < self._max_altitude):
                
                # Fly up.
                # Vx: forward from the current vehicle position
                # Vy: to the right
                # Vz: down (forward, right and down are "positive" values).
                for temp in xrange(4): # Fly up 20 cm.
                    dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, -0.5) # Fly up at 0.5 m/s. Negative z is up.
                    time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 4 * 0.1 s = 20 cm
                
                # Get detection result while flying up.
                ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)) and ((self._L_r + self._R_r)/2.0 >= 5): # If detected.
                    # Stop moving when balloon appears in the cameras' view field.
                    dcf.air_break(self._vehicle)
                    # Change found state to True.
                    self._isFoundBalloon = True
                    # Fly to balloon, stop when visual radius of balloon is big enough.
                    self.approach_balloon() # This function calls make_balloon_in_view_center() first.
                    rtv = self.make_balloon_in_view_center_finer() # At last step, use finer adjustment.
                    # Check if balloon is still in view field.
                    if rtv: # If balloon is still in view field after approaching.
                        # After approaching, update.
                        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                        # Check size to make sure it is really the balloon.
                        if (all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r))) and ((self._L_r + self._R_r)/2.0 >= 30):
                            # Hit balloon. Move forward 5 meters at 5m/s speed.
                            for i_temp in xrange(10): # Send command 10 times, total time is 1 second.
                                # Vx: forward from the current vehicle position
                                # Vy: to the right
                                # Vz: down (forward, right and down are "positive" values).
                                dcf.send_body_frame_velocity_once(self._vehicle, 5, 0, 0) # Fly forward at 5 m/s
                                time.sleep(0.1)
                            # After hitting, change state and air break.
                            self._isMissionCompleted = True
                            dcf.air_break(self._vehicle)
                            print('{} - Mission Completed! Balloon is destroyed!.\n'.format(time.ctime()))
                            # Stop video recording.
                            self._detect_balloon.stop_video_recording()
                            # Stop continuously detection.
                            self._detect_balloon.stop()
                            # Return home.
                            dcf.return_to_launch(self._vehicle)
                            return True
                        else:
                            print('{} - Mission Failed! The detected object is not balloon!.\n'.format(time.ctime()))
                            time.sleep(1)
                            # Stop video recording.
                            self._detect_balloon.stop_video_recording()
                            # Stop continuously detection.
                            self._detect_balloon.stop()
                            # Return home.
                            dcf.return_to_launch(self._vehicle)
                            return False
                    else: # Balloon is lost during approaching.
                        print('{} - Mission Failed! Balloon is lost during approaching!.\n'.format(time.ctime()))
                        time.sleep(1)
                        # Stop video recording.
                        self._detect_balloon.stop_video_recording()
                        # Stop continuously detection.
                        self._detect_balloon.stop()
                        # Return home.
                        dcf.return_to_launch(self._vehicle)
                        return False
            
            else: # If drone has reached the maxinum altitude but does not find balloon, rotate 360 deg to look for balloon.
                while (count_move_down > 0):
                    if (self.rotate_to_search_balloon()): # If balloon is found during rotating.
                        # Get detection result.
                        ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                        if all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r)) and ((self._L_r + self._R_r)/2.0 >= 5): # Detected or not.
                            # Stop moving when balloon appears in the cameras' view field.
                            dcf.air_break(self._vehicle)
                            # Change found state to True.
                            self._isFoundBalloon = True
                            # Fly to balloon, stop at 2 meters in front of it.
                            rtv = self.approach_balloon() # This function calls make_balloon_in_view_center() first.
                            # Check if balloon is still in view field.
                            if rtv: # If balloon is still in view field after approaching.
                                # After approaching, update.
                                ((self._L_x, self._L_y, self._L_r),(self._R_x, self._R_y, self._R_r)) = self._detect_balloon.locate_balloon()
                                # Check size to make sure it is really the balloon.
                                if (all(item is not None for item in (self._L_x, self._L_y, self._L_r, self._R_x, self._R_y, self._R_r))) and ((self._L_r + self._R_r)/2.0 >= 30):
                                    # Hit balloon. Move forward 5 meters at 5m/s speed.
                                    for i_temp in xrange(10): # Send command 10 times, total time is 1 second.
                                        # Vx: forward from the current vehicle position
                                        # Vy: to the right
                                        # Vz: down (forward, right and down are "positive" values).
                                        dcf.send_body_frame_velocity_once(self._vehicle, 5, 0, 0) # Fly forward at 5 m/s
                                        time.sleep(0.1)
                                    # After hitting, change state and air break.
                                    self._isMissionCompleted = True
                                    dcf.air_break(self._vehicle)
                                    print('{} - Mission Completed! Balloon is destroyed!.\n'.format(time.ctime()))
                                    # Stop video recording.
                                    self._detect_balloon.stop_video_recording()
                                    # Stop continuously detection.
                                    self._detect_balloon.stop()
                                    # Return home.
                                    dcf.return_to_launch(self._vehicle)
                                    return True
                                else:
                                    print('{} - Mission Failed! The detected object is not balloon!.\n'.format(time.ctime()))
                                    time.sleep(1)
                                    # Stop video recording.
                                    self._detect_balloon.stop_video_recording()
                                    # Stop continuously detection.
                                    self._detect_balloon.stop()
                                    # Return home.
                                    dcf.return_to_launch(self._vehicle)
                                    return False
                            else: # Balloon is lost during approaching.
                                print('{} - Mission Failed! Balloon is lost during approaching!.\n'.format(time.ctime()))
                                time.sleep(1)
                                # Stop video recording.
                                self._detect_balloon.stop_video_recording()
                                # Stop continuously detection.
                                self._detect_balloon.stop()
                                # Return home.
                                dcf.return_to_launch(self._vehicle)
                                return False
                    else: # If balloon is not found during rotating.
                        # Move down to search again in next iteration.
                        for temp in xrange(10): # Move vehicle down 0.5 meter to search again.
                            dcf.send_body_frame_velocity_once(self._vehicle, 0, 0, 0.5) # Fly down at 0.5 m/s
                            time.sleep(0.1) # Sleep 0.1 second. 0.5 m/s * 10 * 0.1 s = 50 cm
                        dcf.air_break(self._vehicle)
                        count_move_down -= 1
        
                # When while loop is finished (count_move_down = 0) but the function is still not returned.
                dcf.air_break(self._vehicle)
                print('{} - Mission Failed! Balloon is not found!.\n'.format(time.ctime()))
                time.sleep(1)
                # Stop video recording.
                self._detect_balloon.stop_video_recording()
                # Stop continuously detection.
                self._detect_balloon.stop()
                # Return home.
                dcf.return_to_launch(self._vehicle)
                return False
















