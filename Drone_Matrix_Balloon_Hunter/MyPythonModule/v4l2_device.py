import cv2
import os
import sys
#import v4l2 # This import the installed module.
from MyPythonModule import v4l2 # This import the module from user folder. Python wrapper v4l2 API.
from fcntl import ioctl
import mmap
import ctypes
from cStringIO import StringIO # io is slow, use cStringIO instead.
from PIL import Image
import time
import numpy as np

########################################################################################

def print_dict(dict):
    for key in dict:
        print('{}: {}'.format(key,dict[key]))

# This is a wrapper of ioctl. Return 0 on success, -1 on fail.
def xioctl(fd, VIDIOC_control_word, structure):
    try:
        rtv = ioctl(fd, VIDIOC_control_word, structure)
        return rtv # May return non-negative value other than 0 on success.
    except Exception:
        return -1

########################################################################################

# This is a v4l2 Camera class.
class Camera():

    def __init__(self, device_name):
        self.name = device_name
        self.fd = None
        
        # Private variables.
        # specify the number of buffers.
        self._buffer_count = 4
        # dictionary to save mmap objects.
        self._mmap_dict = {}
        # Temperary buffer used in class.
        self._buffer = v4l2.v4l2_buffer()
        self._buffer.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        self._buffer.memory = v4l2.V4L2_MEMORY_MMAP
        self._buffer.index = 0
        self._buffer_offset = None
        self._buffer_size = None
        self._buffer_type = None
        self._ptr_buffer_type = None

    def open(self):
        if self.fd is None:
            self.fd = os.open(self.name, os.O_RDWR)
            print('Device "{}" is opened!'.format(self.name))
        else:
            print('Error: Can not open device "{}", because it is already opened!'.format(self.name))
    
    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
            print('Device "{}" is closed!'.format(self.name))
        else:
            print('Error: Can not close device "{}", because it is not opened!'.format(self.name))
    
    # Getting all supported video format for given device.
    def get_supported_format(self):
        fmtdesc = v4l2.v4l2_fmtdesc()
        fmtdesc.index = 0
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        format_dict = {}
        while(0 == xioctl(self.fd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)):
            format_dict[str(fmtdesc.index)] = {'pixelformat': v4l2.v4l2_fourcc2str(fmtdesc.pixelformat), 'description': fmtdesc.description}
            fmtdesc.index += 1
        return format_dict
    
    # Set video resolution and pixel format.
    def set_video_format(self, frame_width, frame_height, pixelformat_str):
        # The following pixel formats are supported by ELP-USBFHD01M-FV board camera.
        # ELP-USBFHD01M-FV only accept 'MJPG' or 'YUYV'
        # MJPG: V4L2_PIX_FMT_MJPEG
        # YUYV: V4L2_PIX_FMT_YUYV.
        pixelformat = v4l2.v4l2_fourcc(pixelformat_str[0], pixelformat_str[1], pixelformat_str[2], pixelformat_str[3])
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmt.fmt.pix.width = frame_width
        fmt.fmt.pix.height = frame_height
        fmt.fmt.pix.pixelformat = pixelformat
        fmt.fmt.pix.field = v4l2.V4L2_FIELD_NONE
        if(-1 == xioctl(self.fd, v4l2.VIDIOC_S_FMT, fmt)):
            print('Failed to set video format on device "{}".\n'.format(self.name))
            return -1
        else:
            print('Video format of device "{}" is set to:'.format(self.name))
            print('resolution  : {}x{}.'.format(fmt.fmt.pix.width, fmt.fmt.pix.height))
            print('pixelformat : {}\n'.format(v4l2.v4l2_fourcc2str(fmt.fmt.pix.pixelformat)))
            return 0
    
    # Get current video format.
    def get_video_format(self):
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_G_FMT, fmt)):
            print('Failed to get video format for device "{}"!\n'.format(self.name))
            return -1
        else:
            self.frame_width = fmt.fmt.pix.width
            self.frame_height = fmt.fmt.pix.height
            return {'resolution':{'width':fmt.fmt.pix.width, 'height':fmt.fmt.pix.height},\
                           'pixelformat':v4l2.v4l2_fourcc2str(fmt.fmt.pix.pixelformat),\
                           'field':v4l2.v4l2_field_dict[fmt.fmt.pix.field],\
                           'bytesperline':fmt.fmt.pix.bytesperline,\
                           'sizeimage':fmt.fmt.pix.sizeimage,\
                           'colorspace':v4l2.v4l2_colorspace_dict[fmt.fmt.pix.colorspace]}
    
    # Enumerate all control options supported by ELP-USBFHD01M-FV board camera.
    def enumerate_all_controls(self):
        print('Enumerating all controls for device "{}":'.format(self.name))
        queryctrl = v4l2.v4l2_queryctrl()
        querymenu = v4l2.v4l2_querymenu()
        queryctrl.id = v4l2.V4L2_CTRL_FLAG_NEXT_CTRL
        while (0 == xioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, queryctrl)):
            if (~(queryctrl.flags & v4l2.V4L2_CTRL_FLAG_DISABLED)):
                print('Control item: {}'.format(queryctrl.name))
                # If control ID has a type menu, print this menu.
                if (queryctrl.type == v4l2.V4L2_CTRL_TYPE_MENU):
                    # Clear querymenu variable.
                    querymenu.id = queryctrl.id
                    print('        menu:')
                    for querymenu.index in range(queryctrl.minimum, queryctrl.maximum+1):
                        if (0 == xioctl(self.fd, v4l2.VIDIOC_QUERYMENU, querymenu)):
                            print('              {}. {}'.format(querymenu.index, querymenu.name))
            # Get next control ID.
            queryctrl.id |= v4l2.V4L2_CTRL_FLAG_NEXT_CTRL
    
    def get_control_parameter(self, CID):   
        # Get info of given contro ID, and get current control setting.
        # CID means control ID.
        queryctrl = v4l2.v4l2_queryctrl()
        control = v4l2.v4l2_control()
        querymenu = v4l2.v4l2_querymenu()
        '''
        The following control_ID is supported by ELP-USBFHD01M-FV board camera.
        CID_list = [v4l2.V4L2_CID_BRIGHTNESS,
                    v4l2.V4L2_CID_CONTRAST,
                    v4l2.V4L2_CID_SATURATION,
                    v4l2.V4L2_CID_HUE,
                    v4l2.V4L2_CID_AUTO_WHITE_BALANCE,
                    v4l2.V4L2_CID_GAMMA,
                    v4l2.V4L2_CID_GAIN,
                    v4l2.V4L2_CID_POWER_LINE_FREQUENCY,
                    v4l2.V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                    v4l2.V4L2_CID_SHARPNESS,
                    v4l2.V4L2_CID_BACKLIGHT_COMPENSATION,
                    v4l2.V4L2_CID_EXPOSURE_AUTO,
                    v4l2.V4L2_CID_EXPOSURE_ABSOLUTE,
                    v4l2.V4L2_CID_EXPOSURE_AUTO_PRIORITY
        '''
        queryctrl.id = CID
        control.id = CID
        rtv_VIDIOC_QUERYCTRL = xioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, queryctrl)
        rtv_VIDIOC_G_CTRL = xioctl(self.fd, v4l2.VIDIOC_G_CTRL, control)
        if (-1 == rtv_VIDIOC_QUERYCTRL):
            print('Failed to query control parameter for device "{}".'.format(self.name))
            print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
            return -1
        elif (-1 == rtv_VIDIOC_G_CTRL):
            print('Failed to get control parameter for device "{}".'.format(self.name))
            print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
            return -1
        else:
            print('Get control parameter for device "{}" successfully.'.format(self.name))
            print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
            print('name          = {}'.format(queryctrl.name))
            print('minimum       = {}'.format(queryctrl.minimum))
            print('maximum       = {}'.format(queryctrl.maximum))
            print('step          = {}'.format(queryctrl.step))
            print('default_value = {}'.format(queryctrl.default_value))
            print('current_value = {}'.format(control.value))
            print('flags         = {}'.format(v4l2.v4l2_CTRL_FLAG_dict[queryctrl.flags]))
            print('type          = {}'.format(v4l2.v4l2_ctrl_type_dict[queryctrl.type]))
            # If CID is menu type, print this menu.
            if (queryctrl.type == v4l2.V4L2_CTRL_TYPE_MENU):
                # Clear querymenu variable.
                querymenu.id = queryctrl.id;
                print('menu: ')
                for querymenu.index in range(queryctrl.minimum, queryctrl.maximum+1):
                    if (0 == xioctl(self.fd, v4l2.VIDIOC_QUERYMENU, querymenu)):
                        print('     {}. {}'.format(querymenu.index, querymenu.name))
            return 0;
    
    def set_control_parameter(self, CID, control_value):   
        # Set control parameters.

        queryctrl = v4l2.v4l2_queryctrl()
        old_setting = v4l2.v4l2_control()
        new_setting = v4l2.v4l2_control()
        querymenu = v4l2.v4l2_querymenu()
        '''
        The following CID is supported by ELP-USBFHD01M-FV board camera.
        CID_list = [v4l2.V4L2_CID_BRIGHTNESS,
                    v4l2.V4L2_CID_CONTRAST,
                    v4l2.V4L2_CID_SATURATION,
                    v4l2.V4L2_CID_HUE,
                    v4l2.V4L2_CID_AUTO_WHITE_BALANCE,
                    v4l2.V4L2_CID_GAMMA,
                    v4l2.V4L2_CID_GAIN,
                    v4l2.V4L2_CID_POWER_LINE_FREQUENCY,
                    v4l2.V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                    v4l2.V4L2_CID_SHARPNESS,
                    v4l2.V4L2_CID_BACKLIGHT_COMPENSATION,
                    v4l2.V4L2_CID_EXPOSURE_AUTO,
                    v4l2.V4L2_CID_EXPOSURE_ABSOLUTE,
                    v4l2.V4L2_CID_EXPOSURE_AUTO_PRIORITY]
        '''
        queryctrl.id = CID
        old_setting.id = CID
        new_setting.id = CID
        new_setting.value = control_value
        
        # Query current setting.
        rtv_VIDIOC_QUERYCTRL = xioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, queryctrl)
        rtv_VIDIOC_G_CTRL = xioctl(self.fd, v4l2.VIDIOC_G_CTRL, old_setting)
        
        if (-1 == rtv_VIDIOC_QUERYCTRL):
            print('Before setting, failed to query control parameter for device "{}".'.format(self.name))
            print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
            #print('\n')
            return -1
        elif (-1 == rtv_VIDIOC_G_CTRL):
            print('Before setting, failed to get control parameter for device "{}"'.format(self.name))
            print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
            #print('\n')
            return -1
        else:
            # Set new_setting value.
            rtv_VIDIOC_S_CTRL = xioctl(self.fd, v4l2.VIDIOC_S_CTRL, new_setting)
            if (-1 == rtv_VIDIOC_S_CTRL):
                print('Failed to set control parameter for device "{}".'.format(self.name))
                print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
                print('\n')
                return -1
            else:
                print('Set control parameter for device "{}" successfully.'.format(self.name))
                # Query new setting.
                rtv_VIDIOC_QUERYCTRL = xioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, queryctrl)
                rtv_VIDIOC_G_CTRL = xioctl(self.fd, v4l2.VIDIOC_G_CTRL, new_setting)
                if (-1 == rtv_VIDIOC_QUERYCTRL):
                    print('After setting, failed to query control parameter for device "{}".'.format(self.name))
                    print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
                    #print('\n')
                    return -1
                elif (-1 == rtv_VIDIOC_G_CTRL):
                    print('After setting, failed to get control parameter for device "{}".'.format(self.name))
                    print('CID    = {}'.format(v4l2.v4l2_CID_dict[CID]))
                    #print('\n')
                    return -1
                else:
                    print('CID           = {}'.format(v4l2.v4l2_CID_dict[CID]))
                    print('name          = {}'.format(queryctrl.name))
                    print('minimum       = {}'.format(queryctrl.minimum))
                    print('maximum       = {}'.format(queryctrl.maximum))
                    print('step          = {}'.format(queryctrl.step))
                    print('default_value = {}'.format(queryctrl.default_value))
                    print('current_value = {} (old value = {})'.format(new_setting.value, old_setting.value))
                    print('flags         = {}'.format(v4l2.v4l2_CTRL_FLAG_dict[queryctrl.flags]))
                    print('type          = {}'.format(v4l2.v4l2_ctrl_type_dict[queryctrl.type]))
                    # If control ID has a type menu, print this menu.
                    if (queryctrl.type == v4l2.V4L2_CTRL_TYPE_MENU):
                        querymenu.id = queryctrl.id
                        print('menu: ')
                        for querymenu.index in range(queryctrl.minimum, queryctrl.maximum+1):
                            if (0 == xioctl(self.fd, v4l2.VIDIOC_QUERYMENU, querymenu)):
                                print('     {}. {}'.format(querymenu.index, querymenu.name))
                    #print('\n')
                    return 0;
    
    def print_all_control_parameter(self):
        print('Printing all control parameters...\n')
        CID_list = [v4l2.V4L2_CID_BRIGHTNESS,
                    v4l2.V4L2_CID_CONTRAST,
                    v4l2.V4L2_CID_SATURATION,
                    v4l2.V4L2_CID_HUE,
                    v4l2.V4L2_CID_AUTO_WHITE_BALANCE,
                    v4l2.V4L2_CID_GAMMA,
                    v4l2.V4L2_CID_GAIN,
                    v4l2.V4L2_CID_POWER_LINE_FREQUENCY,
                    v4l2.V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                    v4l2.V4L2_CID_SHARPNESS,
                    v4l2.V4L2_CID_BACKLIGHT_COMPENSATION,
                    v4l2.V4L2_CID_EXPOSURE_AUTO,
                    v4l2.V4L2_CID_EXPOSURE_ABSOLUTE,
                    v4l2.V4L2_CID_EXPOSURE_AUTO_PRIORITY]
        for CID in CID_list:
            self.get_control_parameter(CID)
            print('\n')

    def get_supported_frame_size_and_fps(self):
        print('Supported frame size and fps by device "{}":'.format(self.name))
        fmtdesc = v4l2.v4l2_fmtdesc()     # Must fill .index and .type before use.
        frmsize = v4l2.v4l2_frmsizeenum() # Must fill .index and .pixel_format before use.
        frmival = v4l2.v4l2_frmivalenum() # Must fill .index, .pixel_format, .width, and .height before use.
        fmtdesc.index = 0
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        while (0 == xioctl(self.fd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)):            
            print('Pixel Format : {}'.format(v4l2.v4l2_fourcc2str(fmtdesc.pixelformat)))            
            frmsize.index = 0;
            frmsize.pixel_format = fmtdesc.pixelformat;
            while ((0 == xioctl(self.fd, v4l2.VIDIOC_ENUM_FRAMESIZES, frmsize))):
                if (frmsize.type == v4l2.V4L2_FRMSIZE_TYPE_DISCRETE):
                    # Fill frmival to check fps.
                    frmival.index = 0
                    frmival.pixel_format = fmtdesc.pixelformat
                    frmival.width = frmsize.discrete.width
                    frmival.height = frmsize.discrete.height
                    while (0 == xioctl(self.fd, v4l2.VIDIOC_ENUM_FRAMEINTERVALS, frmival)):
                        #print('denominator = {}'.format(frmival.discrete.denominator))
                        #print('numerator   = {}'.format(frmival.discrete.numerator))
                        fps = frmival.discrete.denominator/frmival.discrete.numerator
                        print ('    discrete {: >4}x{: <4} ({: >3} fps)'.format(frmsize.discrete.width, frmsize.discrete.height, fps))
                        frmival.index += 1 # Get next fps.
                elif(frmsize.type == v4l2.V4L2_FRMSIZE_TYPE_STEPWISE):
                    print('    stepwise {}x{}'.format(frmsize.stepwise.min_width, frmsize.stepwise.min_height))
                    print('    min_width   = {}'.format(frmsize.stepwise.min_width))
                    print('    max_width   = {}'.format(frmsize.stepwise.max_width))
                    print('    step_width  = {}'.format(frmsize.stepwise.step_width))
                    print('    min_height  = {}'.format(frmsize.stepwise.min_height))
                    print('    max_height  = {}'.format(frmsize.stepwise.max_height))
                    print('    step_height = {}'.format(frmsize.stepwise.step_height))
                frmsize.index += 1 # Get next frame size.
            fmtdesc.index += 1 # Get next video format.
        return 0;

    def get_frame_rate(self):
        parm = v4l2.v4l2_streamparm()
        parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        rtv = xioctl(self.fd, v4l2.VIDIOC_G_PARM, parm)
        if (-1 == rtv):
            print('Failed to get stream parameters for device "{}".'.format(self.name))
            return -1
        if (parm.parm.capture.capability != v4l2.V4L2_CAP_TIMEPERFRAME):
            print('The frame skipping/repeating controlled by the timeperframe field is not supported for device "{}".'.format(self.name))
            return -1
        return (parm.parm.capture.timeperframe.denominator)/(parm.parm.capture.timeperframe.numerator)
        
        
    def get_frame_width_height(self):
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_G_FMT, fmt)):
            print('Failed to get video format for device "{}"!\n'.format(self.name))
            return (-1, -1)
        return (fmt.fmt.pix.width, fmt.fmt.pix.height)

        
    def get_device_capability(self):
        # Query camera capability.
        capability = v4l2.v4l2_capability()
        rtv = xioctl(self.fd, v4l2.VIDIOC_QUERYCAP, capability)
        if (-1 == rtv):
            print('Failed to get device capability for device "{}".'.format(self.name))
            return -1
        # Get capability variables.
        bus_info = capability.bus_info
        capabilities = capability.capabilities
        card = capability.card
        driver = capability.driver
        reserved = capability.reserved
        version = capability.version
        print('bus_info     : {}'.format(bus_info))
        # capabilities: A 32-bit longer integer withholding your device's capabilities (one bit per capability).
        # You can use a bitwise & to check for a particular one
        print('capabilities : ({})'.format(hex(capabilities)))
        for key in v4l2.v4l2_capabilities_dict:
            if (capabilities & key):
                print('               {}'.format(v4l2.v4l2_capabilities_dict[key]))
        print('card         : {}'.format(card))
        print('driver       : {}'.format(driver))
        print('version      : {}'.format(version))
        return 0
            
    
    def init_device(self, resolution=(480,270), pixel_format='MJPG', exposure='AUTO', white_balance='AUTO'):
        time.sleep(1) # Wait for device warm up.
        # Power line frequency. # 2 = 60 Hz (USA frequency), 0 = disable, 1 = 50 Hz.
        counter = 1
        while ((-1 == self.set_control_parameter(v4l2.V4L2_CID_POWER_LINE_FREQUENCY, 2)) and counter<=3):
            print('Setting power line frequency {} times.'.format(counter))
            counter += 1
            time.sleep(1)
        # Set camera resolution and pixel format.
        counter = 1
        while ((-1 == self.set_video_format(resolution[0], resolution[1], pixel_format)) and counter<=3):
            print('Setting resolution {} times.'.format(counter))
            counter += 1
            time.sleep(1)
        # Set exposure.
        if (exposure=='AUTO'):
            self.set_control_parameter(v4l2.V4L2_CID_EXPOSURE_AUTO, 3) # 1 = Manual Mode, 3 = Aperture Priority Mode
            self.set_control_parameter(v4l2.V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0) # Do not vary frame rate according to exposure.
        elif (isinstance(exposure, (int, long))):
            if ((exposure<1)or(exposure>5000)):
                print('Value Error: exposure range is from 1 to 5000.')
                return -1
            else:
                self.set_control_parameter(v4l2.V4L2_CID_EXPOSURE_AUTO, 1) # 1 = Manual Mode, 3 = Aperture Priority Mode
                self.set_control_parameter(v4l2.V4L2_CID_EXPOSURE_ABSOLUTE, exposure) # Default 177.
        else:
            print('Value Error: please specify "AUTO" or exposure value (int).')
            return -1
        # Set white balance.
        if (white_balance=='AUTO'):
            self.set_control_parameter(v4l2.V4L2_CID_AUTO_WHITE_BALANCE, 1) # 1 = auto, 0 = manual.
        elif (isinstance(white_balance, (int, long))): # white balance temperature range = (2800 ~ 6500), 4600 by default.
            if ((white_balance<2800)or(white_balance>6500)):
                print('Range Error: white balance temperature range is from 2800 to 6500.')
                return -1
            else:
                self.set_control_parameter(v4l2.V4L2_CID_AUTO_WHITE_BALANCE, 0) # 1 = auto, 0 = manual.
                self.set_control_parameter(v4l2.V4L2_CID_WHITE_BALANCE_TEMPERATURE, white_balance)
        else:
            print('Value Error: please specify "AUTO" or white balance temperature value (int).')
            return -1
        return 0
            
    
    def init_mmap(self):
        # Inform the device about your future buffers.
        bufrequest = v4l2.v4l2_requestbuffers()
        bufrequest.count = self._buffer_count
        bufrequest.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        bufrequest.memory = v4l2.V4L2_MEMORY_MMAP
        
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_REQBUFS, bufrequest)):
            print('init_mmap() failed. Can not request buffers.\n')
            return -1
            
        for i in xrange(self._buffer_count):
            self._buffer.index = i
            if (-1 == xioctl(self.fd, v4l2.VIDIOC_QUERYBUF, self._buffer)): # self._buffer.index will automatically increased by 1.
                print('init_mmap() failed. Can not query buffers.\n')
                return -1
            
            self._buffer_offset = self._buffer.m.offset
            self._buffer_size = self._buffer.length
            #print('self._buffer_offset = {}'.format(self._buffer_offset))
            #print('self._buffer_size = {}\n'.format(self._buffer_size))
            
            # After calling the xioctl above, the structure's length and m.offset fields are ready.
            # We can therefore map our memory.
            buffer_mmap = mmap.mmap(self.fd,
                         self._buffer.length,
                         mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE,
                         offset=self._buffer.m.offset)
            # Add buffer_mmap to buffer_mmap_list.
            self._mmap_dict[i] = buffer_mmap


    def stream_on(self):
        for i in xrange(self._buffer_count):
            self._buffer.index = i
            if (-1 == xioctl(self.fd, v4l2.VIDIOC_QBUF, self._buffer)): # Queue buffer won't increase buffer index.
                print('stream_on() failed. Can not queue buffers.\n')
                return -1
        self._buffer_type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        self._ptr_buffer_type = ctypes.cast(self._buffer_type, ctypes.c_void_p)
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_STREAMON, self._ptr_buffer_type)):
            print('stream_on() failed. Can not turn stream on.\n')
            return -1
        self.retrieve() # Release the first frame grabbed in this function.
        print('Stream is turned on on device "{}".'.format(self.name))
        return 0

    def stream_off(self):
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_STREAMOFF, self._ptr_buffer_type)):
            print('stream_off fialed. Either not turn stream on or failed turn off.')
            return -1
        print('Stream is turned off on device "{}".'.format(self.name))
        return 0

    def retrieve(self):
        # In VIDIOC_DQBUF, buffer.index does not accept user specify value.
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_DQBUF, self._buffer)): # Dequeue buffer will increase buffer.index by 1.
            print('retrieve() failed. Can not dequeue buffers.\n')
            return None
        try:
            raw_img = np.asarray(bytearray(self._mmap_dict[self._buffer.index].read(self._buffer_size)), dtype=np.uint8)
            self._mmap_dict[self._buffer.index].seek(0) # Rollback buffer pointer.
            return raw_img
        except Exception:
            return None
        
    def decode_MJPG(self, raw_img, color=True):
        # cv2.IMREAD_COLOR
        # cv2.IMREAD_GRAYSCALE 
        # cv2.IMREAD_UNCHANGED
        try:
            if color:
                return cv2.imdecode(raw_img, cv2.IMREAD_COLOR)
            else:
                return cv2.imdecode(raw_img, cv2.IMREAD_GRAYSCALE)
        except Exception:
            return None
        
    def decode_MJPG_downsample(self, raw_img, (width, height), color=True):
        try:
            # Need import Image from PIL.
            PIL_img = Image.open(StringIO(raw_img)) # 8.04901123047 ms to load 1920x1080.
            #PIL_img = Image.open(io.BytesIO(raw_frameL)) # 11.7862224579 ms to load 1920x1080.
            if color:
                PIL_img.draft('P', (width, height)).convert('RGB')
                return cv2.cvtColor(np.array(PIL_img), cv2.COLOR_RGB2BGR)
            else:
                PIL_img.draft('L', (width, height))
                return np.array(PIL_img)
        except Exception:
            return None
            
    def decode_YUYV(self, raw_img, (width, height)):
        try:
            return cv2.cvtColor(np.reshape(raw_img, (height, width, 2)), cv2.COLOR_YUV2BGR_YUY2)
        except Exception:
            return None

    def grab(self):
        if (-1 == xioctl(self.fd, v4l2.VIDIOC_QBUF, self._buffer)):
            print('grab() failed. Can not queue buffers.\n')
            return -1



#========================= main ===========================

'''
The following resolution is supported by ELP-USBFHD01M-FV board camera.

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

