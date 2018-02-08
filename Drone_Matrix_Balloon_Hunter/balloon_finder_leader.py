# This is the main function for leader drone.
# Version 2.0

import time
import numpy as np
import Queue
import threading
from datetime import datetime
import netifaces as ni
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import os, sys
from MyPythonModule.v4l2_device import Camera
from MyPythonModule import ObjectDetection as od
from MyPythonModule import DroneControlFunction as dcf
import __builtin__ # __builtin.variables can cross multiple files (for imported functions).


# Get local host IP.
local_host = ni.ifaddresses('wlan0')[2][0]['addr']
print('{} - local_host = {}.'.format(time.ctime(), local_host))
host_specifier = local_host[-1]
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Set log.
flight_log_bufsize = 1 # 0 means unbuffered, 1 means line buffered.
flight_log_filename = 'FlightLog_iris' + host_specifier + '_' + '{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '.txt'
flight_log_path = '/home/iris' + host_specifier + '/Log/'
if not os.path.exists(flight_log_path):
    os.makedirs(flight_log_path)
flight_log_path_filename = flight_log_path + flight_log_filename
flight_log = open(flight_log_path_filename, 'w', flight_log_bufsize)
sys.stdout = flight_log

# Specify whether a leader or a follower.
is_leader = True
if is_leader:
    print('{} - This is a leader drone.'.format(time.ctime()))
else:
    print('{} - This is a follower drone.'.format(time.ctime()))

# Reserved port (constants, do not change).
# The port number should be exactly the same as that in follower drone.
__builtin__.port_gps = 60001
__builtin__.port_status = 60002
__builtin__.port_immediate_command = 60003
__builtin__.port_heading = 60004

# IP list (constants, do not change):
iris1_host = '192.168.2.101'
iris2_host = '192.168.2.102'
iris3_host = '192.168.2.103'
iris4_host = '192.168.2.104'

# Assign name to each follower host.
follower1 = iris2_host
follower2 = iris3_host
follower3 = iris4_host
follower_host_tuple = (follower1, follower2, follower3)

########################################## Initialize Leader Drone ##########################################

# Connect to the Vehicle
print('{} - Connecting to vehicle...'.format(time.ctime()))
vehicle_temp = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)
while not 'vehicle_temp' in locals():
    print('{} - Waiting for vehicle connection...'.format(time.ctime()))
    time.sleep(1)
__builtin__.vehicle = vehicle_temp
print('{} - Vehicle is connected!'.format(time.ctime()))
# Enable safety switch(take effect after reboot pixhawk).
__builtin__.vehicle.parameters['BRD_SAFETYENABLE'] = 1 # Enable
#vehicle.parameters['BRD_SAFETYENABLE'] = 0 # Disable

# Start leader server services.
dcf.start_SERVER_service(__builtin__.vehicle, local_host)

# Start leader connection checker. Drone will return home once lost connection.
router_host = '192.168.2.1'
threading.Thread(target=dcf.CHECK_network_connection,args=(__builtin__.vehicle, router_host,),kwargs={'wait_time':10}).start()

# Get GPS coordinate of leader's launch location.
leader_gps_home = __builtin__.vehicle.location.global_relative_frame
leader_lat_home = leader_gps_home.lat
leader_lon_home = leader_gps_home.lon
leader_alt_home = leader_gps_home.alt
print('{} - Home GPS coordinate :'.format(time.ctime()))
print('     leader_lat_home = {}'.format(leader_lat_home))
print('     leader_lon_home = {}'.format(leader_lon_home))
print('     leader_alt_home = {} (relative)'.format(leader_alt_home))

########################################## Define Colors and Command parameters ##########################################

# Red color range.
red_color_range_lower = (165, 90,  80) # HSV color.
red_color_range_upper = (178, 255, 200) # HSV color.
red_color_range_pair = (red_color_range_lower, red_color_range_upper)

# Yellow color range.
yellow_color_range_lower = ( 21, 50, 114) # HSV color.
yellow_color_range_upper = ( 35, 200, 255) # HSV color.
yellow_color_range_pair = (yellow_color_range_lower, yellow_color_range_upper)

# orange color range.
orange_color_range_lower = (177, 100,  110) # HSV color.
orange_color_range_upper = (12,180, 255) # HSV color.
orange_color_range_pair = (orange_color_range_lower, orange_color_range_upper)

# Pink color range.
pink_color_range_lower = (145, 10,  140) # HSV color.
pink_color_range_upper = (177, 170, 255) # HSV color.
pink_color_range_pair = (pink_color_range_lower, pink_color_range_upper)

# Set time and space restriction.
time_out = 300 # In seconds.
max_altitude = 4 # In meters.
max_radius = 10 # In meters.

########################################## Initialize Leader Balloon Finder ##########################################

# Open camera.
print('\n')
cameraL = Camera('/dev/video0')
cameraR = Camera('/dev/video1')
cameraL.open()
cameraR.open()
# Initialize video device.
resolution = (1920,1080)
pixelformat = 'MJPG'  # 'YUYV' or 'MJPG'.
exposure = 3
cameraL.init_device(resolution=resolution, pixel_format=pixelformat, exposure=exposure, white_balance='AUTO')
cameraR.init_device(resolution=resolution, pixel_format=pixelformat, exposure=exposure, white_balance='AUTO')
print('\n')
# Initialize memory map.
cameraL.init_mmap()
cameraR.init_mmap()
# Start streaming.
cameraL.stream_on()
cameraR.stream_on()

# Arm leader drone without RC.
dcf.arm_no_RC(__builtin__.vehicle)

########################################## Coordinate Follower Drones ##########################################

# Wait untill all followers are ready(armed).
dcf.wait_for_follower_ready(follower_host_tuple) # This is a blocking call.

# Take off.
threading.Thread(target=dcf.takeoff, args=(__builtin__.vehicle, 3,)).start() # Wait time is 3 seconds.
# Followers takeoff and hover. Send takeoff command to all followers. Immediate command must be in string type.
# Send command to all followers.
for follower in follower_host_tuple:
    print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower))
    dcf.CLIENT_send_immediate_command(follower, 'dcf.takeoff(__builtin__.vehicle,3)')

########################################## Start Balloon_destroyer ##########################################

# For leader drone. iris1.
leader_path_to_save_frames = '/home/iris1/saved_frames/{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '/'
# Instantiate Balloon_destroyer class.
balloon_terminator = od.Balloon_destroyer(__builtin__.vehicle, cameraL, cameraR, red_color_range_pair, time_out, max_altitude, max_radius, leader_path_to_save_frames)
# Start find_and_destroy_balloon()
threading.Thread(target=balloon_terminator.find_and_destroy_balloon).start()

# For follower drones.
command_str_factory = 'balloon_terminator=od.Balloon_destroyer(__builtin__.vehicle,cameraL,cameraR,{},{},{},{},"{}")'

# iris1 is leader, follower 1,2,3 are iris 2,3,4.
follower1_path_to_save_frames = '/home/iris2/saved_frames/{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '/'
follower2_path_to_save_frames = '/home/iris3/saved_frames/{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '/'
follower3_path_to_save_frames = '/home/iris4/saved_frames/{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '/'

command_str_to_follower1 = command_str_factory.format(yellow_color_range_pair, time_out, max_altitude, max_radius, follower1_path_to_save_frames)
command_str_to_follower2 = command_str_factory.format(orange_color_range_pair, time_out, max_altitude, max_radius, follower2_path_to_save_frames)
command_str_to_follower3 = command_str_factory.format(pink_color_range_pair, time_out, max_altitude, max_radius, follower3_path_to_save_frames)

command_tuple = (command_str_to_follower1, command_str_to_follower2, command_str_to_follower3)
# Send command to each follower drone. follower_host_tuple = (follower1,follower2,follower3)
# Create instance.
for i_temp in xrange(len(follower_host_tuple)):
    print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower_host_tuple[i_temp]))
    dcf.CLIENT_send_immediate_command(follower_host_tuple[i_temp], command_tuple[i_temp])
# Start instance.
for i_temp in xrange(len(follower_host_tuple)):
    print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower_host_tuple[i_temp]))
    dcf.CLIENT_send_immediate_command(follower_host_tuple[i_temp], 'balloon_terminator.find_and_destroy_balloon()')

# When mission completed, balloon_terminator instance will command each drone to go home.







