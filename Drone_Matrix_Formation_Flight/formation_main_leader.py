# This is the main function for leader drone.
# Version 2.1

import time
from datetime import datetime
import netifaces as ni
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import os, sys
sys.path.append(os.getcwd())
from formation_function import *
import __builtin__

# Get local host IP.
local_host = ni.ifaddresses('wlan0')[2][0]['addr']
host_specifier = local_host[-1]

# Set log.
flight_log_bufsize = 1 # 0 means unbuffered, 1 means line buffered.
flight_log_filename = 'FlightLog_iris' + host_specifier + '_' + '{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '.txt'
flight_log_path = '/home/iris' + host_specifier + '/Log/'
flight_log_path_filename = flight_log_path + flight_log_filename
flight_log = open(flight_log_path_filename, 'w', flight_log_bufsize)
sys.stdout = flight_log

# Specify whether a leader or a follower.
is_leader = True
if is_leader:
    print('{} - This is a leader drone.'.format(time.ctime()))
    leader_host = local_host
else:
    print('{} - This is a follower drone.'.format(time.ctime()))

print('{} - local_host = {}.'.format(time.ctime(), local_host))
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Get local host IP.
local_host = ni.ifaddresses('wlan0')[2][0]['addr']
print('{} - local_host = {}.'.format(time.ctime(), local_host))
host_specifier = local_host[-1]
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Reserved port.
# The port number should be exactly the same as that in follower drone.
__builtin__.port_gps = 60001
__builtin__.port_status = 60002
__builtin__.port_immediate_command = 60003
__builtin__.port_heading = 60004

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

# Start server services.
start_SERVER_service(is_leader, local_host)

# Start connection checker. Drone will return home once lost connection.
router_host = '192.168.2.1'
threading.Thread(target=CHECK_network_connection,args=(router_host,),kwargs={'wait_time':10}).start()

# Arm drone without RC.
arm_no_RC()

# IP list:
iris1_host = '192.168.2.101'
iris2_host = '192.168.2.102'
iris3_host = '192.168.2.103'
iris4_host = '192.168.2.104'

follower1 = iris2_host
follower2 = iris3_host
follower3 = iris4_host
follower_host_tuple = (follower1, follower2, follower3,)

# Wait untill all followers are ready(armed).
wait_for_follower_ready(follower_host_tuple) # This is a blocking call.

# Get GPS coordinate of leader's launch location.
leader_gps_home = __builtin__.vehicle.location.global_relative_frame
leader_lat_home = leader_gps_home.lat
leader_lon_home = leader_gps_home.lon
leader_alt_home = leader_gps_home.alt
print('{} - Home GPS coordinate :'.format(time.ctime()))
print('     leader_lat_home = {}'.format(leader_lat_home))
print('     leader_lon_home = {}'.format(leader_lon_home))
print('     leader_alt_home = {} (relative)'.format(leader_alt_home))

# DOUBLE CHECK the following 4 parameters before each flight mission.
leader_hover_height = 20 # In meter.
leader_fly_distance = 20 # In meters.
leader_aim_heading_direction = __builtin__.vehicle.heading #(use current) # In degree, 0~360. 90=East

# Fixed parameters.
# fly_follow() parameters for follower1.
follower1_followee = '\''+leader_host+'\'' # The string must contain ''.
follower1_frame_to_followee = '\''+'body'+'\'' # 'body' or 'local'.
# fly_follow() parameters for follower2.
follower2_followee = follower1_followee
follower2_frame_to_followee = follower1_frame_to_followee
# fly_follow() parameters for follower3.
follower3_followee = follower1_followee
follower3_frame_to_followee = follower1_frame_to_followee



# ===================== Formation 1 (squre) =====================
# When taking off, drones are already in this formation.
# Follower 1.
follower1_hover_height = 20 # In meter.
follower1_distance_to_followee = 10 # In meter.
follower1_azimuth_to_followee = 270 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 2.
follower2_hover_height = 20 # In meter.
follower2_distance_to_followee = 14.4 # In meter.
follower2_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 3.
follower3_hover_height = 20 # In meter.
follower3_distance_to_followee = 10 # In meter.
follower3_azimuth_to_followee = 180 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.

# When all members are ready.
# Leader takeoff and hover (in square shape).
threading.Thread(target=takeoff_and_hover, args=(leader_hover_height,)).start()
# Send takeoff command to all followers.
# Immediate command must be in string type.
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower1))
CLIENT_send_immediate_command(follower1, 'takeoff_and_hover({})'.format(follower1_hover_height))
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower2))
CLIENT_send_immediate_command(follower2, 'takeoff_and_hover({})'.format(follower2_hover_height))
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower3))
CLIENT_send_immediate_command(follower3, 'takeoff_and_hover({})'.format(follower3_hover_height))

# Wait for follower ready. Blocking function.
wait_for_follower_ready(follower_host_tuple)

# Get leader current location.
leader_current_gps = __builtin__.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
print('{} - After taking off and hover, Leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
# Get leader current heading.
leader_current_heading = __builtin__.vehicle.heading
print('{} - Leader current heading is {} degree.'.format(time.ctime(), leader_current_heading))

# Generate a point, leader will fly to this point.
pointA = new_gps_coord_after_offset_inBodyFrame((leader_current_lat,leader_current_lon), leader_fly_distance, leader_current_heading, 0) # 0=Forward, 90=Right, 180=Backward, 270=Left.
print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

# Leader go to new location. Followers fly follow in square shape.
threading.Thread(target=goto_gps_location_relative, args=(pointA[0], pointA[1], leader_hover_height,),kwargs={'groundspeed':1}).start()
# When leader is not at destination location, keep sending follow fly command to followers.
# You can use threading to reduce the delay.
# Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) >0.5) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height)>0.3)):
    print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower3, 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee))
    time.sleep(0.5)

# When leader has reached destination, execute air_break().
# At the same time, send air_break command to all followers immediately.
threading.Thread(target=air_break, args=()).start()
for iter_follower in follower_host_tuple:
    print(iter_follower)
    CLIENT_send_immediate_command(iter_follower, 'air_break()')



# ===================== Formation 2 (Diamond) =====================
time.sleep(3)
# Shape 2 definition(Diamond).
# Follower 1.
follower1_hover_height = 22 # In meter.
follower1_distance_to_followee = 10 # In meter.
follower1_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 2.
follower2_hover_height = 24 # In meter.
follower2_distance_to_followee = 10 # In meter.
follower2_azimuth_to_followee = 135 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 3.
follower3_hover_height = 26 # In meter.
follower3_distance_to_followee = 14.4 # In meter.
follower3_azimuth_to_followee = 180 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.

# Change formation.
# 1) move follower3.
print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
CLIENT_send_immediate_command(follower3, 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.
# 2) move follower2.
print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.
# 3) move follower1.
print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.

# Get leader current location.
leader_current_gps = __builtin__.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
print('{} - In formation 2 (diamond), leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
# Get leader current heading.
leader_current_heading = __builtin__.vehicle.heading
print('{} - Leader current heading is {} degree.'.format(time.ctime(), leader_current_heading))

# Generate a point, leader will fly to this point.
pointA = new_gps_coord_after_offset_inBodyFrame((leader_current_lat,leader_current_lon), leader_fly_distance, leader_current_heading, 0) # 0=Forward, 90=Right, 180=Backward, 270=Left.
print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

# Leader go to new location.
threading.Thread(target=goto_gps_location_relative, args=(pointA[0], pointA[1], leader_hover_height,),kwargs={'groundspeed':1}).start()
# When leader is not at destination location, keep sending follow fly command to followers.
# You can use threading to reduce the delay.
# Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) >0.5) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height)>0.3)):
    print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower3, 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee))
    time.sleep(0.5)

# When leader has reached destination, execute air_break().
# At the same time, send air_break command to all followers immediately.
threading.Thread(target=air_break, args=()).start()
for iter_follower in follower_host_tuple:
    CLIENT_send_immediate_command(iter_follower, 'air_break()')

# ===================== Formation 3 (triangle) =====================
time.sleep(3)
# Shape 3 (triangle).
# Follower 1.
follower1_hover_height = 22 # In meter.
follower1_distance_to_followee = 14.4 # In meter.
follower1_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 2.
follower2_hover_height = 24 # In meter.
follower2_distance_to_followee = 10 # In meter.
follower2_azimuth_to_followee = 180 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
# Follower 3.
follower3_hover_height = 26 # In meter.
follower3_distance_to_followee = 14.4 # In meter.
follower3_azimuth_to_followee = 135 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.

# 1) move follower1.
print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.
# 2) move follower2.
print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.
# 1) move follower3.
print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
CLIENT_send_immediate_command(follower3, 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee))
time.sleep(5) # Give drone 5 seconds to get to its position.

# Get leader current location.
leader_current_gps = __builtin__.vehicle.location.global_relative_frame
leader_current_lat = leader_current_gps.lat
leader_current_lon = leader_current_gps.lon
leader_current_alt = leader_current_gps.alt
print('{} - In formation 3 (triangle), leader\'s GPS coordinate : lat={}, lon={}, alt_relative={}'.format(time.ctime(), leader_current_lat, leader_current_lon, leader_current_alt))
# Get leader current heading.
leader_current_heading = __builtin__.vehicle.heading
print('{} - Leader current heading is {} degree.'.format(time.ctime(), leader_current_heading))

# Generate a point, leader will fly to this point.
pointA = new_gps_coord_after_offset_inBodyFrame((leader_current_lat,leader_current_lon), leader_fly_distance, leader_current_heading, 0) # 0=Forward, 90=Right, 180=Backward, 270=Left.
print('{} - Leader is going to pointA : {}'.format(time.ctime(), pointA))

# Leader go to new location.
threading.Thread(target=goto_gps_location_relative, args=(pointA[0], pointA[1], leader_hover_height,),kwargs={'groundspeed':1}).start()
# When leader is not at destination location, keep sending follow fly command to followers.
# You can use threading to reduce the delay.
# Function prototype : fly_follow(followee_host, frame, height, radius_2D, azimuth)
while ((distance_between_two_gps_coord((__builtin__.vehicle.location.global_relative_frame.lat, __builtin__.vehicle.location.global_relative_frame.lon), (pointA[0], pointA[1])) >0.5) or (abs(__builtin__.vehicle.location.global_relative_frame.alt - leader_hover_height)>0.3)):
    print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower1, 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower2, 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee))
    print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
    CLIENT_send_immediate_command(follower3, 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee))
    time.sleep(0.5)

# When leader has reached destination, execute air_break().
# At the same time, send air_break command to all followers immediately.
threading.Thread(target=air_break, args=()).start()
for iter_follower in follower_host_tuple:
    CLIENT_send_immediate_command(iter_follower, 'air_break()')


# ===================== Mission completed, leader and followers go home =====================
# Wait for follower ready.
wait_for_follower_ready(follower_host_tuple)
print('{} - Mission completed. Return home.'.format(time.ctime()))

# Follower2 go home.
print('{} - Command follower2 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower2, 'return_to_launch()')
time.sleep(2)
# Follower3 go home.
print('{} - Command follower3 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower3, 'return_to_launch()')
time.sleep(2)
# Follower1 go home.
print('{} - Command follower1 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower1, 'return_to_launch()')
time.sleep(2)

# Leader drone go home.
print('{} - Followers have returned home, Leader is returning...'.format(time.ctime()))
return_to_launch()
print('{} - Leader has returned home.'.format(time.ctime()))
