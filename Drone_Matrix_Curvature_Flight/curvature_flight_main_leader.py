# This is the main function for leader drone.
# Version 2.1

import time
import Queue
from datetime import datetime
import netifaces as ni
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import os, sys
sys.path.append(os.getcwd())
from circle_flight_function import *
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

# Follower tuple.
follower1 = iris2_host
follower2 = iris3_host
follower3 = iris4_host
follower_host_tuple = (follower1,follower2,follower3)

#================ 1) Operation on leader drones =========================

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

# Arm leader drone without RC.
arm_no_RC()

# Get GPS coordinate of leader's launch location.
leader_gps_home = __builtin__.vehicle.location.global_relative_frame
leader_lat_home = leader_gps_home.lat
leader_lon_home = leader_gps_home.lon
leader_alt_home = leader_gps_home.alt
print('{} - Home GPS coordinate :'.format(time.ctime()))
print('     leader_lat_home = {}'.format(leader_lat_home))
print('     leader_lon_home = {}'.format(leader_lon_home))
print('     leader_alt_home = {} (relative)'.format(leader_alt_home))

#================ 2) Operation on all drones =========================

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

# Set leader and followers formation parameters.
# DOUBLE CHECK these parameters before each flight mission.
# Leader hover height.
leader_hover_height = 35 # In meter.
# Follower1 hover height.
follower1_hover_height = 37 # In meter.
follower2_hover_height = 39 # In meter.
follower3_hover_height = 41 # In meter.


# Wait untill all followers are ready(armed).
wait_for_follower_ready(follower_host_tuple) # This is a blocking call.

# Take off and hover.
# Leader takeoff and hover.
threading.Thread(target=takeoff_and_hover, args=(leader_hover_height,)).start()
# Followers takeoff and hover. Send takeoff command to all followers. Immediate command must be in string type.
# Send command to follower1.
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower1))
CLIENT_send_immediate_command(follower1, 'takeoff_and_hover({})'.format(follower1_hover_height))
# Send command to follower2.
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower2))
CLIENT_send_immediate_command(follower2, 'takeoff_and_hover({})'.format(follower2_hover_height))
# Send command to follower3.
print('{} - Sending immediate command to : {}.'.format(time.ctime(), follower3))
CLIENT_send_immediate_command(follower3, 'takeoff_and_hover({})'.format(follower3_hover_height))

# Wait for all followers to be ready. Blocking function.
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


def keep_sending_fly_follow_command(control_word):
    global lock
    # Follower1.
    follower1_distance_to_followee = 5 # In meter.
    follower1_azimuth_to_followee = 225 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
    # Follower 2.
    follower2_distance_to_followee = 5 # In meter.
    follower2_azimuth_to_followee = 135 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.
    # Follower 3.
    follower3_distance_to_followee = 7.2 # In meter.
    follower3_azimuth_to_followee = 180 # In degree. 'body' frame: 0=Forwar, 90=Right; 'local' frame: 0=North, 90=East.

    while True:
        with lock:
            control_word_value = control_word.value
        #print('control_word_value = {}'.format(control_word_value))
        if (control_word_value=='T'): # T = Terminate
            # Send air_break() to all followers.
            CLIENT_send_immediate_command(follower1, 'air_break()')
            CLIENT_send_immediate_command(follower2, 'air_break()')
            CLIENT_send_immediate_command(follower3, 'air_break()')
            # Function return to stop thread.
            return
        elif (control_word_value=='P'): # P = Pause
            time.sleep(0.2)
        elif (control_word_value=='R'): # R = Run
            print('{} - Sending command fly_follow() to follower1.'.format(time.ctime()))
            follower1_command_str = 'fly_follow({}, {}, {}, {}, {})'.format(follower1_followee, follower1_frame_to_followee, follower1_hover_height, follower1_distance_to_followee, follower1_azimuth_to_followee)
            CLIENT_send_immediate_command(follower1, follower1_command_str)
            
            print('{} - Sending command fly_follow() to follower2.'.format(time.ctime()))
            follower2_command_str = 'fly_follow({}, {}, {}, {}, {})'.format(follower2_followee, follower2_frame_to_followee, follower2_hover_height, follower2_distance_to_followee, follower2_azimuth_to_followee)
            CLIENT_send_immediate_command(follower2, follower2_command_str)
            
            print('{} - Sending command fly_follow() to follower3.'.format(time.ctime()))
            follower3_command_str = 'fly_follow({}, {}, {}, {}, {})'.format(follower3_followee, follower3_frame_to_followee, follower3_hover_height, follower3_distance_to_followee, follower3_azimuth_to_followee)
            CLIENT_send_immediate_command(follower3, follower3_command_str)
            
            time.sleep(1)
        else:
            print('Incorrect control word value!')

# Create global lock instance.
__builtin__.lock = threading.Lock()
# Initialize a FLAG_control_word instance as Pause.
control_word = FLAG_control_word('P') # P = Pause.
# Start keep_sending_fly_follow_command() thread with control word.
THREAD_keep_sending_fly_follow_command = threading.Thread(target=keep_sending_fly_follow_command, args=(control_word,))
THREAD_keep_sending_fly_follow_command.start()

# Create action queue.
action_queue = Queue.Queue()
# Start thread execute_function_in_queue().
THREAD_execute_function_in_queue = threading.Thread(target=execute_function_in_queue, args=(action_queue,))
THREAD_execute_function_in_queue.start()


'''
# (horizontal_linear_speed, radius_of_curvature, total_turn_degree_deg, velocity_z, atom_segment)
action_queue.put((curvature_flight_body_frame, 2, 10, 180, 0, 1))
action_queue.put((curvature_flight_body_frame, 2, -10, 360, 0, 1))
action_queue.put((dummy_movement,))
action_queue.put((curvature_flight_body_frame, 2, 10, 180, 0, 1))
action_queue.put((air_break,))
action_queue.put((time.sleep, 5))
# Go to initial hover coordinate.
action_queue.put((goto_gps_location_relative, leader_current_lat, leader_current_lon, leader_current_alt))
# Restore inition heading.
action_queue.put((set_yaw, leader_current_heading, False))
'''

# Put actions in action_queue.
# Fly two complete circle to form an 8 shape.
action_queue.put((control_word.set_run,)) # Run keep_sending_fly_follow_command().
# curvature_flight_body_frame(horizontal_linear_speed, radius_of_curvature, total_turn_degree_deg, velocity_z, atom_segment)
action_queue.put((curvature_flight_body_frame, 2, 30, 360, 0, 1))
action_queue.put((curvature_flight_body_frame, 2, -30, 360, 0, 1))
#action_queue.put((control_word.set_pause,)) # Pause keep_sending_fly_follow_command().
action_queue.put((control_word.set_terminate,)) # Terminate the thread keep_sending_fly_follow_command().
action_queue.put((air_break,)) # Stop leader.
action_queue.put(('End',)) # 'End' must be in the end. Terminate execute_function_in_queue thread.
THREAD_execute_function_in_queue.join() # Wait thread to terminate.

# Go home.
# Follower1 go home.
print('{} - Command follower1 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower1, 'return_to_launch()')
time.sleep(2)
# Follower2 go home.
print('{} - Command follower2 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower2, 'return_to_launch()')
time.sleep(2)
# Follower3 go home.
print('{} - Command follower3 return home.'.format(time.ctime()))
CLIENT_send_immediate_command(follower3, 'return_to_launch()')
time.sleep(2)
# Leader go home.
print('{} - Followers have been ordered to return home, Leader is returning...'.format(time.ctime()))
return_to_launch()
print('{} - Leader has been orded to return home.'.format(time.ctime()))








