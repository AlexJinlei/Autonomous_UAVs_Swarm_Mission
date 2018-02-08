# This is the main function for follower drone.

import time
from datetime import datetime
import netifaces as ni
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import threading
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
is_leader = False
if is_leader:
    print('{} - This is a leader drone.'.format(time.ctime()))
else:
    print('{} - This is a follower drone.'.format(time.ctime()))

print('{} - local_host = {}.'.format(time.ctime(), local_host))
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))

# Create global variable to indicate follower status.
__builtin__.status_waitForCommand = False

# Reserved port.
# The port number should be exactly the same as that in leader drone.
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

# Self arm.
print('{} - Self arming...'.format(time.ctime()))
arm_no_RC() # Blocking call.
# Once armed, change status_waitForCommand to True.
__builtin__.status_waitForCommand = True
print('{} - __builtin__.status_waitForCommand = {}'.format(time.ctime(), __builtin__.status_waitForCommand))
print('{} - Follower is armed!'.format(time.ctime()))





