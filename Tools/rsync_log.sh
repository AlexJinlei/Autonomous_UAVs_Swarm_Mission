#!/bin/bash

###################################################################

echo 'Sync log on iris '$1'...'
log_path='/Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris'$1'_log'
echo 'Log path is : '$log_path
if [ ! -d $log_path ]; then mkdir $log_path; fi
# Sync Log on iris1:
rsync -rov iris$1@192.168.2.10$1:/home/iris$1/Log/* /Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris$1_log

# Delete files in /Log folder after Sync
#ssh iris1@192.168.2.101 'rm -f /home/iris1/Log/*'

###################################################################
