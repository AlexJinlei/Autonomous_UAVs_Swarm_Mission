#!/bin/bash

###################################################################

# Usage: . rsync_frame.sh 1 #1 means iris1.

# Sync saved_frames on iris1:

echo 'Packing on server...'
ssh 'iris'$1'@192.168.2.10'$1 'tar -cvzf saved_frames.tar.gz -C /home/iris'$1'/saved_frames/ .'

echo 'Downloading packed file...'
rsync -rov --progress 'iris'$1'@192.168.2.10'$1':/home/iris'$1'/saved_frames.tar.gz' '/Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris'$1'_saved_frames.tar.gz'

echo 'Extract files from downloaded package...'
folder_to_save_frames='/Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris'$1'_saved_frames/'
if [ ! -d $folder_to_save_frames ]; then mkdir $folder_to_save_frames; fi
tar -xvzf '/Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris'$1'_saved_frames.tar.gz' -C $folder_to_save_frames

echo 'Deleting packed file on local...'
rm '/Users/jz0006/GoogleDrive/Electronics/balloon_finder/iris'$1'_saved_frames.tar.gz'

echo 'Deleting packed file on server...'
ssh 'iris'$1'@192.168.2.10'$1 'rm -f /home/iris'$1'/saved_frames.tar.gz'

# Create video.
#ffmpeg -framerate 15 -pattern_type glob -i '*detected.jpg' ../iris.mp4
###################################################################
