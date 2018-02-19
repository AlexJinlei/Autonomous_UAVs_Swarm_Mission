#!/bin/bash
# version 2.2

###################################################################

# Sync code on iris1:
echo '===================================================='
echo 'Uploading code to iris1...'
rsync -rov --exclude '*.pyc' --exclude 'balloon_finder_follower.py' --exclude '.DS_Store' /Users/jz0006/GoogleDrive/Electronics/balloon_finder/balloon_finder_v2.2/ iris1@192.168.2.101:./iris_code
rsync -rov /Users/jz0006/GoogleDrive/Electronics/balloon_finder/sync_script/run_on_boot_iris1.sh iris1@192.168.2.101:./run_on_boot.sh

###################################################################

# Sync code on iris2:
echo '===================================================='
echo 'Uploading code to iris2...'
rsync -rov --exclude '*.pyc' --exclude 'balloon_finder_leader.py' --exclude '.DS_Store' /Users/jz0006/GoogleDrive/Electronics/balloon_finder/balloon_finder_v2.2/ iris2@192.168.2.102:./iris_code
rsync -rov /Users/jz0006/GoogleDrive/Electronics/balloon_finder/sync_script/run_on_boot_iris2.sh iris2@192.168.2.102:./run_on_boot.sh

###################################################################

# Sync code on iris3:
echo '===================================================='
echo 'Uploading code to iris3...'
rsync -rov --exclude '*.pyc' --exclude 'balloon_finder_leader.py' --exclude '.DS_Store' /Users/jz0006/GoogleDrive/Electronics/balloon_finder/balloon_finder_v2.2/ iris3@192.168.2.103:./iris_code
rsync -rov /Users/jz0006/GoogleDrive/Electronics/balloon_finder/sync_script/run_on_boot_iris3.sh iris3@192.168.2.103:./run_on_boot.sh

###################################################################

# Sync code on iris4:
echo '===================================================='
echo 'Uploading code to iris4...'
rsync -rov --exclude '*.pyc' --exclude 'balloon_finder_leader.py' --exclude '.DS_Store' /Users/jz0006/GoogleDrive/Electronics/balloon_finder/balloon_finder_v2.2/ iris4@192.168.2.104:./iris_code
rsync -rov /Users/jz0006/GoogleDrive/Electronics/balloon_finder/sync_script/run_on_boot_iris4.sh iris4@192.168.2.104:./run_on_boot.sh

###################################################################
