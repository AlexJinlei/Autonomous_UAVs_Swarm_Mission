#!/bin/bash
# IMPORTANT: Path has to be absolute path.
today=`date '+%Y%m%d_%H-%M-%S'`;
filename="/home/iris2/Log/stderr_FlightLog_iris2_$today.txt";
python /home/iris2/iris_code/formation_main_follower.py > $filename 2>&1;
