#!/bin/bash
# IMPORTANT: Path has to be absolute path.
today=`date '+%Y%m%d_%H-%M-%S'`;
filename="/home/iris1/Log/stderr_FlightLog_iris1_$today.txt";
python /home/iris1/iris_code/formation_main_leader.py > $filename 2>&1;
