#!/bin/bash

# $1 domain
# $2 problem
# $3 number of plans (k)
# $4 json file name

RUNOPT="kstar(lmcut(),k=$3,dump_plan_files=false,json_file_to_dump=$4)"

LOG_FILE=$4.run.log

SOURCE="$( dirname "${BASH_SOURCE[0]}" )"
#echo $SOURCE
$SOURCE/fast-downward.py $1 $2 --search $RUNOPT > $LOG_FILE
