#!/bin/bash

# $1 domain
# $2 problem
# $3 quality bound (q)
# $4 upper bound on the number of plans (k)
# $5 json file name

RUNOPT="kstar(ipdb(),symmetries=sym,q=$3,k=$4,dump_plan_files=false,json_file_to_dump=$5,pruning=limited_pruning(pruning=atom_centric_stubborn_sets(use_sibling_shortcut=true,atom_selection_strategy=quick_skip)),find_unordered_plans=true)"

LOG_FILE=$5.run.log

SOURCE="$( dirname "${BASH_SOURCE[0]}" )"
#echo $SOURCE
$SOURCE/fast-downward.py $1 $2 --translate-options --keep-unimportant-variables --search-options --symmetries "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)" --search $RUNOPT > $LOG_FILE