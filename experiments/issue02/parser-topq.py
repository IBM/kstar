#! /usr/bin/env python
# -*- coding: utf-8 -*-


import re
import os
import json
import itertools

from lab.parser import Parser

_PLAN_INFO_REGEX = re.compile(r"; cost = (\d+) \((unit cost|general cost)\)\n")

def get_plan_cost(path):
    try:
        with open(path) as input_file:
            line = None
            for line in input_file:
                if line.strip().startswith(";"):
                    continue
            # line is now the last line
            match = _PLAN_INFO_REGEX.match(line)
            if match:
                return int(match.group(1))
            return None
    except:
        return None


def get_plan_costs(plans_folder):
    ret = []
    for counter in itertools.count(1):
        name = "sas_plan.%d" % counter
        plan_filename = os.path.join(plans_folder, name)
        if not os.path.exists(plan_filename):
            break
        cost = get_plan_cost(plan_filename)
        if cost is not None:
            ret.append(cost)
    return ret


def plans(content, props):
    costs = get_plan_costs('found_plans')
    props["plan_costs"] = costs
    props["num_plans"] = len(costs)


def get_data_from_static():
    with open("static-properties", 'r') as sp:
        return json.load(sp)


def coverage(content, props):
    # If k is specified, then the coverage is 1 only if the number of found plans is smaller than k 
    if props.get("stop_on_k") == 1:
        props["coverage"] = int("total_time" in props and props["num_found_plans"] < props["k"])
    else:
        props["coverage"] = int("total_time" in props)


def kstar_coverage(content, props):
    k_bound_reached = proved_no_more_plans= False
    try:
        if props["normal_termination"] == 1:
            if props["num_plans"] >= props.get("k", 10000):
                k_bound_reached = True
            else:       
                # kstar terminated normally before finding desired number of plans
                proved_no_more_plans = True
    except:
        print("normal_termination missing -> can't compute kstar coverage")
    else:    
        props["kstar_coverage"] = int(k_bound_reached or proved_no_more_plans)
        props["kstar_coverage_ratio"] = float("{:.3f}".format(float(props["num_plans"]) / float(props["k"])))


def timer_values(content, props):
    try:
        props["after_goal_astar_time"] = props["total_astar_time"] - props["first_astar_time"]
    except:
        print("total/first_astar_time missing -> can't compute after_goal_astar_time")
    
    try:
        props["total_kstar_time"] = props["total_astar_time"] + props["total_eppstein_time"]
    except:
        print("total_astar/eppstein_time missing -> can't compute total_kstar_time")

def _get_states_pattern(attribute, name):
    return (attribute, rf"{name}=(\d+) state\(s\)\.", int)

first_astar_patterns = [
    ("fist_astar_evaluations", r"first_astar::Evaluations:=(.+)\n", int),
    _get_states_pattern("first_astar_dead_ends", "first_astar::Dead ends:"),
    _get_states_pattern("first_astar_evaluated", "first_astar::Evaluated"),
    _get_states_pattern("first_astar_expansions", "first_astar::Expanded"),
    _get_states_pattern("first_astar_generated", "first_astar::Generated"),
    _get_states_pattern("first_astar_reopened", "first_astar::Reopened"),
    _get_states_pattern("first_astar_evaluations_until_last_jump", "Evaluated until last jump:"),
    _get_states_pattern("first_astar_expansions_until_last_jump", "Expanded until last jump:"),
    _get_states_pattern("first_astar_generated_until_last_jump", "Generated until last jump:"),
    _get_states_pattern("first_astar_reopened_until_last_jump", "Reopened until last jump:")
]


def record_runfolder(content, props):
    props["run_folder"] = os.getcwd()


def multiplan_validation(content, props):
    # from validate_multiplan_solutions.validate_run import main as validate_run_main
    # class TempArg:
    #     def __init__(self):
    #         pass
    # args = TempArg()
    # args.planner="topk"
    # args.number_of_plans = props["k"]
    # args.run_folder = props["run_folder"]
    # args.plans_folder = "found_plans"
    # ret_code = validate_run_main(args)    
    # while ret_code == "RERUN":
    #     ret_code = validate_run_main(args)
    # if ret_code == "PASSED":
    #     props["validation_result"] = 1
    # else:
    #     props["validation_result"] = 0
    props["validation_result"] = 0
    

class KstarParser(Parser):
    def __init__(self):
        Parser.__init__(self)
        self.add_pattern("k", r"initialize::top-(\d+)", type=int)
        self.add_pattern("normal_termination", r"normal_termination=(\d+)", type=int)
        # steps
        self.add_pattern("total_steps", r"search::total_step_iter=(\d+)", type=int)
        self.add_pattern("total_astar_steps", r"search::total_num_astar_calls=(\d+)", type=int)
        self.add_pattern("total_eppsetein_steps", r"search::total_num_eppstein_calls=(\d+)", type=int)
        # times
        self.add_pattern("first_astar_time", r"first_astar::stop=(.+)s", type=float)
        self.add_pattern("total_astar_time", r"search::total astar time=(.+)s", type=float)
        self.add_pattern("total_eppstein_time", r"search::total eppstein time=(.+)s", type=float)
        self.add_pattern('search_time', r'Search time: (.+)s', required=False, type=float)
        self.add_pattern('total_time', r'Total time: (.+)s', required=False, type=float)
        self.add_pattern('first_plan_total_time', r'Total first plan time: (.+)s', required=False, type=float)
        self.add_pattern("num_found_plans", r"Found plans: (\d+)", type=int)
        self.add_pattern("dump_plans", r"Dumping plans to disk: (\d+)", type=int)
        self.add_pattern("stop_on_k", r"Termination criteria with top_k used=(\d+)", type=int)
        self.add_pattern("stop_on_q", r"Termination criteria with top_q used=(\d+)", type=int)


        for name, pattern, typ in first_astar_patterns:
            self.add_pattern(name, pattern, type=typ)
        
        self.add_function(plans)
        self.add_function(kstar_coverage)
        self.add_function(timer_values)
        self.add_function(record_runfolder)
        self.add_function(coverage)

        self.add_function(multiplan_validation)
        

def main():
    parser = KstarParser()
    parser.parse()

main()
