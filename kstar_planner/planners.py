#! /usr/bin/env python

import sys, json
import subprocess
import logging
from subprocess import SubprocessError
from pathlib import Path
from typing import Optional

import kstar_planner
build_dir = Path(kstar_planner.__file__).parent / 'builds' / 'release' / 'bin'
default_build_args = ["--build", str(build_dir.absolute())]

def run_planner(planner_args) -> dict:
    data = dict()

    try:
        import tempfile
        with tempfile.NamedTemporaryFile() as result_file:

            args_json_name = [a.replace("PLANS_JSON_NAME", str(result_file.name)) for a in planner_args]
            out = subprocess.run([sys.executable, "-B", "-m", "kstar_planner.driver.main"] + default_build_args + args_json_name, 
                           stdout = subprocess.PIPE, stderr = subprocess.PIPE)

            data["planner_output"] = out.stdout.decode()
            data["planner_error"] = out.stderr.decode()
            
            data["timeout_triggered"] = "search::time limit" in data["planner_output"]
            plans_file = Path(str(result_file.name))
            plans = []
            if plans_file.is_file() and plans_file.stat().st_size > 0:
                plans = json.loads(plans_file.read_text(encoding="UTF-8"))

            data["plans"] = plans
            data["unsolvable"] = len(data["plans"]) == 0 and not data["timeout_triggered"] and len(data["planner_error"]) == 0
            return data
        
    except SubprocessError as err:
        logging.error(err.output.decode())
        data["planner_output"] = err.output.decode()
        return data
    

def plan_unordered_topq(domain_file : Path, problem_file : Path, quality_bound : float, number_of_plans_bound : Optional[int] = None, timeout: Optional[int] = None, search_heuristic: Optional[str] = None) -> dict:
    # ORK*
    stopping = f'q={quality_bound}'
    if number_of_plans_bound:
        stopping += f', k={number_of_plans_bound}'
    if timeout:
        stopping += f', max_time={timeout}'

    heuristic = search_heuristic if search_heuristic else "lmcut(transform=undo_to_origin())"

    planner_args = [str(domain_file.absolute()), str(problem_file.absolute()), 
                    "--symmetries",  "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)", 
                    "--search",  f"kstar({heuristic}, {stopping}, find_unordered_plans=true, dump_plan_files=false, json_file_to_dump=PLANS_JSON_NAME, symmetries=sym, pruning=limited_pruning(pruning=atom_centric_stubborn_sets(use_sibling_shortcut=true, atom_selection_strategy=quick_skip)))"]
    
    return run_planner(planner_args)


def plan_topq(domain_file : Path, problem_file : Path, quality_bound : float, number_of_plans_bound : Optional[int] = None, timeout: Optional[int] = None, search_heuristic: Optional[str] = None) -> dict:
    # OK*
    stopping = f'q={quality_bound}'
    if number_of_plans_bound:
        stopping += f', k={number_of_plans_bound}'
    if timeout:
        stopping += f', max_time={timeout}'

    heuristic = search_heuristic if search_heuristic else "lmcut(transform=undo_to_origin())"

    planner_args = [str(domain_file.absolute()), str(problem_file.absolute()), 
                    "--symmetries",  "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)", 
                    "--search",  f"kstar({heuristic}, {stopping}, find_unordered_plans=false, dump_plan_files=false, json_file_to_dump=PLANS_JSON_NAME, symmetries=sym)"]
    
    return run_planner(planner_args)

def plan_topk(domain_file : Path, problem_file : Path, number_of_plans_bound : int, quality_bound : Optional[float] = None, timeout: Optional[int] = None, search_heuristic: Optional[str] = None) -> dict:
    # OK*
    stopping = f'k={number_of_plans_bound}'
    if quality_bound:
        stopping += f', q={quality_bound}'
    if timeout:
        stopping += f', max_time={timeout}'

    heuristic = search_heuristic if search_heuristic else "lmcut(transform=undo_to_origin())"

    planner_args = [str(domain_file.absolute()), str(problem_file.absolute()), 
                    "--symmetries",  "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)", 
                    "--search",  f"kstar({heuristic}, {stopping}, find_unordered_plans=false, dump_plan_files=false, json_file_to_dump=PLANS_JSON_NAME, symmetries=sym)"]
    
    return run_planner(planner_args)
    


