#! /usr/bin/env python

import argparse
import sys
from pathlib import Path

from kstar_planner.planners import plan_unordered_topq, plan_topq, plan_topk

DEFAULT_HEURISTIC_TIME_BOUND = 60

def get_heuristic(nick, overall_time_bound):
    if overall_time_bound is None:
        heuristic_time_bound = DEFAULT_HEURISTIC_TIME_BOUND
    else:
        heuristic_time_bound = min(DEFAULT_HEURISTIC_TIME_BOUND, overall_time_bound//2)
    HEURISTIC_CONFIGS = {
        "lmcut" : "lmcut(transform=undo_to_origin())",
        "ms" : "merge_and_shrink(transform=undo_to_origin(),shrink_strategy=shrink_bisimulation(greedy=false),merge_strategy=merge_sccs(order_of_sccs=topological,merge_selector=score_based_filtering(scoring_functions=[goal_relevance,dfp,total_order])),label_reduction=exact(before_shrinking=true,before_merging=false),max_states=50k,threshold_before_merge=1)",
        "msnpu" : "merge_and_shrink(prune_unreachable_states=false,transform=undo_to_origin(),shrink_strategy=shrink_bisimulation(greedy=false),merge_strategy=merge_sccs(order_of_sccs=topological,merge_selector=score_based_filtering(scoring_functions=[goal_relevance,dfp,total_order])),label_reduction=exact(before_shrinking=true,before_merging=false),max_states=50k,threshold_before_merge=1)",
        "cegar" : "cegar(transform=undo_to_origin())",
        "blind" : "blind(transform=undo_to_origin())",
        "ipdb" : f"cpdbs(transform=undo_to_origin(),patterns=hillclimbing(max_time={heuristic_time_bound}))",
    }
    return HEURISTIC_CONFIGS[nick]

def print_plans(plans):
    for i, plan in enumerate(plans):
        c = plan["cost"]
        print(f"Plan {i+1}, of cost {c}")
        for j, a in enumerate(plan["actions"]):
            print(f"{j+1}. ({a})")
        print()


def main():
    heuristics = ["lmcut", 
                #   "ms",
                  "ipdb", 
                  "cegar", 
                  "blind"]

    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    argparser.add_argument(dest="domain")
    argparser.add_argument(dest="problem")
    argparser.add_argument("-q","--quality-bound", type=float, default=None)
    argparser.add_argument("-k","--number-of-plans-bound", type=int, default=None)
    argparser.add_argument("-H","--heuristic", choices=heuristics, default="lmcut")
    argparser.add_argument('--unordered', action='store_true')
    argparser.add_argument("-t","--time-bound", type=int, default=None)

    args = argparser.parse_args()

    if args.quality_bound == None and args.number_of_plans_bound == None:
        print(
            "ERROR: at least one bound on either the number of plans or quality must be specified\n",
            file=sys.stderr,
        )
        argparser.print_help()
        sys.exit(2)

    if args.quality_bound == None and args.unordered:
        print(
            "ERROR: the quality bound is a required parameter for an unordered top-quality planner \n",
            file=sys.stderr,
        )
        argparser.print_help()
        sys.exit(2)

    heuristic = get_heuristic(args.heuristic, args.time_bound)

    result = None
    if args.unordered:
        result = plan_unordered_topq(domain_file=Path(args.domain), 
                                     problem_file=Path(args.problem),
                                     quality_bound=args.quality_bound,
                                     number_of_plans_bound=args.number_of_plans_bound,
                                     timeout=args.time_bound,
                                     search_heuristic=heuristic)
    elif args.number_of_plans_bound == None:
        result = plan_topq(domain_file=Path(args.domain), 
                                     problem_file=Path(args.problem),
                                     quality_bound=args.quality_bound,
                                     number_of_plans_bound=args.number_of_plans_bound,
                                     timeout=args.time_bound,
                                     search_heuristic=heuristic)
    else:
        result = plan_topk(domain_file=Path(args.domain), 
                                     problem_file=Path(args.problem),
                                     quality_bound=args.quality_bound,
                                     number_of_plans_bound=args.number_of_plans_bound,
                                     timeout=args.time_bound,
                                     search_heuristic=heuristic)        

    if "plans" in result and len(result["plans"]) > 0:
        print_plans(result["plans"])
    elif "unsolvable" in result and result["unsolvable"]:
        print("The problem was proved unsolvable by the planner")
    elif "timeout_triggered" in result and result["timeout_triggered"]:
        print("Timeout was reached before a solution could be found")
    else:
        print(f'The following error occurred: {result["planner_error"]}')
        

if __name__ == "__main__":
    main()