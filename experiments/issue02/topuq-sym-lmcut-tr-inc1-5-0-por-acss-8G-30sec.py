#! /usr/bin/env python
# -*- coding: utf-8 -*-

import itertools
import os

from lab.environments import LocalEnvironment
# from lab.reports import Attribute, geometric_mean
# from downward.reports.compare import ComparativeReport

import common_setup
from common_setup import IssueConfig, IssueExperiment

DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_NAME = os.path.splitext(os.path.basename(__file__))[0]
BENCHMARKS_DIR = os.environ["DOWNWARD_BENCHMARKS"]
REVISIONS = ["main", "issue02"]
BUILDS = ["release"]

exp_q = 1.0
CONFIG_NICKS = []

HEURISTICS = {
    "lmcut" : "lmcut(transform=undo_to_origin())",
    "ms" : "merge_and_shrink(transform=undo_to_origin(),shrink_strategy=shrink_bisimulation(greedy=false),merge_strategy=merge_sccs(order_of_sccs=topological,merge_selector=score_based_filtering(scoring_functions=[goal_relevance,dfp,total_order])),label_reduction=exact(before_shrinking=true,before_merging=false),max_states=50k,threshold_before_merge=1)",
    "msnpu" : "merge_and_shrink(prune_unreachable_states=false,transform=undo_to_origin(),shrink_strategy=shrink_bisimulation(greedy=false),merge_strategy=merge_sccs(order_of_sccs=topological,merge_selector=score_based_filtering(scoring_functions=[goal_relevance,dfp,total_order])),label_reduction=exact(before_shrinking=true,before_merging=false),max_states=50k,threshold_before_merge=1)",
    "cegar" : "cegar(transform=undo_to_origin())",
    "blind" : "blind(transform=undo_to_origin())",
    "ipdb" : "cpdbs(transform=undo_to_origin(),patterns=hillclimbing(max_time=60))",
}

params = [(1, 5, "false", "lmcut")]
for (inc_percent_l, inc_percent_u, sg, h) in params:
    hc = HEURISTICS[h]
    CONFIG_NICKS.append(('sym-uq-acss-%s-tr-%s-%s-%s-%s' % (h, exp_q, inc_percent_l, inc_percent_u, sg), 
        ["--symmetries", "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)", 
        '--search', 'kstar(%s, pruning=limited_pruning(pruning=atom_centric_stubborn_sets(use_sibling_shortcut=true, atom_selection_strategy=quick_skip)), symmetries=sym, q=%s, openlist_inc_percent_lb=%s, openlist_inc_percent_ub=%s, switch_on_goal=%s, extend_plans_with_symm=false, extend_plans_with_reordering=false, dump_plan_files=false, find_unordered_plans=true, json_file_to_dump=plans.json)' % (hc, exp_q, inc_percent_l, inc_percent_u, sg)]))



CONFIGS = [
    IssueConfig(
        config_nick,
        config,
        build_options=[build],
        driver_options=["--build", build, "--overall-memory-limit", "8192M", "--overall-time-limit", "30s"])
    for build in BUILDS
    for config_nick, config in CONFIG_NICKS
]

SUITE = common_setup.DEFAULT_OPTIMAL_SUITE

# SUITE = ['grid', 'gripper', 'blocks']

ENVIRONMENT = LocalEnvironment(processes=23)

exp = IssueExperiment(
    revisions=REVISIONS,
    configs=CONFIGS,
    environment=ENVIRONMENT,
    # time_limit="30m",       # this soft-kills running executable
    # memory_limit="3584M"
)
exp.set_property("q", exp_q)
exp.set_property("planner_time_limit", 1800)     # pass this to executable
exp.set_property("planner_memory_limit", "8g")

exp.add_suite(BENCHMARKS_DIR, SUITE)

exp.add_parser(exp.EXITCODE_PARSER)
exp.add_parser(exp.TRANSLATOR_PARSER)
exp.add_parser(exp.SINGLE_SEARCH_PARSER)
exp.add_parser(exp.PLANNER_PARSER)
exp.add_parser("parser-topq.py")

exp.add_step('build', exp.build)
exp.add_step('start', exp.start_runs)
exp.add_fetcher(name='fetch')
# exp.add_parse_again_step()

KSTAR_ATTRIBUTES = [
    # plans
    "num_plans",
    "num_found_plans",
    "dump_plans",
    "stop_on_k",
    "stop_on_q",
    "kstar_coverage",
    "kstar_coverage_ratio",
    # time in sec
    "total_kstar_time",
    "total_astar_time",
    "total_eppsetein_time",
    "first_astar_time",
    "after_goal_astar_time",
    # steps or iterations
    "total_steps",
    "total_astar_steps",
    "total_eppsetein_steps",
    #
    "first_astar_dead_ends",
    "first_astar_evaluated",
    "first_astar_expansions",
    "first_astar_generated",
    "first_astar_reopened",
    "first_astar_evaluations_until_last_jump",
    "first_astar_expansions_until_last_jump",
    "first_astar_generated_until_last_jump",
    "first_astar_reopened_until_last_jump",
    #
    "validation_result",
    "first_plan_total_time"
]

IssueExperiment.DEFAULT_TABLE_ATTRIBUTES += KSTAR_ATTRIBUTES
attributes = IssueExperiment.DEFAULT_TABLE_ATTRIBUTES
# exp.add_comparison_table_step(attributes=attributes)
exp.add_absolute_report_step(attributes=attributes)

exp.run_steps()


