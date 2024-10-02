#! /usr/bin/env python
# -*- coding: utf-8 -*-

import itertools
import os

from lab.environments import LocalEnvironment
# from lab.reports import Attribute, geometric_mean
# from downward.reports.compare import ComparativeReport
from parser import KstarParser
import common_setup
from common_setup import IssueConfig, IssueExperiment

DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_NAME = os.path.splitext(os.path.basename(__file__))[0]
BENCHMARKS_DIR = os.environ["DOWNWARD_BENCHMARKS"]
BENCHMARKS_DIR = '/data/software/fd-symbolic-axioms/benchmarks'
REVISIONS = ["axioms"]
REVISIONS = ["88043694f02c44871b5fd31fdfd1fcdf4538a8f6"]
BUILDS = ["release"]

exp_k = 1000
CONFIG_NICKS = []

CONFIG_NICKS.append(('kstar', ['--search', f'kstar(blind(), k={exp_k})']))
CONFIG_NICKS.append(('okstar', ["--symmetries", "sym=structural_symmetries(time_bound=0,search_symmetries=oss,stabilize_initial_state=false,keep_operator_symmetries=true)",
                '--search', f'kstar(blind(), symmetries=sym, k={exp_k})']))

CONFIGS = [
    IssueConfig(
        config_nick,
        config,
        build_options=[build],
        driver_options=["--build", build])
    for build in BUILDS
    for config_nick, config in CONFIG_NICKS
]

SUITE = common_setup.DEFAULT_OPTIMAL_SUITE

# SUITE = ['grid', 'gripper', 'blocks']

SUITE = ['blocks-axioms', 'grid-axioms', 'miconic-axioms', 'optical-telegraphs', 'psr-middle', 'psr-large', 'philosophers', 'assembly', 'airport-adl', 'trucks',  'blocker', 'social-planning', 'sokoban-axioms', 'acc-cc2', 'grid-cc2', 'collab-and-comm', 'muddy-children', 'muddy-child', 'sum', 'word-rooms']


ENVIRONMENT = LocalEnvironment(processes=48)

exp = IssueExperiment(
    revisions=REVISIONS,
    configs=CONFIGS,
    environment=ENVIRONMENT,
    time_limit="30m",       # this soft-kills running executable
    memory_limit="3584M"
)
exp.set_property("k", exp_k)
exp.set_property("planner_time_limit", 1800)     # pass this to executable
exp.set_property("planner_memory_limit", "3.5g")

exp.add_suite(BENCHMARKS_DIR, SUITE)

exp.add_parser(exp.EXITCODE_PARSER)
exp.add_parser(exp.TRANSLATOR_PARSER)
exp.add_parser(exp.SINGLE_SEARCH_PARSER)
exp.add_parser(exp.PLANNER_PARSER)
exp.add_parser(KstarParser())

exp.add_step('build', exp.build)
exp.add_step('start', exp.start_runs)
exp.add_step("parse", exp.parse)

exp.add_fetcher(name='fetch')
# exp.add_parse_again_step()

KSTAR_ATTRIBUTES = [
    # plans
    "num_plans",
    "num_found_plans",
    "plan_costs",
    "plan_cost_min",
    "plan_cost_max",
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


