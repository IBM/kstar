import subprocess

import pytest

from kstar_planner import planners
from kstar_planner.driver import limits
from kstar_planner.driver import returncodes


PLANNER_CALLS = [
    (planners.plan_unordered_topq, {"quality_bound": 2}),
    (planners.plan_topq, {"quality_bound": 2}),
    (planners.plan_topk, {"number_of_plans_bound": 2}),
]


@pytest.mark.parametrize("planner, planner_options", PLANNER_CALLS)
def test_timeout_is_passed_to_driver_and_search(
    monkeypatch, tmp_path, planner, planner_options
):
    monkeypatch.setattr(planners.limits, "can_set_time_limit", lambda: True)
    monkeypatch.setattr(planners, "run_planner", lambda args: args)

    domain = tmp_path / "domain.pddl"
    problem = tmp_path / "problem.pddl"
    args = planner(domain, problem, timeout=30, **planner_options)

    assert args[:4] == [
        "--overall-time-limit",
        "30s",
        str(domain.absolute()),
        str(problem.absolute()),
    ]
    assert "max_time=30" in args[-1]


def test_timeout_keeps_search_limit_when_driver_limit_is_unsupported(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(planners.limits, "can_set_time_limit", lambda: False)
    monkeypatch.setattr(planners, "run_planner", lambda args: args)

    args = planners.plan_topk(
        tmp_path / "domain.pddl",
        tmp_path / "problem.pddl",
        number_of_plans_bound=2,
        timeout=30,
    )

    assert "--overall-time-limit" not in args
    assert "max_time=30" in args[-1]


def test_remaining_overall_limit_is_an_integer(monkeypatch):
    monkeypatch.setattr(limits.util, "get_elapsed_time", lambda: 0.25)

    assert limits.get_time_limit(None, 30) == 29


def test_translation_timeout_is_reported(monkeypatch):
    completed_process = subprocess.CompletedProcess(
        args=[],
        returncode=returncodes.TRANSLATE_OUT_OF_TIME,
        stdout=b"",
        stderr=b"",
    )
    monkeypatch.setattr(planners.subprocess, "run", lambda *args, **kwargs: completed_process)

    result = planners.run_planner([])

    assert result["timeout_triggered"] is True
    assert result["unsolvable"] is False


def test_search_timeout_output_is_still_reported(monkeypatch):
    completed_process = subprocess.CompletedProcess(
        args=[],
        returncode=returncodes.SEARCH_UNSOLVED_INCOMPLETE,
        stdout=b"search::time limit reached",
        stderr=b"",
    )
    monkeypatch.setattr(planners.subprocess, "run", lambda *args, **kwargs: completed_process)

    result = planners.run_planner([])

    assert result["timeout_triggered"] is True
    assert result["unsolvable"] is False
