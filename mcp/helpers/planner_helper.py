from pathlib import Path
import tempfile

from kstar_planner.planners import plan_unordered_topq
from helpers.file_helper import open_atomic


def get_plan_unordered_topq(
    domain: str,
    problem: str,
    timeout: int = 600,
    quality_bound: float = 1.0,
    num_plans: int = 1,
) -> dict:
    planner_result = dict()

    try:
        with tempfile.NamedTemporaryFile() as domain_temp, tempfile.NamedTemporaryFile() as problem_temp:
            domain_file = Path(tempfile.gettempdir()) / domain_temp.name
            problem_file = Path(tempfile.gettempdir()) / problem_temp.name

            with open_atomic(domain_file, "w") as domain_handle:
                domain_handle.write(domain)

            with open_atomic(problem_file, "w") as problem_handle:
                problem_handle.write(problem)

            planner_result = plan_unordered_topq(
                domain_file=domain_file,
                problem_file=problem_file,
                timeout=timeout,
                quality_bound=quality_bound,
                number_of_plans_bound=num_plans,
            )
    except Exception as e:
        planner_result["planner_error"] = str(e)

    return planner_result
