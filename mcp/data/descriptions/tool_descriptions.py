UNORDERED_TOP_Q_PLANNER_DESCRIPTION = "An unordered top-q planner finds all plans whose cost is at most q times the cost of an optimal plan, skipping plans that are considered equivalent, deeming two plans equivalent if they differ only in the actions order. This contrasts with standard top-k or top-q planners that distinguish between plans that don't agree on actions orders. The planner accepts domain and problem descriptions in Planning Domain Definition Language (PDDL), a float quality bound relative to the optimal cost, an integer overall bound on the number of plans, and the timeout in seconds, and it returns a set of plans."
PDDL_DOMAIN_STRING_DESCRIPTION = "A PDDL domain string defines the rules of the world described by the planning problem. It includes several parts: requirements for a planner to support, objects type hierarchy, predicates (facts about objects according to their types), and actions (state-changing operations defined via parameters, preconditions, and effects)."
PDDL_PROBLEM_STRING_DESCRIPTION = "A PDDL problem string is paired to the PDDL domain, grounds it into a concrete instance by defining three main components: the objects with their types, the initial state (what is currently true), and the goal formula (what you want to achieve), in terms of the predicates defined in the paired PDDL domain string."
TIMEOUT_DESCRIPTION = (
    "The number of seconds to limit the time a planner spends finding the requested plans."
)
QUALITY_BOUND_DESCRIPTION = "A numerical value, a multiple of the optimal cost that bounds the quality of plans in the solutions found by the planner. For instance, the value of 1.2 ensures that the plans found are within 20% margin above the optimal plan cost."
NUM_PLANS_DESCRIPTION = "The maximum number of plans for the planner to find. The planner will stop once the number is reached."
LOG_DESCRIPTION = "Log strings from a planner"
ERROR_MESSAGE_DESCRIPTION = "Error string from a planner"
TIMEOUT_TRIGGERED_DESCRIPTION = (
    "Indicates if the plan search was interrupted by a timeout"
)
COST_DESCRIPTION = "The cost of the plan."
PLANS_DESCRIPTION = "A list of plans found by the planner."
ACTIONS_DESCRIPTION = "A list of actions representing a plan, formatted according to the PDDL (Planning Domain Definition Language) standard. Each action represents a step in the plan and may contain parameters that specify the objects and conditions for that step.  These parameters are crucial for grounding the abstract action into a concrete execution step."
