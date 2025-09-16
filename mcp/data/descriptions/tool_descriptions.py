UNORDERED_TOP_Q_PLANNER_DESCRIPTION = "An unordered top-q planner within the context of a K* search-based algorithm finds all plans whose cost is at most q times the cost of an optimal plan, but it considers different orderings of the same set of actions to be equivalent. This contrasts with standard top-k or top-q planners that distinguish between plans with the same actions but different action sequences. The planner accepts problem and domain descriptions in Planning Domain Definition Language (PDDL) and returns a set of plans."
PDDL_DOMAIN_STRING_DESCRIPTION = "A PDDL domain string describes a general problem domain by defining the predicates (observable facts), types of objects, and types of actions (with their preconditions and effects) that are possible in that world."
PDDL_PROBLEM_STRING_DESCRIPTION = "A PDDL problem string defines a specific instance of a planning problem using the Planning Domain Definition Language (PDDL), specifying a problem's objects, the initial state of the world (what is currently true), and the goal state (what you want to achieve)."
TIMEOUT_DESCRIPTION = (
    "The number of seconds to limit the time a planner spends finding a plan."
)
QUALITY_BOUND_DESCRIPTION = 'A numerical value that serves as a filter for the solutions found by the K* planner. It is used to define the search space and limit the output to only "good enough" plans'
NUM_PLANS_DESCRIPTION = "The maximum number of plans the planner will attempt to find."
LOG_DESCRIPTION = "Log strings from a planner"
ERROR_MESSAGE_DESCRIPTION = "Error string from a planner"
TIMEOUT_TRIGGERED_DESCRIPTION = (
    "Indicates if the plan search was interrupted by a timeout"
)
COST_DESCRIPTION = "The cost of the plan."
PLANS_DESCRIPTION = "A list of plans found by the planner."
ACTIONS_DESCRIPTION = "A list of actions representing a plan, formatted according to the PDDL (Planning Domain Definition Language) standard. Each action represents a step in the plan and may contain parameters that specify the objects and conditions for that step.  These parameters are crucial for grounding the abstract action into a concrete execution step."
