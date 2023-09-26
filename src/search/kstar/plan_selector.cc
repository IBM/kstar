#include "plan_selector.h"

#include "../utils/logging.h"


namespace kstar {


PlanSelector::PlanSelector(const TaskProxy &task_proxy, bool keep_plans_unordered) : task_proxy(task_proxy), keep_plans_unordered(keep_plans_unordered) {
}

void PlanSelector::get_plan_for_op_ids(const std::vector<int>& plan_ids, Plan& plan) const {
    for (int op_no : plan_ids) {
        plan.push_back(OperatorID(op_no));
    }
}

bool PlanSelector::add_plan_if_necessary(const Plan& plan)
{
    PlanCanonical canonical(plan, keep_plans_unordered, to_preserve);
    return plans_sets.insert(canonical).second;
}

void PlanSelector::set_regex(std::string re) {
    OperatorsProxy operators = task_proxy.get_operators();
    to_preserve.assign(operators.size(), false);

    for (OperatorProxy op : operators) {      
        if (std::regex_match (op.get_name(), std::regex(re) )) {
            to_preserve[op.get_id()] = true;
        }
    }
}

}
