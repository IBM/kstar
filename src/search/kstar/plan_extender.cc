#include "plan_extender.h"

#include "../utils/logging.h"

#include <unordered_set>

namespace kstar {


PlanExtender::PlanExtender(const TaskProxy &task_proxy, bool keep_plans_unordered) : task_proxy(task_proxy), keep_plans_unordered(keep_plans_unordered) {
}

bool PlanExtender::add_plan_to_set(const std::vector<int>& plan) {
    if (keep_plans_unordered) {
        // Copying for sorting
        std::vector<int> found_plan(plan.begin(), plan.end());
        std::sort(found_plan.begin(), found_plan.end());
        return plans_sets.insert(found_plan).second;
    }
    return plans_sets.insert(plan).second;
}

bool PlanExtender::add_plan_to_set(const Plan& plan)
{
    std::vector<int> found_plan;
    for (OperatorID op : plan) {
        int op_no = op.get_index();
        found_plan.push_back(op_no);
    }
    if (keep_plans_unordered)
        std::sort(found_plan.begin(), found_plan.end());

    return plans_sets.insert(found_plan).second;
}

void PlanExtender::extend_plan(const Plan& plan, std::vector<Plan>& plans, int ) {
    if (add_plan_to_set(plan)) { // New plan 
        plans.push_back(plan);
    } 
}
}
