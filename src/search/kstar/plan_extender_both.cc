#include "plan_extender_both.h"

#include "../utils/logging.h"


namespace kstar {


PlanExtenderBoth::PlanExtenderBoth(const TaskProxy &task_proxy, bool keep_plans_unordered) : 
        PlanExtender(task_proxy, keep_plans_unordered),
        PlanExtenderSymmetries(task_proxy, keep_plans_unordered), 
        PlanExtenderReordering(task_proxy, keep_plans_unordered) {
}


void PlanExtenderBoth::extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) {
    int num_plans_total = (int) plans.size() + number_of_plans;
    // First, extend with reordering, then with symmetries 
    std::vector<Plan> plans_reordering;
    PlanExtenderReordering::extend_plan(plan, plans_reordering, number_of_plans);
    plans.insert(plans.end(), plans_reordering.begin(), plans_reordering.end());
    int number_of_plans_remaining = num_plans_total - plans.size();
    if (number_of_plans_remaining > 0) {
        // More plans are needed
        PlanExtenderSymmetries::extend_plan(plan, plans, number_of_plans_remaining);
    }
    for (const Plan& p : plans_reordering) {
        number_of_plans_remaining = num_plans_total - plans.size();
        if (number_of_plans_remaining <= 0) 
            return;
        // More plans are needed        
        PlanExtenderSymmetries::extend_plan(p, plans, number_of_plans_remaining);
    }
}


} 
