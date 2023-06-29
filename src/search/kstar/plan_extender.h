#ifndef KSTAR_PLAN_EXTENDER_H
#define KSTAR_PLAN_EXTENDER_H

#include "../plugin.h"
#include "../task_proxy.h"

#include "../plan_manager.h"


namespace kstar {

/*
Plan generator and storage (for duplicate check),
extending the plan
*/ 


class PlanExtender {
    utils::HashSet<std::vector<int>> plans_sets;

protected:
    TaskProxy task_proxy;
    void get_plan_for_op_ids(const std::vector<int>& plan_ids, Plan& plan) const {
        for (int op_no : plan_ids) {
            plan.push_back(OperatorID(op_no));
        }
    }
    bool add_plan_to_set(const std::vector<int>& plan);
    bool keep_plans_unordered;
public:

    PlanExtender(const TaskProxy &task_proxy, bool keep_plans_unordered);

    virtual ~PlanExtender() = default;

    // Extending the given plans, until all found or number_of_plans is reached
    virtual void extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans);
    
    void clear() {
        plans_sets.clear();
    }
    bool add_plan_to_set(const Plan& plan);

};

}
#endif
