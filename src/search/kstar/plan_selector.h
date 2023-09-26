#ifndef KSTAR_PLAN_SELECTOR_H
#define KSTAR_PLAN_SELECTOR_H

#include "../plugin.h"
#include "../task_proxy.h"

#include "../plan_manager.h"

#include <string>
#include <regex>

namespace kstar {

/*
Plan generator and storage (for duplicate check),
extending the plan
*/ 

class PlanCanonical {
    std::vector<int> plan_as_set;
    std::vector<int> plan_ordered;

public:

    PlanCanonical(const Plan& plan, bool keep_plans_unordered, const std::vector<bool>& to_preserve) {
        // Three options: 
        // 1. keep_plans_unordered is true: the entire plan goes into plan_as_set
        // 2. keep_plans_unordered is false, vector to_preserve is empty: the entire plan goes into plan_ordered
        // 3. keep_plans_unordered is false, vector to_preserve is not empty: the elements in to_preserve go into plan_ordered, the rest goes into plan_as_set 

        if (keep_plans_unordered) {
            for (OperatorID op : plan) {
                int op_no = op.get_index();
                plan_as_set.push_back(op_no);
            }
            std::sort(plan_as_set.begin(), plan_as_set.end());
        } else if (to_preserve.size() == 0) {
            for (OperatorID op : plan) {
                int op_no = op.get_index();
                plan_ordered.push_back(op_no);
            }
        } else {
            for (OperatorID op : plan) {
                int op_no = op.get_index();
                if (to_preserve[op_no])
                    plan_ordered.push_back(op_no);
                else
                    plan_as_set.push_back(op_no);
            }
            std::sort(plan_as_set.begin(), plan_as_set.end());
        }
    }

    virtual ~PlanCanonical() = default;

    bool operator==(const PlanCanonical &other) const {
        return plan_as_set == other.plan_as_set && plan_ordered == other.plan_ordered;
    }

    bool operator!=(const PlanCanonical &other) const {
        return !(*this == other);
    }

    const std::vector<int>& get_plan_as_set() const {
        return plan_as_set;
    }
    const std::vector<int>& get_plan_ordered() const {
        return plan_ordered;
    }
};




class PlanSelector {
    utils::HashSet<PlanCanonical> plans_sets;
    std::vector<bool> to_preserve;

protected:
    TaskProxy task_proxy;
    void get_plan_for_op_ids(const std::vector<int>& plan_ids, Plan& plan) const;
    bool keep_plans_unordered;
public:
    PlanSelector(const TaskProxy &task_proxy, bool keep_plans_unordered);

    virtual ~PlanSelector() = default;

    bool add_plan_if_necessary(const Plan& plan);
    
    void set_regex(std::string re);

    void clear() {
        plans_sets.clear();
    }

};

}

namespace utils {
inline void feed(HashState &hash_state, kstar::PlanCanonical plan) {
    feed(hash_state, std::make_pair(plan.get_plan_as_set(), plan.get_plan_ordered()));
}
}
#endif
