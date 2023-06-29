#ifndef KSTAR_PLAN_EXTENDER_SYMMETRIES_H
#define KSTAR_PLAN_EXTENDER_SYMMETRIES_H

#include "../search_engines/search_common.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"
#include "../utils/hash.h"

#include "../structural_symmetries/group.h"
#include "../plan_manager.h"

#include "plan_extender.h"

class Group;

namespace kstar {

/*
Plan generator and storage (for duplicate check),
extending the given set of plans with symmetries.
*/ 


class PlanExtenderSymmetries : virtual public PlanExtender {
    std::shared_ptr<Group> symmetry_group;
public:

    PlanExtenderSymmetries(const TaskProxy &task_proxy, bool keep_plans_unordered);

    virtual ~PlanExtenderSymmetries() = default;

    // Extending the given plans with symmetries, until all found or number_of_plans is reached
    virtual void extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) override;

};

}
#endif
