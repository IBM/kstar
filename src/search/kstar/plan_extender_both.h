#ifndef KSTAR_PLAN_EXTENDER_BOTH
#define KSTAR_PLAN_EXTENDER_BOTH

#include "../task_proxy.h"
#include "../utils/hash.h"

#include "../plan_manager.h"

#include "plan_extender_symmetries.h"
#include "plan_extender_reordering.h"


namespace kstar {

/*
Plan generator and storage (for duplicate check),
extending the given set of plans with symmetries and reordering.
*/ 


class PlanExtenderBoth : public PlanExtenderSymmetries, PlanExtenderReordering {
public:

    PlanExtenderBoth(const TaskProxy &task_proxy, bool keep_plans_unordered);

    virtual ~PlanExtenderBoth() = default;

    // Extending the given plans with symmetries, until all found or number_of_plans is reached
    virtual void extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) override;

};

} 
#endif
