#ifndef TASKS_UNDO_SINGLE_GOAL_TASK_H
#define TASKS_UNDO_SINGLE_GOAL_TASK_H

#include "delegating_task.h"

#include "../operator_cost.h"

namespace options {
class Options;
}

namespace tasks {
/*
  Task transformation that changes transformed operator costs back.
  The transformation in the input is 
  c -> c*N + 1

  The transformation back is 
  c -> (c-1)/N

  Regardless of the cost_type value, axioms will always keep their original
  cost, which is 0 by default.
*/
class UndoSingeGoalTask : public DelegatingTask {
public:
    UndoSingeGoalTask(
        const std::shared_ptr<AbstractTask> &parent);
    virtual ~UndoSingeGoalTask() override = default;

    virtual int get_variable_domain_size(int var) const override;
    virtual int get_num_variables() const override;
    virtual int get_num_operators() const override;

    virtual int get_num_operator_preconditions(int index, bool is_axiom) const override;

    virtual int get_num_goals() const override;
    virtual FactPair get_goal_fact(int index) const override;

    virtual std::vector<int> get_initial_state_values() const override;

    virtual void convert_state_values_from_parent(std::vector<int> &) const override;


};
}

#endif
