#include "undo_single_goal_task.h"

#include "../operator_cost.h"
#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"
#include "../tasks/root_task.h"
#include "../utils/system.h"

#include <iostream>
#include <memory>

using namespace std;
using utils::ExitCode;

namespace tasks {

UndoSingeGoalTask::UndoSingeGoalTask(
    const shared_ptr<AbstractTask> &parent)
    : DelegatingTask(parent) {
}

int UndoSingeGoalTask::get_num_variables() const {
    return parent->get_num_variables() - 1;
}

int UndoSingeGoalTask::get_variable_domain_size(int var) const {
    return parent->get_variable_domain_size(var) - 1;
}

int UndoSingeGoalTask::get_num_operators() const {
    return parent->get_num_operators() - 1;
}

int UndoSingeGoalTask::get_num_operator_preconditions(int index, bool is_axiom) const {
    return parent->get_num_operator_preconditions(index, is_axiom) - (is_axiom ? 0 : 1);
}

int UndoSingeGoalTask::get_num_goals() const {
    // Taking from the precondition of the last parent operator
    int op_index = parent->get_num_operators() - 1;
    return parent->get_num_operator_preconditions(op_index, false) - 1;
}

FactPair UndoSingeGoalTask::get_goal_fact(int index) const {
    // Taking from the precondition of the last parent operator
    int op_index = parent->get_num_operators() - 1;
    return parent->get_operator_precondition(op_index, index, false);
}

std::vector<int> UndoSingeGoalTask::get_initial_state_values() const {
    std::vector<int> init = parent->get_initial_state_values();
    init.pop_back();
    return init;
}

void UndoSingeGoalTask::convert_state_values_from_parent(std::vector<int> &values) const {
    // Check if the last value is 1, then assign a goal state. 
    // We choose to assign 0 to all non-goal variables.
    if (values.back() == 1) {
        for (int i = 0 ; i < (int) values.size() - 1 ; ++i ) {
            values[i] = 0;
        }
        for (int i = 0 ; i < get_num_goals() ; ++i ) {
            FactPair fact = get_goal_fact(i);
            values[fact.var] = fact.value;
        }
    } 
    values.pop_back();
}


static shared_ptr<AbstractTask> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Original task, assuming transformation is made on the root task",
        "The transformation, performed on a root task removes the changes made to create a single goal state.");
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    } else {
        return make_shared<UndoSingeGoalTask>(g_root_task);
    }
}

static Plugin<AbstractTask> _plugin("undo_to_origin", _parse);
}
