#include "plan_extender_reordering.h"
#include "../utils/timer.h"
#include "../utils/logging.h"
#include "../task_proxy.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"

#include <vector>


using namespace std;
namespace kstar {


PlanExtenderReordering::PlanExtenderReordering(const TaskProxy &_task_proxy, bool keep_plans_unordered) : PlanExtender(_task_proxy, keep_plans_unordered)
{
	if (keep_plans_unordered)
		std::cerr << "Warning! Extending plans with reorderings while keeping plans unordered. Please make sure that it is used as intended" << std::endl;
    registry = make_shared<StateRegistry>(task_proxy);
	successor_generator = utils::make_unique_ptr<successor_generator::SuccessorGenerator>(task_proxy);
}

void PlanExtenderReordering::plan_to_multiset(const Plan& plan, MultisetPlan &plan_multiset) const {
	for (OperatorID op : plan) {
		int op_no = op.get_index();
		auto it = plan_multiset.find(op_no);
        if (it == plan_multiset.end()) {
            plan_multiset[op_no] = 1;
        } else {
            plan_multiset[op_no]++;
        }
	}
}

size_t PlanExtenderReordering::num_appearances(OperatorID op, MultisetPlan &plan_multiset) const {
	int op_no = op.get_index();
	auto it = plan_multiset.find(op_no);
	if (it == plan_multiset.end()) {
		return 0;
	}
	return it->second;
}

void PlanExtenderReordering::extend_plans_dfs_naive_no_duplicate_detection(const Plan& plan, std::vector<Plan>& found_plans, int number_of_plans) {
	// Going over all possible orders in a DFS fashion (to be able to stop if enough plans were found)

	// utils::g_log << "Starting dumping plans in DFS manner, no duplicate detection ... " << endl;
	// utils::g_log << "Searching for plans of length " << plan.size() << endl;

	int num_found_plans = 0;
	vector<shared_ptr<DFSNode>> dfs_queue;

	if (add_plan_to_set(plan)) {
		found_plans.push_back(plan);
		num_found_plans++;
	}
	MultisetPlan plan_multiset;
	plan_to_multiset(plan, plan_multiset);

	const State& init = registry->get_initial_state();
	OperatorsProxy operators = task_proxy.get_operators();

	shared_ptr<DFSNode> init_node = make_shared<DFSNode>(init.get_id(), nullptr, -1, 0);
	dfs_queue.push_back(init_node);

	while (!dfs_queue.empty()) {
		if (num_found_plans >= number_of_plans) {
			// Checking if we can break here already - update the actual number of current optimal plans
			break;
		}
		// utils::g_log << "Getting node from the queue" << endl;
		shared_ptr<DFSNode> curr = dfs_queue.back();
		// curr->dump();
		dfs_queue.pop_back();
		const StateID& currID = curr->get_state();
		const State& current = registry->lookup_state(currID);

		// Check if done
		if (curr->get_plan_length() == (int) plan.size()) {
			if (task_properties::is_goal_state(task_proxy, current)) {
				Plan curr_plan;
    			// utils::g_log << "Goal node" << endl;
				curr->reconstruct_plan(curr_plan);
			    if (add_plan_to_set(curr_plan)) {
    				found_plans.push_back(curr_plan);
    				num_found_plans++;
					// utils::g_log << "Unique plan, dumping. Number of plans found so far is " << num_found_plans << endl;
			    }
			}
			// utils::g_log << "Goal depth, not going further" << endl;
			continue;
		}

		// Getting successors: getting the applicable operators, 
		//                     checking if on plan (and how many times) and 
		//                     checking if already applied along the path that many times.
		// utils::g_log << "Getting successors" << endl;
    	vector<OperatorID> applicable_ops;

    	successor_generator->generate_applicable_ops(current, applicable_ops);
        // bool successor_added = false;
	    for (OperatorID op_id : applicable_ops) {
			OperatorProxy op = operators[op_id];
			// utils::g_log << op.get_name() << endl;
			size_t cnt = num_appearances(op_id, plan_multiset);
			if (cnt == 0) {
				// utils::g_log << "Successor not on plan" << endl;
				continue;
			}
			int op_no = op_id.get_index();
			if (curr->is_already_applied(op_no, cnt)) {
				// utils::g_log << "Successor already applied to the path as many times as on the plan" << endl;
				continue;
			}
			// Now we can create the successor
			const State& next = registry->get_successor_state(current, op);
			//utils::g_log << "Operator: " << op.get_name() << " generates node " << next.get_id() << endl;
			// Creating a new node
			shared_ptr<DFSNode> next_node = make_shared<DFSNode>(next.get_id(), curr, op_no, curr->get_plan_length() + 1);
			dfs_queue.push_back(next_node);
            // successor_added = true;
		}
        // if (!successor_added) {
        //     utils::g_log << "No successor added at level " <<  curr->get_plan_length() + 1 << ", queue size " << dfs_queue.size() << endl;
        // }
	}
}

void PlanExtenderReordering::extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) {
    // Extending the given plans with reordering, until all found or number_of_plans is reached
    //TODO: Think of a more memory efficient way of doing that.
    //TODO: Dump plans as we go

	extend_plans_dfs_naive_no_duplicate_detection(plan, plans, number_of_plans);
}


}
