#ifndef KSTAR_PLAN_EXTENDER_REORDERING_H
#define KSTAR_PLAN_EXTENDER_REORDERING_H

#include "../state_registry.h"
#include "../search_engine.h"
#include "../task_proxy.h"
#include "../state_id.h"
#include "../task_utils/successor_generator.h"

#include "plan_extender.h"

#include <map>
#include <memory>
#include <vector>



namespace kstar {

typedef utils::HashSet<Plan> PlansSet;
typedef std::map<int, size_t> MultisetPlan;

class DFSNodeRecursive {

    void reconstruct_recursive_plan(Plan &plan) const {
		if (achieving_op == -1)
			return;
		plan.push_back(OperatorID(achieving_op));
		parent->reconstruct_recursive_plan(plan);
    }

public:
    StateID state_id;
    std::shared_ptr<DFSNodeRecursive> parent;
    int achieving_op;
    int plan_length;
    DFSNodeRecursive(StateID _state_id, std::shared_ptr<DFSNodeRecursive> _parent, int _achieving_op, int _plan_cost) : state_id(_state_id), parent(_parent), achieving_op(_achieving_op), plan_length(_plan_cost) {}

   	const StateID& get_state() const {return state_id; }

   	void dump() const {
   		std::cout << "StateID: " << state_id << std::endl;
   		std::cout << "Achieving op: " << achieving_op << std::endl;
   		std::cout << "Sequence length: " << plan_length << std::endl;
   	}

	int get_plan_length() const {
		return plan_length;
	}
    void reconstruct_plan(Plan &plan) const {
		reconstruct_recursive_plan(plan);
		// Revert plan
		std::reverse(plan.begin(),plan.end());
    }

	bool is_already_applied(int op_no, size_t cnt) const {
		if (cnt == 0 || achieving_op == -1) 
			return false;
		
		if (achieving_op == op_no) {
			if (cnt == 1)
				return true;
			return parent->is_already_applied(op_no, cnt - 1);
		}
		return parent->is_already_applied(op_no, cnt);
	}
};

class DFSNodeDirect {

    void reconstruct_recursive_plan(Plan &plan) const {
		if (achieving_op == -1)
			return;
		plan.push_back(OperatorID(achieving_op));
		parent->reconstruct_recursive_plan(plan);
    }

	MultisetPlan plan_multiset;
public:
    StateID state_id;
    std::shared_ptr<DFSNodeDirect> parent;
    int achieving_op;
    int plan_length;
    DFSNodeDirect(StateID _state_id, std::shared_ptr<DFSNodeDirect> _parent, int _achieving_op, int _plan_cost) : 
			state_id(_state_id), 
			parent(_parent), 
			achieving_op(_achieving_op), 
			plan_length(_plan_cost) {
		if (achieving_op != -1) {
			auto it = plan_multiset.find(achieving_op);
			if (it == plan_multiset.end()) {
				plan_multiset[achieving_op] = 1;
			} else {
				plan_multiset[achieving_op]++;
			}
		}
	}

   	const StateID& get_state() const {return state_id; }

   	void dump() const {
   		std::cout << "StateID: " << state_id << std::endl;
   		std::cout << "Achieving op: " << achieving_op << std::endl;
   		std::cout << "Sequence length: " << plan_length << std::endl;
   	}

	int get_plan_length() const {
		return plan_length;
	}
    void reconstruct_plan(Plan &plan) const {
		reconstruct_recursive_plan(plan);
		// Revert plan
		std::reverse(plan.begin(),plan.end());
    }

	bool is_already_applied(int op_no, size_t cnt) const {
		if (cnt == 0 || achieving_op == -1) 
			return false;

		size_t curr_count = 0;
		auto it = plan_multiset.find(op_no);
		if (it != plan_multiset.end()) {
			curr_count = it->second;
		}
		return (curr_count >= cnt);
	}
};

typedef DFSNodeRecursive DFSNode;
// typedef DFSNodeDirect DFSNode;

class PlanExtenderReordering : virtual public PlanExtender {
	// typedef utils::HashSet<Plan> PlansSet;
    // typedef std::map<int, size_t> MultisetPlan;

private:
	std::shared_ptr<StateRegistry> registry;
	std::unique_ptr<successor_generator::SuccessorGenerator> successor_generator;

	size_t num_appearances(OperatorID op, MultisetPlan& plan_multiset) const;
	void plan_to_multiset(const Plan &plan, MultisetPlan& plan_multiset) const;
    void extend_plans_dfs_naive_no_duplicate_detection(const Plan& plan, std::vector<Plan> &found_plans, int number_of_plans);

public:
    PlanExtenderReordering(const TaskProxy& _task_proxy, bool keep_plans_unordered);
    virtual ~PlanExtenderReordering() = default;

	// The main function to use
    virtual void extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) override;

};
}
#endif
