#include "plan_extender_symmetries.h"

#include "../structural_symmetries/group.h"
#include "../structural_symmetries/permutation.h"
#include "../structural_symmetries/operator_permutation.h"

#include "../utils/logging.h"

#include <unordered_set>

namespace kstar {


PlanExtenderSymmetries::PlanExtenderSymmetries(const TaskProxy &task_proxy, bool keep_plans_unordered) : PlanExtender(task_proxy, keep_plans_unordered) {
        Options opts;
        opts.set("search_symmetries", SearchSymmetries::DKS);
        opts.set<bool>("stabilize_initial_state", true);
        opts.set<bool>("keep_operator_symmetries", true);
        opts.set<bool>("keep_state_identity_operator_symmetries", false);        
        opts.set<bool>("stabilize_goal", true);
        opts.set<bool>("use_color_for_stabilizing_goal", true);
        opts.set<int>("time_bound", 0);

        opts.set<bool>("dump_symmetry_graph", false);
        opts.set<bool>("dump_permutations", false);
        opts.set<bool>("write_search_generators", false);
        opts.set<bool>("write_all_generators", false);
        
        symmetry_group = std::make_shared<Group>(opts);
        if (!symmetry_group->is_initialized()) 
        {
            utils::g_log << "Initializing symmetries for extending the set of plans" << std::endl;
            symmetry_group->compute_symmetries(task_proxy);
            // for (int i = 0; i < symmetry_group->get_num_operator_symmetries(); i++) {
            //     utils::g_log << "Operators mapping by generator " << i << std::endl;
            //     symmetry_group->get_operator_permutation(i).dump(task_proxy);
            // }
        }
}


void PlanExtenderSymmetries::extend_plan(const Plan& plan, std::vector<Plan>& plans, int number_of_plans) {
    // Extending the given plans with symmetries, until all found or number_of_plans is reached
    // Going over the symmetries until fixed point is reached
    //TODO: Think of a more memory efficient way of doing that.
    //TODO: Dump plans as we go

    // Start by copying plans into vectors of ids
    std::vector<std::vector<int>> frontier;
    std::vector<int> found_plan;
    for (OperatorID op : plan) {
        int op_no = op.get_index();
        found_plan.push_back(op_no);
    }
    if (add_plan_to_set(found_plan)) { // New plan 
        frontier.push_back(found_plan);
        plans.push_back(plan);
    } 

    // utils::g_log << "Finding symmetric plans (bound "<< number_of_plans << ")..." << std::flush;
    int num_symmetries = symmetry_group->get_num_operator_symmetries();

    while (number_of_plans > (int)plans.size()) {
        // Applying all symmetries to all plans in a frontier. The new ones become the new frontier.
        std::vector<std::vector<int>> new_frontier;
        bool change = false;
        for (int i=0; i < num_symmetries; ++i) {
            const OperatorPermutation& op_sym = symmetry_group->get_operator_permutation(i);

            for (std::vector<int> current_plan : frontier) {
                // Permuting the plan, adding to found plans, and if new, to new frontier
                std::vector<int> permuted_plan;
                for (int op_no : current_plan) {
                    int permuted_op_no = op_sym.get_permuted_operator_no(op_no);
                    permuted_plan.push_back(permuted_op_no);
                }
                if (add_plan_to_set(permuted_plan)) { // New plan 
                    // Element inserted, adding to the new frontier
                    Plan p;
                    get_plan_for_op_ids(permuted_plan, p);
                    plans.push_back(p);
                    new_frontier.push_back(permuted_plan);
                    change = true;
                }
                if (number_of_plans <= (int)plans.size())
                    break;
            }
            if (number_of_plans <= (int)plans.size())
                break;
        }
        if (!change)
            break;
        frontier.swap(new_frontier);
    }
    // utils::g_log << "done! [t=" << utils::g_timer << "]" << std::endl;
}


}
