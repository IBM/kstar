#include "plan_selector.h"
#include "../option_parser.h"

#include "../utils/logging.h"
#include <string>
#include <fstream>
#include <iostream>

namespace kstar {


PlanSelector::PlanSelector(const Options &opts, const TaskProxy &task_proxy) : 
            num_preserved(0),
            task_proxy(task_proxy), 
            keep_plans_unordered(opts.get<bool>("find_unordered_plans", false)),
            dump_plans(opts.get<bool>("dump_plans", true)),
            dump_plan_files(opts.get<bool>("dump_plan_files", true)),
            dump_json(opts.contains("json_file_to_dump")),
            use_regex(opts.contains("preserve_orders_actions_regex")),
            decoded_plans(utils::make_unique_ptr<std::vector<Plan>>())
            {
    utils::g_log << "Dumping plans to disk: " << (dump_plans ? "1" : "0") << std::endl;
    if (dump_json) {
        json_filename = opts.get<std::string>("json_file_to_dump");
        utils::g_log << "Dumping plans to a single json file " << json_filename << std::endl;
    }
    if (use_regex) {
        std::string action_name_regex_expression = opts.get<std::string>("preserve_orders_actions_regex");
        set_regex(action_name_regex_expression);
        utils::g_log << "Using regex " << action_name_regex_expression << " to specify action names to preserve orders of"  << std::endl;
    }
}

int PlanSelector::add_plan_if_necessary(const Plan& plan) {
    PlanCanonical canonical(plan, keep_plans_unordered, to_preserve);
    if (plans_sets.insert(canonical).second) {
        decoded_plans->push_back(plan);
        return 1;
    }
    return 0;
}

void PlanSelector::set_regex(std::string re) {
    OperatorsProxy operators = task_proxy.get_operators();
    to_preserve.assign(operators.size(), false);
    num_preserved = 0;
    for (OperatorProxy op : operators) {      
        if (std::regex_match (op.get_name(), std::regex(re) )) {
            to_preserve[op.get_id()] = true;
            num_preserved++;
        }
    }
}

bool PlanSelector::is_ordering_preserved(OperatorID id) const {
    if (keep_plans_unordered)
        return false;
    if (!use_regex) 
        return true;

    return to_preserve[id.get_index()];
}


void PlanSelector::save_plans(PlanManager& plan_manager) const {
    std::ofstream os(json_filename.c_str());
    bool first_dumped = false;
    if (dump_json) {
        // Writing plans to JSON
        os << "{ \"plans\" : [" << std::endl;
    }
    for (auto it = decoded_plans->begin(); it != decoded_plans->end(); ++it)
    {
        Plan p {*it};
        p.pop_back();
        if (dump_plan_files)
            plan_manager.save_plan(p, this->task_proxy, true);

        if (dump_json) {
            if (first_dumped) {
                os << "," << std::endl;
            }
            first_dumped = true;
            plan_manager.write_plan_json(p, os, task_proxy);
        }
    }
    if (dump_json) {
        os << "]}" << std::endl;
    }
}


}
