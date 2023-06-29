#include "plan_manager.h"

#include "task_proxy.h"

#include "task_utils/task_properties.h"
#include "utils/logging.h"

#include <fstream>
#include <iostream>
#include <sstream>
// #include <filesystem>


using namespace std;

int calculate_plan_cost(const Plan &plan, const TaskProxy &task_proxy) {
    OperatorsProxy operators = task_proxy.get_operators();
    int plan_cost = 0;
    for (OperatorID op_id : plan) {
        plan_cost += operators[op_id].get_cost();
    }
    return plan_cost;
}

PlanManager::PlanManager()
    : plan_filename("sas_plan"),
      plans_dirname("found_plans"),
      num_previously_generated_plans(0),
      is_part_of_anytime_portfolio(false) {
}

void PlanManager::set_plan_filename(const string &plan_filename_) {
    plan_filename = plan_filename_;
}

void PlanManager::set_plan_dirname(const string &plan_dirname_) {
    plans_dirname = plan_dirname_;
}

void PlanManager::set_num_previously_generated_plans(int num_previously_generated_plans_) {
    num_previously_generated_plans = num_previously_generated_plans_;
}

void PlanManager::set_is_part_of_anytime_portfolio(bool is_part_of_anytime_portfolio_) {
    is_part_of_anytime_portfolio = is_part_of_anytime_portfolio_;
}

void PlanManager::save_plan(
    const Plan &plan, const TaskProxy &task_proxy,
    bool generates_multiple_plan_files) {
    ostringstream filename;
    filename << plans_dirname << "/";
    filename << plan_filename;
    int plan_number = num_previously_generated_plans + 1;
    if (generates_multiple_plan_files || is_part_of_anytime_portfolio) {
        filename << "." << plan_number;
    } else {
        assert(plan_number == 1);
    }
    ofstream outfile(filename.str());
    if (outfile.rdstate() & ofstream::failbit) {
        cerr << "Failed to open plan file: " << filename.str() << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
    OperatorsProxy operators = task_proxy.get_operators();
    for (OperatorID op_id : plan) {
        // cout << operators[op_id].get_name() << " (" << operators[op_id].get_cost() << ")" << endl;
        outfile << "(" << operators[op_id].get_name() << ")" << endl;
    }
    int plan_cost = calculate_plan_cost(plan, task_proxy);
    bool is_unit_cost = task_properties::is_unit_cost(task_proxy);
    outfile << "; cost = " << plan_cost << " ("
            << (is_unit_cost ? "unit cost" : "general cost") << ")" << endl;
    outfile.close();
    // utils::g_log << "Plan length: " << plan.size() << " step(s)." << endl;
    // utils::g_log << "Plan cost: " << plan_cost << endl;
    ++num_previously_generated_plans;
}

void PlanManager::delete_plans(const std::string& del_dirname)
{
    string filename {del_dirname};
    filename = filename + "/" + plan_filename + ".";
    for (int plan_number=1; plan_number <= num_previously_generated_plans; ++plan_number)
    {
        string planfile = filename + std::to_string(plan_number);
        std::remove(planfile.c_str());
    }
}

void PlanManager::move_plans(const std::string& source_dirname, const std::string& dest_dirname)
{
    string source_filename {source_dirname};
    string dest_filename {dest_dirname};
    source_filename = source_filename + "/" + plan_filename + ".";
    dest_filename = dest_filename + "/" + plan_filename + ".";
    
    for (int plan_number=1; plan_number <= num_previously_generated_plans; ++plan_number)
    {
        string source_planfile = source_filename + std::to_string(plan_number);
        string dest_planfile = dest_filename + std::to_string(plan_number);
        std::rename(source_planfile.c_str(), dest_planfile.c_str());
    }
}

void PlanManager::write_plan_json(Plan plan, std::ostream& os, const TaskProxy &task_proxy) const {
    int plan_cost = calculate_plan_cost(plan, task_proxy);
    os << "{ ";
    os << "\"cost\" : " << plan_cost << "," << endl; 
    os << "\"actions\" : [" << endl;
    OperatorsProxy operators = task_proxy.get_operators();
    if (plan.size() > 0) {
        os << "\""  << operators[plan[0]].get_name() << "\"";
        for (size_t i = 1; i < plan.size(); ++i) {
            os << ", \"" << operators[plan[i]].get_name() << "\"";
        }
    }
    os << "]";
    os << "}" << endl;
}
