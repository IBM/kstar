#ifndef KSTAR_TOP_K_EAGER_SEARCH_H
#define KSTAR_TOP_K_EAGER_SEARCH_H

#include "path_graph.h"

#include "../open_list.h"
#include "../open_list_factory.h"
#include "../search_engine.h"
#include "../per_state_information.h"
#include "../evaluation_result.h"

#include "../utils/countdown_timer.h"
#include "../utils/timer.h"

#include "plan_selector.h"

#include <memory>
#include <vector>
#include <unordered_map>
#include <queue>


class Evaluator;
class Group;
class PruningMethod;

namespace options {
class OptionParser;
class Options;
}

namespace kstar {


class TopKEagerSearch : public SearchEngine {
    bool decode_plans_upfront = false;  // if true, use extender during search to decode plans
    bool ignore_quality = true;         // if target_q is less than 1, target_k is the only criteria
    bool ignore_k = false;              // if target_k is less than 1, target_q is the only criteria
    const bool find_unordered_plans;
    const bool dump_plans;
    int report_period;
    const bool reopen_closed_nodes;
    std::shared_ptr<Group> group;
    int target_k;                    // number of k plans
    double target_q;                 // quality bounds; extract k plans within bound q; stop any of the k or q hits targets
    bool allow_greedy_k_plans_selection;                // Allow choosing any k out of top-q plans
    int target_cost_bound = EvaluationResult::INFTY;
    int openlist_inc_percent_lb;
    int openlist_inc_percent_ub;
    bool switch_on_goal;
    bool restart_eppstein;
    int outer_step_iter = 0;
    int num_astar_calls = 0;
    int num_eppstein_calls = 0;
    int number_of_plans = 0;
    int previous_number_of_plans = 0;   // track the changes in the number of plans per outer iteration
    int astar_number_of_plans = 1;      // with OSS this can be greater than 1
    int eppstein_thr = -1;
    int previous_report_sec = 0;
    int step_astar_iter_after_reopen = 0;

    bool first_goal_reached = false;
    bool goal_node_generated = false;
    bool reopen_occurred = false;
    std::unique_ptr<utils::CountdownTimer> timer;
    utils::Timer astar_search_timer;
    utils::Timer eppstein_search_timer;

    // A*
    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_evaluator;
    /*
      Note: orbit space search and duplicate pruning with dks does not work
      with preferred operators and multi path search.
    */
    bool use_oss() const;
    bool use_dks() const;
    std::shared_ptr<PlanSelector> plan_selector;

    std::vector<Evaluator *> path_dependent_evaluators;
    std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
    std::shared_ptr<Evaluator> lazy_evaluator;
    std::shared_ptr<PruningMethod> pruning_method;

    // Dumping plans
    bool dump_plan_files;
    bool dump_json;
    std::string json_filename;

    bool use_regex;
    std::string action_name_regex_expression;

    // EA   
    std::unique_ptr<std::priority_queue<PathGraphNode>> open_list_eppstein;
    std::unique_ptr<std::vector<PathGraphNode>> solution_path_nodes;
    std::unique_ptr<std::vector<Plan>> decoded_plans;
    std::unique_ptr<std::vector<Plan>> astar_decoded_plans;
    PerStateInformation<HinList> HinLists;
    PerStateInformation<HtreeList> HtreeLists;
    
    std::unique_ptr<PathGraphNode> goal_root;
    StateID initial_state_id = StateID::no_state; 
    StateID goal_state_id = StateID::no_state; 
    int min_f_open_list = 0;
    int optimal_cost = EvaluationResult::INFTY;
    
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(EvaluationContext &eval_context);
    void reward_progress();

    int get_astar_head_value();
    void initialize_astar();
    void initialize_eppstein();
    void rebuild_eppstein();

    SearchStatus step_astar();
    SearchStatus step_eppstein();
    void build_htree_list(StateID sid);
    void generate_eppstein_children(PathGraphNode* pn, std::vector<PathGraphNode>& children_nodes);
    void decode_plan_from_path_graph_node(PathGraphNode* pn, Plan& plan, std::vector<StateID>& decoded_states);
    SideTrackEdgeHandle get_ste_from_path_graph_node(PathGraphNode* pn);
    void report_intermediate_plans();
    int add_plan_if_necessary(const Plan& reference_plan);
    Plan decode_actual_plan(PathGraphNode* pn);
    
    // Setting up the plan extender when we know whether reordering is needed
    void setup_plan_selector();
    
protected:
    virtual void initialize() override;             // initialize search
    virtual SearchStatus step() override;           // 1 step alternation

public:
    explicit TopKEagerSearch(const options::Options &opts);
    virtual ~TopKEagerSearch() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;

    virtual void search() override;

    virtual void save_plan_if_necessary() override;
    
};


extern void add_options_to_parser(options::OptionParser &parser);
}

#endif
