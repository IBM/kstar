#include "top_k_eager_search.h"
#include "../search_engines/search_common.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../structural_symmetries/group.h"

using namespace std;

namespace plugin_kstar {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {
    parser.document_synopsis("K* search (eager)", "");
    parser.document_note(
        "lazy_evaluator",
        "When a state s is taken out of the open list, the lazy evaluator h "
        "re-evaluates s. If h(s) changes (for example because h is path-dependent), "
        "s is not expanded, but instead reinserted into the open list. "
        "This option is currently only present for the A* algorithm.");    
    parser.document_note(
        "Equivalent statements using general eager search",
        "\n```\n--search kstar(evaluator)\n```\n"
        "is equivalent to\n"
        "```\n--evaluator h=evaluator\n"
        "--search kstartiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
        "               reopen_closed=true, f_eval=sum([g(), h]))\n"
        "```\n", true);
    parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for h-value");
    parser.add_option<shared_ptr<Evaluator>>(
        "lazy_evaluator",
        "An evaluator that re-evaluates a state before it is expanded.",
        OptionParser::NONE);

    parser.add_option<int>("k", "number of plans, default -1", "-1");
    parser.add_option<double>("q", "quality bound if it was 0 then we don't check quality, default 0.0", "0.0");
    parser.add_option<int>("openlist_inc_percent_lb", "astar expand at least this amount, default 1 percent", "1");
    parser.add_option<int>("openlist_inc_percent_ub", "astar expand at most this amount, default 5 percent", "5");
    parser.add_option<bool>("switch_on_goal", "switch to eppstein when astar reached a goal", "false");
    parser.add_option<bool>("restart_eppstein", "extract plans more and restart eppstein", "true");
    parser.add_option<bool>("dump_plans", "dump intermediate plan files", "true");
    parser.add_option<int>("report_period", "report number of plans found so far in sec", "540");
    parser.add_option<bool>("find_unordered_plans", "find unordered plans by skipping reordered plans", "false");
    parser.add_option<bool>("dump_plan_files", "dump plan files", "true");
    parser.add_option<bool>("allow_greedy_k_plans_selection", "Allows returning any k out of the top-q plans", "false");
    parser.add_option<string>("json_file_to_dump",
        "A path to the json file to use for dumping",
        OptionParser::NONE);
    parser.add_option<string>("preserve_orders_actions_regex",
        "A regex expression for specifying actions whose orders are not to be ignored",
        OptionParser::NONE);
        
    parser.add_option<shared_ptr<Group>>(
        "symmetries",
        "symmetries object to compute structural symmetries for pruning",
        OptionParser::NONE);
    kstar::add_options_to_parser(parser);
    Options opts = parser.parse();

    shared_ptr<kstar::TopKEagerSearch> engine;
    if (!parser.dry_run()) {
        if (opts.contains("symmetries")) {
            shared_ptr<Group> group = opts.get<shared_ptr<Group>>("symmetries");
            if (group->get_search_symmetries() == SearchSymmetries::NONE) {
                cerr << "Symmetries option passed to eager search, but no "
                     << "search symmetries should be used." << endl;
                utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
            }
        }
        auto temp = search_common::create_astar_open_list_factory_and_f_eval(opts);
        opts.set("open", temp.first);
        opts.set("f_eval", temp.second);        
        opts.set("reopen_closed", true);
        vector<shared_ptr<Evaluator>> preferred_list;
        opts.set("preferred", preferred_list);
        engine = make_shared<kstar::TopKEagerSearch>(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("kstar", _parse);
}
