#include "top_k_eager_search.h"
#include "path_graph.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../open_list_factory.h"
#include "../option_parser.h"
#include "../pruning_method.h"

#include "../algorithms/ordered_set.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../tasks/root_task.h"

#include "../utils/logging.h"
#include "../structural_symmetries/group.h"
#include "../structural_symmetries/operator_permutation.h"
#include "../utils/countdown_timer.h"
#include "../utils/timer.h"
#include "../utils/memory.h"
#include "../utils/collections.h"

#include <cassert>
#include <cstdlib>
#include <vector>
#include <stack>
#include <memory>
#include <optional.hh>
#include <set>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

namespace kstar
{
    TopKEagerSearch::TopKEagerSearch(const Options &opts)
        : SearchEngine(opts),
          extend_plans_with_symm(opts.get<bool>("extend_plans_with_symm", false)),
          extend_plans_with_reordering(opts.get<bool>("extend_plans_with_reordering", false)),
          find_unordered_plans(opts.get<bool>("find_unordered_plans", false)),
          dump_plans(opts.get<bool>("dump_plans", true)),
          report_period(opts.get<int>("report_period", 540)),
          reopen_closed_nodes(opts.get<bool>("reopen_closed", true)),
          target_k(opts.get<int>("k", -1)),
          target_q(opts.get<double>("q", 0.0)),
          openlist_inc_percent_lb(opts.get<int>("openlist_inc_percent_lb", 1)),
          openlist_inc_percent_ub(opts.get<int>("openlist_inc_percent_ub", 5)),
          switch_on_goal(opts.get<bool>("switch_on_goal", false)),
          restart_eppstein(opts.get<bool>("restart_eppstein", true)),
          open_list(opts.get<shared_ptr<OpenListFactory>>("open")->create_state_open_list()),
          f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
          preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
          lazy_evaluator(opts.get<shared_ptr<Evaluator>>("lazy_evaluator", nullptr)),
          pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")),
          dump_plan_files(opts.get<bool>("dump_plan_files", true)),
          dump_json(opts.contains("json_file_to_dump"))
    {
	    if (opts.contains("symmetries")) {
            group = opts.get<shared_ptr<Group>>("symmetries");
            if (group && !group->is_initialized()) {
                utils::g_log << "Initializing symmetries (eager search)" << endl;
                group->compute_symmetries(task_proxy);
            }

            if (use_dks()) {
                utils::g_log << "Setting group in registry for DKS search" << endl;
                state_registry.set_group(group);
            }
            // use plan_extender only if symmetry activated and option was true
            this->extend_plans_with_symm = this->extend_plans_with_symm && (use_dks() || use_oss());
	    } else {
    		group = nullptr;
            extend_plans_with_symm = false;
	    }
        this->decode_plans_upfront = this->extend_plans_with_symm || this->extend_plans_with_reordering || this->find_unordered_plans;

        if (lazy_evaluator && !lazy_evaluator->does_cache_estimates())
        {
            cerr << "lazy_evaluator must cache its estimates" << endl;
            utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
        }
        timer = utils::make_unique_ptr<utils::CountdownTimer>(max_time);
        open_list_eppstein = utils::make_unique_ptr<std::priority_queue<PathGraphNode>>();
        solution_path_nodes = utils::make_unique_ptr<std::vector<PathGraphNode>>();
        decoded_plans = utils::make_unique_ptr<std::vector<Plan>>();
        astar_decoded_plans = utils::make_unique_ptr<std::vector<Plan>>();
        goal_root = nullptr;
        this->plan_manager.set_plan_dirname("found_plans");

        ignore_quality = target_q < 1.0;
        ignore_k = target_k < 1;
        if (ignore_quality && ignore_k) {
            cerr << "At least one of q>=1.0 or k>=1 must be specified" << endl;
            utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
        }
        if (this->ignore_k)
            target_k = EvaluationResult::INFTY;

        utils::g_log << "Termination criteria with top_k used=" << !ignore_k << std::endl;
        utils::g_log << "Termination criteria with top_q used=" << !ignore_quality << std::endl;
        if (log.is_at_least_debug()) {
            utils::g_log << "target_k=" << this->target_k << std::endl;
            utils::g_log << "target_q=" << this->target_q << std::endl;
            utils::g_log << "target_cost_bound=" << this->target_cost_bound << std::endl;
        }
        utils::g_log << "Dumping plans to disk: " << (dump_plans ? "1" : "0") << endl;
        if (dump_json) {
            json_filename = opts.get<string>("json_file_to_dump");
            utils::g_log << "Dumping plans to a single json file " << json_filename << endl;
        }
    }

    void TopKEagerSearch::setup_plan_extender() {
        if (extend_plans_with_symm) {
            if (extend_plans_with_reordering) {
                plan_extender = make_shared<PlanExtenderBoth>(task_proxy, find_unordered_plans);
                utils::g_log << "Extending plans with both symmetry and reordering" << endl;
            } else {
                plan_extender = make_shared<PlanExtenderSymmetries>(task_proxy, find_unordered_plans);
                utils::g_log << "Extending plans with symmetry" << endl;
            }
        } else {
            if (extend_plans_with_reordering) {
                plan_extender = make_shared<PlanExtenderReordering>(task_proxy, find_unordered_plans);
                utils::g_log << "Extending plans with reordering" << endl;
            } else {
                plan_extender = make_shared<PlanExtender>(task_proxy, find_unordered_plans);
            }
        }
    }

    bool TopKEagerSearch::use_oss() const {
        return group && group->has_symmetries() && group->get_search_symmetries() == SearchSymmetries::OSS;
    }

    bool TopKEagerSearch::use_dks() const {
        return group && group->has_symmetries() && group->get_search_symmetries() == SearchSymmetries::DKS;
    }


    void TopKEagerSearch::initialize_astar()
    {
        utils::g_log << "Conducting best first search"
                     << (reopen_closed_nodes ? " with" : " without")
                     << " reopening closed nodes, (real) bound = " << bound
                     << std::endl;
        assert(this->open_list);

        set<Evaluator *> evals;
        open_list->get_path_dependent_evaluators(evals);

        /*
          Collect path-dependent evaluators that are used for preferred operators
          (in case they are not also used in the open list).
        */
        for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators)
        {
            evaluator->get_path_dependent_evaluators(evals);
        }

        /*
          Collect path-dependent evaluators that are used in the f_evaluator.
          They are usually also used in the open list and will hence already be
          included, but we want to be sure.
        */
        if (f_evaluator)
        {
            f_evaluator->get_path_dependent_evaluators(evals);
        }

        /*
          Collect path-dependent evaluators that are used in the lazy_evaluator
          (in case they are not already included).
        */
        if (lazy_evaluator)
        {
            lazy_evaluator->get_path_dependent_evaluators(evals);
        }

        path_dependent_evaluators.assign(evals.begin(), evals.end());

        State initial_state = this->state_registry.get_initial_state();
        if (use_oss()) {
            vector<int> canonical_state = group->get_canonical_representative(initial_state);
            initial_state = state_registry.register_state_buffer(canonical_state);
        }
        this->initial_state_id = initial_state.get_id();

        for (Evaluator *evaluator : path_dependent_evaluators)
        {
            evaluator->notify_initial_state(initial_state);
        }

        /*
          Note: we consider the initial state as reached by a preferred
          operator.
        */
        EvaluationContext eval_context(initial_state, 0, false, &this->statistics);

        this->statistics.inc_evaluated_states();

        if (this->open_list->is_dead_end(eval_context))
        {
            utils::g_log << "Initial state is a dead end." << endl;
        }
        else
        {
            if (this->search_progress.check_progress(eval_context))
                this->statistics.print_checkpoint_line(0);
            start_f_value_statistics(eval_context);
            SearchNode node = this->search_space.get_node(initial_state);
            node.open_initial();

            this->open_list->insert(eval_context, initial_state.get_id());
        }
        print_initial_evaluator_values(eval_context, log);

        pruning_method->initialize(task);
    }

    void TopKEagerSearch::initialize()
    {
        utils::g_log << "initialize::top-" << target_k << " search" << std::endl;
        this->eppstein_search_timer.reset();
        this->astar_search_timer.reset();
        this->astar_search_timer.resume();
        this->initialize_astar();
        this->astar_search_timer.stop();
    }

    void TopKEagerSearch::search()
    {
        initialize();
        SearchStatus status = IN_PROGRESS;
        this->outer_step_iter = 0;
        while (status == IN_PROGRESS)
        {
            this->outer_step_iter++;
            status = step();
            if (timer->is_expired())
            {
                utils::g_log << "search::time limit" << std::endl;
                utils::g_log << "search::normal_termination=" << 0 << std::endl;
                status = TIMEOUT;
            }            
        }
        utils::g_log << "search::total_step_iter=" << this->outer_step_iter << std::endl;
        utils::g_log << "search::total_num_astar_calls=" << this->num_astar_calls << std::endl;
        utils::g_log << "search::total_num_eppstein_calls=" << this->num_eppstein_calls << std::endl;
        utils::g_log << "search::total astar time=" << this->astar_search_timer() << std::endl;
        utils::g_log << "search::total eppstein time=" << this->eppstein_search_timer() << std::endl;
        utils::g_log << "Actual search time: " << timer->get_elapsed_time() << std::endl;
        utils::g_log << "Found plans: " << this->number_of_plans << std::endl;
        // dump_search_space();
    }

    SearchStatus TopKEagerSearch::step()
    {
        this->reopen_occurred = false;
        this->goal_node_generated = false;            

        if (!this->open_list->empty())
        {
            SearchStatus astar_status = IN_PROGRESS;
            this->min_f_open_list = get_astar_head_value();
            this->num_astar_calls++;
            int step_astar_iter = 0;
            this->step_astar_iter_after_reopen = 0;
            
            int target_steps_low = int(double(this->statistics.get_expanded()) * double(this->openlist_inc_percent_lb) / 100.0);
            int target_steps_upper = int(double(this->statistics.get_expanded()) * double(this->openlist_inc_percent_ub) / 100.0);
            target_steps_low = (target_steps_low < 1) ? 1 : target_steps_low;
            target_steps_upper = (target_steps_upper < 1) ? 1 : target_steps_upper;
            while(astar_status == IN_PROGRESS)
            {
                this->astar_search_timer.resume();
                astar_status = step_astar();
                double astar_duration = (double) this->astar_search_timer.stop();

                step_astar_iter += 1;
                this->step_astar_iter_after_reopen += 1;        // reset to 0 inside step_astar when reopen occurred

                if (!this->open_list->empty())
                    this->min_f_open_list = get_astar_head_value(); 

                if (astar_status == SOLVED)
                {
                    std::string preamble = "step[1]::first_astar::";
                    this->statistics.print_detailed_statistics_with_preamble(preamble);
                    utils::g_log << "A* search found the optimal path to " << this->goal_state_id << std::endl;
                    utils::g_log << "step[1]::first_astar::stop=" << astar_duration << std::endl;
                    utils::g_log << "Total first plan time: " << utils::g_timer << endl;
                    
                    if (!pruning_method->was_pruned()) {
                        // No pruning happened so far, disabling
                        pruning_method->disable_pruning();
                        // We don't need to extend plans with reordering
                        extend_plans_with_reordering = false;
                    } else {
                        utils::g_log << "Pruning disabled " << pruning_method->was_pruning_disabled() << std::endl;
                        utils::g_log << "Successors were pruned " << pruning_method->was_pruned() << std::endl;
                    }
                    setup_plan_extender();

                    if (this->decode_plans_upfront) {
                        utils::g_log << "A* number of plans before extending is " <<this->astar_number_of_plans << std::endl;
                        this->astar_number_of_plans = this->extend_plan(get_plan());
                        utils::g_log << "A* number of plans after extending is " <<this->astar_number_of_plans << std::endl;
                        for (auto it = this->decoded_plans->begin(); it != this->decoded_plans->end(); ++it)
                            this->astar_decoded_plans->push_back(*it);
                        assert(this->astar_decoded_plans->size() == this->decoded_plans->size());
                        assert((int) this->astar_decoded_plans->size() == this->astar_number_of_plans);
                    }
                    else {
                        this->astar_number_of_plans = 1;
                    }

                    if (!this->ignore_quality) {
                        this->target_cost_bound = (int) std::floor(this->target_q * (double) this->optimal_cost);
                    }
                    this->number_of_plans = this->astar_number_of_plans;

                    if (!this->ignore_k && this->number_of_plans >= this->target_k) {
                        utils::g_log << "step[1]::normal_termination=" << 1 << std::endl;
                        return SOLVED;
                    }
                    else {
                        astar_status = IN_PROGRESS;
                    }
                }
                else if (astar_status == FAILED)    // open list is empty
                {
                    if (!this->first_goal_reached) 
                    {
                        std::string preamble = "step[1]::first_astar::";
                        this->statistics.print_detailed_statistics_with_preamble(preamble);
                        utils::g_log << "A* search found no optimal path" << std::endl;
                        utils::g_log << "step[1]::first_astar::stop=" << astar_duration << std::endl;
                        utils::g_log << "Total first plan time: " << utils::g_timer << endl;          
                        this->number_of_plans = 0;
                        utils::g_log << "step[1]::normal_termination=" << 1 << std::endl;
                        return SOLVED;
                    }
                }
                else if (timer->is_expired())
                {
                    return TIMEOUT;
                }
                
                if (this->first_goal_reached)
                {
                    assert(!this->ignore_k || !this->ignore_quality);

                    // using quality bound and min_f exceeds the bound; finish astar
                    if (!this->ignore_quality && this->target_cost_bound < this->min_f_open_list)
                        break;

                    if (!this->ignore_k)
                    {
                        if (step_astar_iter == target_steps_upper || (this->switch_on_goal && this->goal_node_generated))
                            break;

                        if (!this->reopen_occurred)
                        {
                            if (this->eppstein_thr >= 0)    // assumed there's no negative cost plan
                            {
                                bool thr_lt_min_f;
                                if (this->restart_eppstein)
                                    thr_lt_min_f = this->eppstein_thr + this->optimal_cost <= this->min_f_open_list;
                                else
                                    thr_lt_min_f = this->eppstein_thr + this->optimal_cost < this->min_f_open_list;

                                if (thr_lt_min_f)   // ensure extracting at least one plan
                                    break;
                            }
                            else if (step_astar_iter >= target_steps_low)   // don't know eppstein_thr, so switch after expanding steps_low
                                break;
                        }
                        // reopen occurred and at least steps_low amount of nodes expanded
                        else if (this->step_astar_iter_after_reopen >= target_steps_low)        
                        {
                            if (this->eppstein_thr >= 0)
                            {
                                bool thr_lt_min_f;
                                if (this->restart_eppstein)
                                    thr_lt_min_f = this->eppstein_thr + this->optimal_cost <= this->min_f_open_list;
                                else
                                    thr_lt_min_f = this->eppstein_thr + this->optimal_cost < this->min_f_open_list;

                                if (thr_lt_min_f)   // ensure extracting at least one plan
                                    break;
                            }
                            else    // don't know eppstein_thr, so switch after expanding steps_low 
                                break;                            
                        }
                    }
                }       // break conditions after reaching the first goal
            }           // while astar status
        }               // astar steps

        if (this->reopen_occurred)
        {
            this->eppstein_search_timer.resume();
            rebuild_eppstein();
            this->eppstein_search_timer.stop();
        }
        
        this->eppstein_search_timer.resume();
        initialize_eppstein();
        this->eppstein_search_timer.stop();

        if (!this->open_list_eppstein->empty())
        {
            SearchStatus eppstein_status = IN_PROGRESS;
            this->num_eppstein_calls++;
            while (eppstein_status == IN_PROGRESS)
            {
                this->eppstein_search_timer.resume();
                eppstein_status = step_eppstein();
                this->eppstein_search_timer.stop();

                if (eppstein_status == SOLVED)
                {
                    utils::g_log << "step[" << this->outer_step_iter << "]::"
                                 << "step_eppstein::found " << this->number_of_plans << " plans" << std::endl;
                    utils::g_log << "step[" << this->outer_step_iter << "]::"
                                 << "step_eppstein::normal_termination=" << 1 << std::endl;
                    return SOLVED;
                }
                else if (timer->is_expired())
                    return TIMEOUT;
            }
        }

        if (this->open_list_eppstein->empty())
        {
            if (this->open_list->empty())
            {
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                             << "termination due to both queues are empty" << std::endl;
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                             << "found " << this->number_of_plans << " plans" << std::endl;            
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                             << "normal_termination=" << 1 << std::endl;
                return SOLVED;
            }
            if (!this->ignore_quality && this->target_cost_bound < this->min_f_open_list)
            {
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                             << "termination due to target_cost_bound < min_f_open_list" << std::endl;
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                            << "found " << this->number_of_plans << " plans" << std::endl;            
                utils::g_log << "step[" << this->outer_step_iter << "]::"
                            << "normal_termination=" << 1 << std::endl;
                return SOLVED;                

            }
        }
        this->report_intermediate_plans();
        return IN_PROGRESS;
    }

    SearchStatus TopKEagerSearch::step_astar()
    {
        tl::optional<SearchNode> node;
        while (true)
        {
            if (open_list->empty())
            {
                utils::g_log << "Completely explored state space -- no solution!" << endl;
                return FAILED;
            }
            StateID id = open_list->remove_min();
            State s = state_registry.lookup_state(id);
            node.emplace(search_space.get_node(s));

            if (node->is_closed())
                continue;

            /*
              We can pass calculate_preferred=false here since preferred
              operators are computed when the state is expanded.
            */
            EvaluationContext eval_context(s, node->get_g(), false, &this->statistics);

            if (lazy_evaluator)
            {
                /*
                  With lazy evaluators (and only with these) we can have dead nodes
                  in the open list.
                  For example, consider a state s that is reached twice before it is expanded.
                  The first time we insert it into the open list, we compute a finite
                  heuristic value. The second time we insert it, the cached value is reused.
                  During first expansion, the heuristic value is recomputed and might become
                  infinite, for example because the reevaluation uses a stronger heuristic or
                  because the heuristic is path-dependent and we have accumulated more
                  information in the meantime. Then upon second expansion we have a dead-end
                  node which we must ignore.
                */
                if (node->is_dead_end())
                    continue;

                if (lazy_evaluator->is_estimate_cached(s))
                {
                    int old_h = lazy_evaluator->get_cached_estimate(s);
                    int new_h = eval_context.get_evaluator_value_or_infinity(lazy_evaluator.get());
                    if (open_list->is_dead_end(eval_context))
                    {
                        node->mark_as_dead_end();
                        statistics.inc_dead_ends();
                        continue;
                    }
                    if (new_h != old_h)
                    {
                        open_list->insert(eval_context, id);
                        continue;
                    }
                }
            }
            node->close();
            assert(!node->is_dead_end());
            update_f_value_statistics(eval_context);
            this->statistics.inc_expanded();
            break;
        }
        const State &s = node->get_state();
    
        this->HinLists[s].node_closed = true;

        if (!reopen_occurred) 
        {
            const SearchNodeInfo &info = this->search_space.look_up_search_node_info(s.get_id());
            HinLists[s].update_ste_delta(s.get_id(), this->state_registry, this->search_space);
            HinLists[s].create_list_from_set(info.parent_state_id, info.creating_operator);
        }

        if (!this->first_goal_reached && task_properties::is_goal_state(task_proxy, s)) 
        {
            // check for the optimal plan corresponding to k=1
            this->goal_state_id = s.get_id();
            State goal_s = state_registry.lookup_state(goal_state_id);
            tl::optional<SearchNode> goal_node;
            goal_node.emplace(search_space.get_node(goal_s));
            EvaluationContext goal_eval_context(goal_s, goal_node->get_g(), false, &this->statistics);
            this->optimal_cost = goal_eval_context.get_g_value();
            utils::g_log << "step_astar::Optimal cost:" << this->optimal_cost << endl;
            this->first_goal_reached = true;

            if (use_dks() || use_oss()) {
                // follow the same steps as path graph nodes, find state and action trace
                std::vector<StateID> surrogate_states;
                surrogate_states.push_back(this->goal_state_id);
                Plan surrogate_plan;
                this->search_space.trace_partial_plan(this->initial_state_id, this->goal_state_id, surrogate_plan, surrogate_states);
                std::reverse(surrogate_plan.begin(), surrogate_plan.end());      // tracing from goal to initial state, reverse it
                std::reverse(surrogate_states.begin(), surrogate_states.end());

                // obtained 1 actual plan from surrogate_plan
                Plan actual_plan;
                this->search_space.surrogate_trace_to_plan(surrogate_states, surrogate_plan, actual_plan, task, group);
                this->set_plan(actual_plan);
            }
            else {
                check_goal_and_set_plan(s, nullptr);
            }
            return SOLVED;
        }

        vector<OperatorID> applicable_ops;
        this->successor_generator.generate_applicable_ops(s, applicable_ops);

        /*
          TODO: When preferred operators are in use, a preferred operator will be
          considered by the preferred operator queues even when it is pruned.
        */
        pruning_method->prune_operators(s, applicable_ops);

        // This evaluates the expanded state (again) to get preferred ops
        EvaluationContext eval_context(s, node->get_g(), false, &statistics, true);
        ordered_set::OrderedSet<OperatorID> preferred_operators;
        for (const shared_ptr<Evaluator> &preferred_operator_evaluator : preferred_operator_evaluators)
        {
            collect_preferred_operators(eval_context,
                                        preferred_operator_evaluator.get(),
                                        preferred_operators);
        }

        for (OperatorID op_id : applicable_ops)
        {
            OperatorProxy op = this->task_proxy.get_operators()[op_id];
            // this bound is infinity by default, we can limit the cost using this bound
            if ((node->get_real_g() + op.get_cost()) >= this->bound)
                continue;

            if ((node->get_real_g() + op.get_cost()) > this->target_cost_bound)
                continue;

            /*
            NOTE: In orbit search tmp_registry has to survive as long as
                    succ_state is used. This could be forever, but for heuristics
                    that do not store per state information it is ok to keep it
                    only for this operator application. In regular search it is not
                    actually needed, but I don't see a way around having it there,
                    too.
            */
            StateRegistry tmp_registry(task_proxy);
            StateRegistry *successor_registry = use_oss() ? &tmp_registry : &state_registry;
            State succ_state = successor_registry->get_successor_state(s, op);
            if (use_oss()) {
                vector<int> canonical_state = group->get_canonical_representative(succ_state);
                succ_state = state_registry.register_state_buffer(canonical_state);
            }
            
            if (this->first_goal_reached && task_properties::is_goal_state(this->task_proxy, succ_state))
                this->goal_node_generated = true;
            
            this->statistics.inc_generated();
            bool is_preferred = preferred_operators.contains(op_id);

            SearchNode succ_node = this->search_space.get_node(succ_state);

            for (Evaluator *evaluator : path_dependent_evaluators)
                evaluator->notify_state_transition(s, op_id, succ_state);

            // Previously encountered dead end. no need to re-evaluate.
            if (succ_node.is_dead_end())
                continue;

            if (succ_node.is_new())
            {
                // We have not seen this state before.
                // Evaluate and create a new node.

                // Careful: succ_node.get_g() is not available here yet,
                // hence the stupid computation of succ_g.
                // Make this less fragile.
                int succ_g = node->get_g() + get_adjusted_cost(op);

                /*
                  NOTE: previous versions used the non-canocialized successor state
                  here, but this lead to problems because the EvaluationContext was
                  initialized with one state and the insertion was performed with
                  another state.
                */ 
                EvaluationContext succ_eval_context(
                    succ_state, succ_g, is_preferred, &this->statistics);
                this->statistics.inc_evaluated_states();

                if (this->open_list->is_dead_end(succ_eval_context))
                {
                    succ_node.mark_as_dead_end();
                    this->statistics.inc_dead_ends();
                    continue;
                }
                succ_node.open(*node, op, get_adjusted_cost(op));

                this->open_list->insert(succ_eval_context, succ_state.get_id());
                if (search_progress.check_progress(succ_eval_context))
                {
                    statistics.print_checkpoint_line(succ_node.get_g());
                    reward_progress();
                }
                // TODO: using extending with sym, we don't need to add all stes
                // we can skip stes with operator symmetry and recover with extender 
                // rather than generating all ste per transitions
                SideTrackEdge ste(s.get_id(), succ_state.get_id(), op_id, &this->state_registry);
                ste.update_cost_op(get_adjusted_cost(op));
                ste.update_g_from(node->get_g());
                ste.update_g_to(succ_g);
                ste.compute_delta();
                this->HinLists[succ_state].erase_ste_from_set(ste);
                this->HinLists[succ_state].insert_ste_to_set(ste);
            }
            else
            {
                if (succ_node.get_g() > node->get_g() + get_adjusted_cost(op))
                {
                    // We found a new cheapest path to an open or closed state.
                    if (reopen_closed_nodes)
                    {
                        if (succ_node.is_closed())
                        {
                            /*
                            TODO: It would be nice if we had a way to test
                            that reopening is expected behaviour, i.e., exit
                            with an error when this is something where
                            reopening should not occur (e.g. A* with a
                            consistent heuristic).
                            */
                            statistics.inc_reopened();
                            this->reopen_occurred = true;
                            this->step_astar_iter_after_reopen = 0;
                        }
                        succ_node.reopen(*node, op, get_adjusted_cost(op));

                        EvaluationContext succ_eval_context(
                            succ_state, succ_node.get_g(), is_preferred, &this->statistics);
                        /*
                            Note: our old code used to retrieve the h value from
                            the search node here. Our new code recomputes it as
                            necessary, thus avoiding the incredible ugliness of
                            the old "set_evaluator_value" approach, which also
                            did not generalize properly to settings with more
                            than one evaluator.
                            Reopening should not happen all that frequently, so
                            the performance impact of this is hopefully not that
                            large. In the medium term, we want the evaluators to
                            remember evaluator values for states themselves if
                            desired by the user, so that such recomputations
                            will just involve a look-up by the Evaluator object
                            rather than a recomputation of the evaluator value
                            from scratch.
                        */
                        this->open_list->insert(succ_eval_context, succ_state.get_id());

                        SideTrackEdge ste(s.get_id(), succ_state.get_id(), op_id, &this->state_registry);
                        ste.update_cost_op(get_adjusted_cost(op));
                        ste.update_g_from(node->get_g());
                        ste.update_g_to(node->get_g() + get_adjusted_cost(op));
                        ste.compute_delta();
                        this->HinLists[succ_state].erase_ste_from_set(ste);
                        this->HinLists[succ_state].insert_ste_to_set(ste);
                        this->HinLists[succ_state].node_closed = false;
                    }
                    else
                    {
                        assert(reopen_closed_nodes);
                        // If we do not reopen closed nodes, we just update the parent pointers.
                        // Note that this could cause an incompatibility between
                        // the g-value and the actual path that is traced back.
                        succ_node.update_parent(*node, op, get_adjusted_cost(op));                        
                    }
                }
                else 
                {
                    if (this->switch_on_goal && task_properties::is_goal_state(this->task_proxy, s))
                        this->goal_node_generated = true;

                    SideTrackEdge ste(s.get_id(), succ_state.get_id(), op_id, &this->state_registry);
                    ste.update_cost_op(get_adjusted_cost(op));
                    ste.update_g_from(node->get_g());
                    ste.update_g_to(succ_node.get_g());
                    ste.compute_delta();
                    this->HinLists[succ_state].erase_ste_from_set(ste);
                    this->HinLists[succ_state].insert_ste_to_set(ste);
                    
                    if (!this->reopen_occurred && HinLists[succ_state].node_closed)
                        this->HinLists[succ_state].push_back_ste_handle_to_sorted_list(ste);
                }
            }
        }
        return IN_PROGRESS;
    }

    void TopKEagerSearch::rebuild_eppstein()
    {
        for (auto it = this->state_registry.begin(); it != this->state_registry.end(); ++it)
        {
            StateID sid = *it;
            State s = this->state_registry.lookup_state(sid);
            this->HinLists[s].clear_list();
            this->HinLists[s].node_closed = (sid == this->initial_state_id)? true : false;
            
            tl::optional<SearchNode> node;
            node.emplace(search_space.get_node(s));
            if (node->is_closed())
            {
                this->HinLists[s].node_closed = true;
                this->HinLists[s].update_ste_delta(sid, this->state_registry, this->search_space);
                const SearchNodeInfo &info = this->search_space.look_up_search_node_info(sid);
                this->HinLists[s].create_list_from_set(info.parent_state_id, info.creating_operator);
            }
            this->HtreeLists[s].clear_list();
        }        
        this->open_list_eppstein = utils::make_unique_ptr<std::priority_queue<PathGraphNode>>();
        this->solution_path_nodes = utils::make_unique_ptr<std::vector<PathGraphNode>>();
        if (this->goal_root != nullptr) 
            this->goal_root.reset();
    }
    
    void TopKEagerSearch::build_htree_list(StateID sid)
    {
        const State& s = this->state_registry.lookup_state(sid);
        // htree_list was built during this eppstein iteration, skip re-doing the same thing
        if (this->HtreeLists[s].updated_iter == this->outer_step_iter)
            return;

        std::vector<StateID> sid_path;  // state_ids from init to current s in search tree
        this->search_space.trace_state_path(s, sid_path);
        assert(sid_path.size() > 0);
        assert(sid_path.size() >= this->HtreeLists[s].get_size());

        this->HtreeLists[s].htreelist_stable = true;
        bool erased_occured = false;
        for (StateID sid_iter : sid_path)
        {
            State s_iter = this->state_registry.lookup_state(sid_iter);
            if (this->HinLists[s_iter].get_size() > 0)
            {
                SideTrackEdgeHandle root_ste_handle {*this->HinLists[s_iter].get_first_it()};
                bool thr_lt_min_f;
                if (this->restart_eppstein)
                    thr_lt_min_f = root_ste_handle.ste_ptr->get_delta() + this->optimal_cost <= this->min_f_open_list;
                else
                    thr_lt_min_f = root_ste_handle.ste_ptr->get_delta() + this->optimal_cost < this->min_f_open_list;

                if (this->open_list->empty() || thr_lt_min_f)
                {
                    erased_occured = this->HtreeLists[s].insert_ste_handle_to_sorted_list(root_ste_handle);
                    if (erased_occured)
                        this->HtreeLists[s].htreelist_stable = false;
                }
            }
        }
        this->HtreeLists[s].updated_iter = this->outer_step_iter;
    }

    void TopKEagerSearch::initialize_eppstein()
    {
        if (this->goal_state_id == StateID::no_state)
            return;
            
        this->build_htree_list(this->goal_state_id);

        if (!this->restart_eppstein && !this->open_list_eppstein->empty())
            return;

        // purge eppstein queues with nodes generated in the previous eppstein iteration
        if (!this->open_list_eppstein->empty())
            this->open_list_eppstein = utils::make_unique_ptr<std::priority_queue<PathGraphNode>>();
        if (!this->solution_path_nodes->empty())
            this->solution_path_nodes = utils::make_unique_ptr<std::vector<PathGraphNode>>();
        if (this->goal_root != nullptr) 
            this->goal_root.reset();
        
        this->number_of_plans = this->astar_number_of_plans;                
        this->decoded_plans = utils::make_unique_ptr<std::vector<Plan>>();
        this->plan_extender->clear();       // clear unordered_set inside plan extender
        
        if (this->decode_plans_upfront)
        {
            // plan_extender does not remove dummy action!
            for (auto it = this->astar_decoded_plans->begin(); it != this->astar_decoded_plans->end(); ++it)
            {
                this->extend_plan(*it);
            }
        }

        const State& goal_state = this->state_registry.lookup_state(this->goal_state_id);
        if (this->HtreeLists[goal_state].get_size() > 0)
        {
            assert(this->goal_root == nullptr);
            std::list<SideTrackEdgeHandle>::iterator it_htreelist = this->HtreeLists[goal_state].get_first_it();
            StateID hin_sid = (*it_htreelist).ste_ptr->get_to();
            State hin_s = this->state_registry.lookup_state(hin_sid);
            assert(this->HinLists[hin_s].get_size() > 0);
            std::list<SideTrackEdgeHandle>::iterator it_hinlist = this->HinLists[hin_s].get_first_it();
            assert(*it_hinlist == *it_htreelist);   // they are the same root ste

            this->goal_root = utils::make_unique_ptr<PathGraphNode>(
                this->goal_state_id, it_htreelist, it_hinlist,
                nullptr, false, &this->state_registry);

            if (!this->restart_eppstein)
                this->goal_root->creation_time = (double) this->timer->get_elapsed_time();

            bool thr_lt_min_f;
            if (this->restart_eppstein)
                thr_lt_min_f = this->goal_root->path_value + this->optimal_cost <= this->min_f_open_list;
            else
                thr_lt_min_f = this->goal_root->path_value + this->optimal_cost < this->min_f_open_list;

            if (this->open_list->empty() || thr_lt_min_f) {
                this->open_list_eppstein->push(*this->goal_root);
            }
        }
    }        

    SearchStatus TopKEagerSearch::step_eppstein()
    {
        if (this->open_list_eppstein->empty())
            return FAILED;

        PathGraphNode *temp_ptr = new PathGraphNode(this->open_list_eppstein->top());
        std::vector<PathGraphNode> children_nodes;
        generate_eppstein_children(temp_ptr, children_nodes);
        if (!this->restart_eppstein)
        {
            for (auto &ch : children_nodes)
            {
                if (ch.path_value > this->eppstein_thr)
                    this->eppstein_thr = ch.path_value;
            }
        }
        else {
            this->eppstein_thr = temp_ptr->path_value;   
        }

        bool thr_gt_bound;
        if (!this->restart_eppstein)  {
            // TODO non-restarting case needs confirmation on this condition
            thr_gt_bound = this->optimal_cost + this->eppstein_thr >= this->min_f_open_list;
        }
        else  {
            if (this->ignore_quality) {
                // top-k; compare thr with min_f
                thr_gt_bound = this->optimal_cost + this->eppstein_thr > this->min_f_open_list;
            }   
            else {
                // top-k with quality; looser bound than min_f
                thr_gt_bound = this->optimal_cost + this->eppstein_thr > this->target_cost_bound;
            }
        }

        if (!this->open_list->empty() && thr_gt_bound)  // we cannot extract solution yet, so FAILED status and switch back to astar
            return FAILED;

        // process one valid path graph node to count plans found so far
        this->open_list_eppstein->pop();

        // push to solution_path_nodes only if the path graph node is within the cost bound
        if (temp_ptr->path_value + this->optimal_cost <= this->target_cost_bound) 
        {
            if (this->decode_plans_upfront) {
                // decode path graph node here to know the number of symmetric plans
                Plan decoded_plan = this->decode_actual_plan(temp_ptr);
                this->number_of_plans += this->extend_plan(decoded_plan);
            }
            else {
                this->solution_path_nodes->push_back(*temp_ptr);
                this->number_of_plans += 1;     // don't decode path graph node now, do it later
            }
    
            for (auto &ch : children_nodes) {
                this->open_list_eppstein->push(ch);
            }
        }
            
        if (!this->ignore_k && this->number_of_plans >= this->target_k)
            return SOLVED;

        if (!this->ignore_quality && this->target_cost_bound < this->optimal_cost + this->eppstein_thr)
            return SOLVED;

        return IN_PROGRESS;
    }

    void TopKEagerSearch::generate_eppstein_children(PathGraphNode *pn, std::vector<PathGraphNode> &children_nodes)
    {
        build_htree_list(pn->sid_htree);
        State s_tree = this->state_registry.lookup_state(pn->sid_htree);       
        SideTrackEdgeHandle ste_hanlde_in_pn = this->get_ste_from_path_graph_node(pn);
        StateID sid_hin = ste_hanlde_in_pn.ste_ptr->get_to();
        State s_hin = this->state_registry.lookup_state(sid_hin);

        // go right
        // pn points to a root_ste; generate next root_ste in HtreeLists[s_tree]
        // it_hinlist has to point begin() and it_htreelist cannot be the last one
        if (pn->it_hinlist == this->HinLists[s_hin].get_first_it() &&
            pn->it_htreelist != this->HtreeLists[s_tree].get_last_it())
        {
            auto ch_htree_it = std::next(pn->it_htreelist);
            StateID ch_sid_hin = (*ch_htree_it).ste_ptr->get_to();
            State ch_s_hin = this->state_registry.lookup_state(ch_sid_hin);
            auto ch_hin_it = this->HinLists[ch_s_hin].get_first_it();
            
            PathGraphNode ch (pn->sid_htree, ch_htree_it, ch_hin_it, 
                              pn, false, pn->state_registry);

            if (!this->restart_eppstein)
                ch.creation_time = (double) this->timer->get_elapsed_time();
            
            if (this->ignore_quality)  {
                children_nodes.push_back(ch);
            }
            else if (ch.path_value + this->optimal_cost <= this->target_cost_bound) {
                children_nodes.push_back(ch);
            }            
        }

        // go down
        // pn points to a ste in some root_ste, it is not a root_ste; generate next ste in hinlist
        if (pn->it_hinlist != this->HinLists[s_hin].get_last_it())
        {
            auto ch_hin_it = std::next(pn->it_hinlist);
            PathGraphNode ch (pn->sid_htree, 
                              pn->it_htreelist, 
                              ch_hin_it,
                              pn, false, pn->state_registry);
            if (!this->restart_eppstein)
                ch.creation_time = (double) this->timer->get_elapsed_time();

            if (this->ignore_quality) {
                children_nodes.push_back(ch);
            }
            else if (ch.path_value + this->optimal_cost <= this->target_cost_bound) {
                children_nodes.push_back(ch);
            }
        }
        // crossing arc from the current ste_hanlde_in_pn
        StateID sid_from = ste_hanlde_in_pn.ste_ptr->get_from();
        State s_from = this->state_registry.lookup_state(sid_from);
        build_htree_list(sid_from);
        if (this->HtreeLists[s_from].get_size() > 0)
        {
            auto it_htree_first = this->HtreeLists[s_from].get_first_it();
            StateID ch_sid_hin = (*it_htree_first).ste_ptr->get_to();
            State ch_s_hin = this->state_registry.lookup_state(ch_sid_hin);
            auto ch_hin_it = this->HinLists[ch_s_hin].get_first_it();

            PathGraphNode ch (sid_from, it_htree_first, ch_hin_it,
                              pn, true, pn->state_registry);
            
            if (!this->restart_eppstein)
                ch.creation_time = (double) this->timer->get_elapsed_time();

            if (this->ignore_quality) {
                children_nodes.push_back(ch);
            }
            else if (ch.path_value + this->optimal_cost <= this->target_cost_bound) {
                children_nodes.push_back(ch);
            }                
        }
    }

    void TopKEagerSearch::decode_plan_from_path_graph_node(PathGraphNode *pn, Plan &plan, vector<StateID>& decoded_states)
    {
        std::stack<SideTrackEdgeHandle> active_deviations;
        bool active = true;

        PathGraphNode *current = pn;
        while (current != nullptr && current->sid_htree != StateID::no_state)
        {
            if (active) {
                SideTrackEdgeHandle ste_handle = get_ste_from_path_graph_node(current);
                active_deviations.push(ste_handle);
            }
            active = current->by_crossing_arc;
            current = current->parent_node;
        }
        // only accumulated active deviations in a stack, the top element is the deviation closest to the goal state node
        assert(!active_deviations.empty());

        StateID prev_from = StateID::no_state;
        bool first_fragment = true;
        while (!active_deviations.empty())
        {
            SideTrackEdgeHandle ste_handle = active_deviations.top();

            StateID current_from = ste_handle.ste_ptr->get_from();
            StateID current_to = ste_handle.ste_ptr->get_to();
            OperatorID current_op = ste_handle.ste_ptr->get_op();
            
            if (first_fragment) {
                decoded_states.push_back(this->goal_state_id);
                this->search_space.trace_partial_plan(current_to, this->goal_state_id, plan, decoded_states);
                first_fragment = false;
            }
            else {
                this->search_space.trace_partial_plan(current_to, prev_from, plan, decoded_states);
            }
            
            plan.push_back(current_op);
            decoded_states.push_back(current_from);
            prev_from = current_from;
            active_deviations.pop();
        }

        if (prev_from != this->initial_state_id)
            this->search_space.trace_partial_plan(this->initial_state_id, prev_from, plan, decoded_states);

        std::reverse(plan.begin(), plan.end()); // the decoding gives us the reverse of the plan including dummy action
        std::reverse(decoded_states.begin(), decoded_states.end());
    }

    SideTrackEdgeHandle TopKEagerSearch::get_ste_from_path_graph_node(PathGraphNode* pn) {
        return *pn->it_hinlist;
    }

    int TopKEagerSearch::get_astar_head_value()
    {
        StateID id = this->open_list->remove_min();
        State s = this->state_registry.lookup_state(id);
        tl::optional<SearchNode> node;
        node.emplace(search_space.get_node(s));
        EvaluationContext eval_context(s, node->get_g(), false, &this->statistics);
        this->open_list->insert(eval_context, id);
        int f_value = eval_context.get_evaluator_value(this->f_evaluator.get());
        return f_value;
    }

    void TopKEagerSearch::print_statistics() const
    {
        this->statistics.print_detailed_statistics();
        this->search_space.print_statistics();
        this->pruning_method->print_statistics();
    }

    void TopKEagerSearch::report_intermediate_plans()
    {
        this->save_plan_if_necessary();

        int elapsed_sec = (int) this->timer->get_elapsed_time();
        if (elapsed_sec - this->previous_report_sec > this->report_period && this->number_of_plans > this->previous_number_of_plans)
        {
            utils::g_log << "step[" << this->outer_step_iter << "]::found_plans=" << this->number_of_plans << std::endl;
            this->previous_number_of_plans = this->number_of_plans;
            this->previous_report_sec = elapsed_sec;
        }
    }

    int TopKEagerSearch::extend_plan(const Plan& reference_plan)
    {
        // The reference plan is added if new
        int num_target_plans = this->target_k;
        if (!this->ignore_k) {
            num_target_plans -= (int) this->decoded_plans->size();
        }

        int count_plans = 0;
        vector<Plan> plans;
        plan_extender->extend_plan(reference_plan, plans, num_target_plans);
        for (Plan p : plans)
        {
            // p.pop_back();    // commented out removing the dummy operator when saving to file; dot it when dumping files
            this->decoded_plans->push_back(p);
            ++count_plans;
        }
        return count_plans;
    }

    Plan TopKEagerSearch::decode_actual_plan(PathGraphNode* pn)
    {
        Plan actual_plan;
        Plan surrogate_plan;
        vector<StateID> decoded_states;
        decode_plan_from_path_graph_node(pn, surrogate_plan, decoded_states);        
        if (!use_dks() && !use_oss()) {
            // In case the symmetry reduction is not used, the surrogate plan is the plan we want
            return surrogate_plan;     
        }   
        search_space.surrogate_trace_to_plan(decoded_states, surrogate_plan, actual_plan, task, group);
        return actual_plan;
    }

    void TopKEagerSearch::save_plan_if_necessary()
    {
        if ((!this->dump_plans) || this->number_of_plans == 0)
            return; 

        if (!this->decode_plans_upfront && this->number_of_plans != (int) this->decoded_plans->size())
        {
            int count_plans = 0;
            if (!this->decoded_plans->empty()) {
                this->decoded_plans = utils::make_unique_ptr<std::vector<Plan>>();    
                this->plan_extender->clear();
            }

            // process astar plan; this->decoded_plans stores astar plan
            if (found_solution())
            {
                // TODO: When we move things into the plan extender, we can change its behavior to not check for duplicates
                //       and store plans in the case when no decoding upfront was performed.
                this->decoded_plans->push_back(get_plan());
                count_plans = this->astar_number_of_plans;
            }
            
            // process kstar plans
            while (count_plans < this->target_k)
            {
                auto it = this->solution_path_nodes->begin();
                for (; it != this->solution_path_nodes->end(); ++it)
                {
                    PathGraphNode pn = *it;
                    Plan decoded_plan;
                    decoded_plan = this->decode_actual_plan(&pn);       // all cases are divided inside this method
                    // TODO: here as well, as in todo above, when we move things to the plan extender, ensure correct behavior. 
                    this->decoded_plans->push_back(decoded_plan);
                    count_plans++;
                }
                if (it == this->solution_path_nodes->end())
                    break;
            }
            if (!this->ignore_k) {
                assert ((int) this->decoded_plans->size() <= this->target_k);
            }
            assert ((int) this->decoded_plans->size() == this->number_of_plans);
            assert ((int) this->decoded_plans->size() == count_plans);
        }

        this->plan_manager.delete_plans("found_plans/done");
        this->plan_manager.move_plans("found_plans", "found_plans/done");
        this->plan_manager.set_num_previously_generated_plans(0);

        ofstream os(json_filename.c_str());
        bool first_dumped = false;
        if (dump_json) {
            // Writing plans to JSON
            os << "{ \"plans\" : [" << endl;
        }
        for (auto it = this->decoded_plans->begin(); it != this->decoded_plans->end(); ++it)
        {
            Plan p {*it};
            p.pop_back();
            if (dump_plan_files)
                this->plan_manager.save_plan(p, this->task_proxy, true);

            if (dump_json) {
                if (first_dumped) {
                    os << "," << endl;
                }
                first_dumped = true;
                plan_manager.write_plan_json(p, os, task_proxy);
            }
        }
        if (dump_json) {
            os << "]}" << endl;
        }
    }

    void TopKEagerSearch::reward_progress()
    {
        // Boost the "preferred operator" open lists somewhat whenever
        // one of the heuristics finds a state with a new best h value.
        open_list->boost_preferred();
    }

    void TopKEagerSearch::dump_search_space() const
    {
        search_space.dump(task_proxy);
    }

    void TopKEagerSearch::start_f_value_statistics(EvaluationContext &eval_context)
    {
        if (f_evaluator)
        {
            int f_value = eval_context.get_evaluator_value(f_evaluator.get());
            statistics.report_f_value_progress(f_value);
        }
    }

    /* HACK! This is very inefficient for simply looking up an h value.
       Also, if h values are not saved it would recompute h for each and every state. */
    void TopKEagerSearch::update_f_value_statistics(EvaluationContext &eval_context)
    {
        if (f_evaluator)
        {
            int f_value = eval_context.get_evaluator_value(f_evaluator.get());
            statistics.report_f_value_progress(f_value);
        }
    }

    void add_options_to_parser(OptionParser &parser)
    {
        SearchEngine::add_pruning_option(parser);
        SearchEngine::add_options_to_parser(parser);
    }

}
