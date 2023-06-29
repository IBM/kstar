#ifndef STRUCTURAL_SYMMETRIES_GRAPH_CREATOR_H
#define STRUCTURAL_SYMMETRIES_GRAPH_CREATOR_H

#include <vector>

namespace bliss {
    class Digraph;
}
struct DotGraph;
class FactProxy;
class EffectsProxy;
class Group;
class OperatorProxy;
class TaskProxy;

/**
 * This class will create a bliss graph which will be used to find the
 * automorphism groups
 */

class GraphCreator  {
    void create_bliss_directed_graph(
        const TaskProxy &task_proxy,
        const bool stabilize_initial_state,
        const bool stabilize_goal,
        const bool use_color_for_stabilizing_goal,
        const bool dump_symmetry_graph,
        Group *group,
        bliss::Digraph &bliss_graph) const;
    void add_operator_directed_graph(
        const bool dump_symmetry_graph,
        Group *group,
        bliss::Digraph &bliss_graph,
        DotGraph &dot_graph,
        const OperatorProxy &op,
        int op_vertex) const;
    bool effect_can_be_overwritten(
        int effect_id,
        const EffectsProxy &effects) const;
    bool is_fact_none_of_those(FactProxy fact) const;
public:
    GraphCreator() = default;
    ~GraphCreator() = default;
    bool compute_symmetries(
        const TaskProxy &task_proxy,
        const bool stabilize_initial_state,
        const bool stabilize_goal,
        const bool use_color_for_stabilizing_goal,
        const int time_bound,
        const bool dump_symmetry_graph,
        Group *group);
};

#endif
