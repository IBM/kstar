#ifndef STRUCTURAL_SYMMETRIES_GROUP_H
#define STRUCTURAL_SYMMETRIES_GROUP_H

#include <memory>
#include <unordered_map>
#include <vector>

class Permutation;
class OperatorPermutation;
class State;
class TaskProxy;

namespace options {
class Options;
}

enum class SearchSymmetries {
    NONE,
    OSS,
    DKS
};

// Permutation of bliss graph vertices.
using RawPermutation = std::vector<int>;
// Trace is a vector of pair of generator id and bool, representing whether it is regular or inverse (false)
using PermutationTrace = std::vector<std::pair<int,bool>>;

class Group {
    // Options for Bliss and the type of symmetries used
    const bool stabilize_initial_state;
    const bool stabilize_goal;
    const bool use_color_for_stabilizing_goal;
    const int time_bound;
    const bool dump_symmetry_graph;
    const SearchSymmetries search_symmetries;
    const bool dump_permutations;
    const bool write_search_generators;
    const bool write_all_generators;
    const bool keep_operator_symmetries;
    const bool keep_state_identity_operator_symmetries;

    // Group properties
    int num_vars;
    int num_operators;
    int permutation_length; // as used for search generators
    int graph_size; // including vertices for operators
    std::vector<int> dom_sum_by_var;
    std::vector<int> var_by_val;
    int num_identity_generators;

    // Group creation
    bool initialized;
    std::vector<Permutation> generators;
    std::vector<OperatorPermutation> operator_generators;
    std::vector<OperatorPermutation> operator_state_identity_generators;
    std::vector<std::unordered_map<int, int>> to_be_written_generators;
    std::vector<OperatorPermutation> operator_inverse_generators;

    // Path tracing
    void compute_permutation_trace_to_canonical_representative(const State& state, std::vector<int>&) const;
    RawPermutation compute_permutation_from_trace(const std::vector<int> &permutation_trace) const;
    RawPermutation compute_inverse_permutation(const RawPermutation &permutation) const;

    void write_generators() const;
    void add_to_be_written_generator(const unsigned int *generator);
public:
    explicit Group(const options::Options &opts);
    ~Group() = default;

    // Graph creator
    void add_to_dom_sum_by_var(int summed_dom);
    void add_to_var_by_val(int var);

    // Methods for creating the group
    void compute_symmetries(const TaskProxy &task_proxy);
    void add_raw_generator(const unsigned int *generator);
    void set_permutation_num_variables(int nvars) {
        num_vars = nvars;
    }
    int get_permutation_num_variables() const {
        return num_vars;
    }
    void set_permutation_num_operators(int nops) {
        num_operators = nops;
    }
    int get_permutation_num_operators() const {
        return num_operators;
    }
    void set_permutation_length(int length) {
        permutation_length = length;
    }
    int get_permutation_length() const {
        return permutation_length;
    }
    void set_graph_size(int length) {
        graph_size = length;
    }
    int get_var_by_index(int val) const;
    std::pair<int, int> get_var_val_by_index(const int ind) const;
    int get_index_by_var_val_pair(const int var, const int val) const;

    // Using the group
    int get_num_generators() const;
    int get_num_identity_generators() const {
        return num_identity_generators;
    }
    void dump_generators() const;
    void dump_variables_equivalence_classes() const;
    void write_generators_to_file() const;
    void statistics() const;
    bool is_stabilizing_initial_state() const {
        return stabilize_initial_state;
    }
    bool is_initialized() const {
        return initialized;
    }
    bool has_symmetries() const {
        return !generators.empty();
    }
    SearchSymmetries get_search_symmetries() const {
        return search_symmetries;
    }

    // Used for OSS
    std::vector<int> get_canonical_representative(const State &state) const;
    // Following methods: used for path tracing (OSS and DKS)
    RawPermutation new_identity_raw_permutation() const;
    RawPermutation compose_permutations(
        const RawPermutation &permutation1, const RawPermutation &permutation2) const;
    RawPermutation create_permutation_from_state_to_state(
        const State &from_state, const State &to_state, PermutationTrace& trace) const;

    // Used for plans graph
    int get_num_operator_symmetries() const;
    const Permutation &get_permutation(int index) const;
    const OperatorPermutation &get_operator_permutation(int index) const;
    int permute_operator(int op, const PermutationTrace& trace) const;
    // void create_operator_inverse_generators();

};

#endif
