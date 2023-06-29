#ifndef KSTAR_PATH_GRAPH_H
#define KSTAR_PATH_GRAPH_H


#include "side_track_edge.h"
#include "../state_id.h"
#include "../operator_id.h"
#include "../utils/hash.h"
#include "../state_registry.h"
#include "../search_space.h"
#include "../task_proxy.h"

#include <algorithm>
#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <optional.hh>


namespace kstar {

class HinList {
public:
    std::list<SideTrackEdgeHandle> ste_handle_list;
    std::unordered_set<SideTrackEdge, SideTrackEdgeHasher> ste_set;
    bool node_closed;

    HinList()
    {
        this->node_closed = false;
    }
    ~HinList() = default;    

    void clear_list() {
        this->ste_handle_list.clear();
    }
   
    void erase_ste_from_set(const SideTrackEdge& ste) {
        this->ste_set.erase(ste);
    }

    void insert_ste_to_set(const SideTrackEdge& ste) 
    {
        assert(ste.state_registry != nullptr);        
        this->ste_set.insert(ste);
    } 

    void create_list_from_set(StateID pa_in_tree, OperatorID op_in_tree) 
    {
        this->clear_list();
        assert (this->node_closed);
        for (const auto& ste: this->ste_set)
        {
            if (ste.get_from() != pa_in_tree || ste.get_op() != op_in_tree)
                this->ste_handle_list.push_back(SideTrackEdgeHandle::create(ste));
        }
        this->ste_handle_list.sort();
    }

    void push_back_ste_handle_to_sorted_list(const SideTrackEdge& ste)
    {
        SideTrackEdgeHandle ste_handle = SideTrackEdgeHandle::create(ste);
        this->insert_ste_handle_to_sorted_list(ste_handle);
    }

    void insert_ste_handle_to_sorted_list(const SideTrackEdgeHandle& ste_handle)
    {
        auto it = ste_handle_list.begin();
        while(it != ste_handle_list.end())
        {
            if (*it <= ste_handle)
                it = std::next(it);
            else
                break;
        }
        this->ste_handle_list.insert(it, ste_handle);
    }

    void update_ste_delta(StateID sid_to, StateRegistry& state_registry, SearchSpace& search_space) 
    {
        assert(sid_to != StateID::no_state);
        State s_to = state_registry.lookup_state(sid_to);
        tl::optional<SearchNode> node_to;
        node_to.emplace(search_space.get_node(s_to));
        int g_to = node_to->get_g();

        std::vector<SideTrackEdge> temp_ste_holder;
        for (auto &ste: this->ste_set) {
            temp_ste_holder.push_back(ste);
        }
        this->ste_set.clear();
        tl::optional<SearchNode> node_from;
        for (auto& ste : temp_ste_holder) {
            State s_from = state_registry.lookup_state(ste.from);    
            node_from.emplace(search_space.get_node(s_from));
            ste.g_from = node_from->get_g();
            ste.g_to = g_to;
            assert(ste.precompute_delta() >=0);
            ste.compute_delta();
            this->ste_set.insert(ste);
        }
    }

    std::size_t get_size() {
        return this->ste_handle_list.size();
    }

    std::list<SideTrackEdgeHandle>::iterator get_first_it() 
    {
        return this->ste_handle_list.begin();
    }

    std::list<SideTrackEdgeHandle>::iterator get_last_it() 
    {
        std::list<SideTrackEdgeHandle>::iterator it {this->ste_handle_list.end()};
        it = std::prev(it);
        return it;
    }

    void dump_ste_list() 
    {
        std::cout << "HinList::dump_ste_list" << std::endl;
        if (this->ste_handle_list.size() == 0)
            std::cout << "node_closed=" << this->node_closed << ", empty list" << std::endl;
        else {
            auto it = this->ste_handle_list.begin();
            while (it != this->ste_handle_list.end()) {
                std::cout << "ste=" << *it << ", delta=" << (*it).ste_ptr->get_delta() << std::endl;
                it = std::next(it);
            }
        }
        std::cout << "----\n";
    }

    void dump_ste_set() 
    {
        std::cout << "HiList::dump_ste_set" << std::endl;
        for (auto item : this->ste_set)
            std::cout << "ste=" << item << ", delta=" << item.delta << std::endl;
        
        std::cout << "----\n";

    }
};

class HtreeList {
public:
    std::list<SideTrackEdgeHandle> hinroot_handles;
    bool htreelist_stable;
    int updated_iter;

    HtreeList() : htreelist_stable(false), updated_iter(-1) { }

    ~HtreeList() = default;

    void clear_list() {
        this->hinroot_handles.clear();
        this->htreelist_stable = false;
    }

    bool insert_ste_handle_to_sorted_list(const SideTrackEdgeHandle& ste_handle)
    {
        bool erase_occurred = false;
        auto it = this->hinroot_handles.begin();
        StateID sin = ste_handle.ste_ptr->get_to();
        while (it != hinroot_handles.end())
        {
            StateID sin_it = (*it).ste_ptr->get_to();
            if (*it == ste_handle)
                return erase_occurred;      // ste *it is already stored, but ste_handle is the current root ste
            if (sin == sin_it)
                break;
            it = std::next(it);
        }
        if (it != hinroot_handles.end())
        {
            // if ste from the same Hin found, erase the old one
            this->hinroot_handles.erase(it);
            erase_occurred = true;
        }
        
        it = this->hinroot_handles.begin();
        while(it != hinroot_handles.end())
        {
            if (*it <= ste_handle)
                it = std::next(it);
            else
                break;
        }
        this->hinroot_handles.insert(it, ste_handle);
        return erase_occurred;
    }

    std::size_t get_size() {
        return this->hinroot_handles.size();
    }

    std::list<SideTrackEdgeHandle>::iterator get_first_it()
    {
        return hinroot_handles.begin();
    }

    std::list<SideTrackEdgeHandle>::iterator get_last_it()
    {
        std::list<SideTrackEdgeHandle>::iterator it {this->hinroot_handles.end()};
        it = std::prev(it);
        return it;
    }

    void dump_ste_list() {
        std::cout << "HtreeList::dump_ste_list" << std::endl;
        if (this->hinroot_handles.size() == 0)
            std::cout << "htreelist_stable=" << this->htreelist_stable << ", empty list" << std::endl;
        else 
        {
            auto it = this->hinroot_handles.begin();
            while (it != this->hinroot_handles.end())
            {
                std::cout << "std=" << *it << ", delta=" << (*it).ste_ptr->get_delta() << std::endl;
                it = std::next(it);
            }
        }
        std::cout << "stable=" << this->htreelist_stable << std::endl;
        std::cout << "----\n";
    }
};


struct PathGraphNode 
{
    StateID sid_htree = StateID::no_state;
    std::list<SideTrackEdgeHandle>::iterator it_htreelist{};
    std::list<SideTrackEdgeHandle>::iterator it_hinlist{};
    PathGraphNode* parent_node;
    bool by_crossing_arc;
    StateRegistry* state_registry;

    int ste_delta = -1;
    int edge_value = -1;
    int path_value = -1;
    double creation_time = 0.0;

    PathGraphNode(StateID sid_htree, 
                  std::list<SideTrackEdgeHandle>::iterator it_htreelist,
                  std::list<SideTrackEdgeHandle>::iterator it_hinlist,
                  PathGraphNode* parent_node, 
                  bool by_crossing_arc,
                  StateRegistry* state_registry
    ) : 
        sid_htree(sid_htree), 
        it_htreelist(it_htreelist),
        it_hinlist(it_hinlist),
        parent_node(parent_node),
        by_crossing_arc(by_crossing_arc),
        state_registry(state_registry)
    { 
        this->compute_node_value();
    }

    PathGraphNode(const PathGraphNode& other):
        sid_htree(other.sid_htree), 
        it_htreelist(other.it_htreelist), 
        it_hinlist(other.it_hinlist),
        parent_node(other.parent_node),
        by_crossing_arc(other.by_crossing_arc),
        state_registry(other.state_registry)
    { 
        ste_delta = other.ste_delta;
        edge_value = other.edge_value;
        path_value = other.path_value;
        creation_time = other.creation_time;
    }

    ~PathGraphNode() = default;

    bool operator<(const PathGraphNode& other) const {
        // flip operator order and make max to min heap
        if (this->path_value == other.path_value)
            return this->creation_time > other.creation_time;
        else
            return this->path_value > other.path_value;
    }

    bool operator==(const PathGraphNode& other) const 
    {
        return (this->sid_htree == other.sid_htree) && \
        (this->it_htreelist == other.it_htreelist) && \
        (this->it_hinlist == other.it_hinlist) && \
        (this->parent_node == other.parent_node) && \
        (this->ste_delta == other.ste_delta) && \
        (this->by_crossing_arc == other.by_crossing_arc) && \
        (this->edge_value == other.edge_value) && \
        (this->path_value == other.path_value);
    }

    bool operator!=(const PathGraphNode& other) const {
        return (*this == other);
    }

    std::size_t hash() const {
        assert(this->state_registry);
        return utils::get_hash(*this);        
    }

    void compute_node_value() {
        this-> ste_delta = (*it_hinlist).ste_ptr->get_delta();

        this->edge_value = this->ste_delta;
        if (!this->by_crossing_arc && this->parent_node != nullptr)
            this->edge_value -= this->parent_node->ste_delta;
        
        this->path_value = this->edge_value;
        if (this->parent_node != nullptr)
            this->path_value += this->parent_node->path_value;
    }
};


inline std::ostream &operator<<(std::ostream &os, PathGraphNode& pn) {
    os << "pn [htree sid=" << pn.sid_htree
       << " ste=" << *(*pn.it_hinlist).ste_ptr
       << " crossing=" <<  pn.by_crossing_arc
       << " delta=" <<  pn.ste_delta
       << " edge="  <<  pn.edge_value
       << " path="  <<  pn.path_value;
    
    if (pn.parent_node != nullptr) 
        os << " pa=[" << *(*(pn.parent_node->it_hinlist)).ste_ptr << "]]";
    else 
        os << " pa=[nullptr]]";
    return os;
}


struct PathGraphNodeHasher {
    std::size_t operator() (const PathGraphNode& node) const {
        return node.hash();
    }
};
}


namespace utils {

inline void feed(HashState &hash_state, const kstar::PathGraphNode &node) 
{
    feed(hash_state, *node.it_htreelist);
    feed(hash_state, *node.it_hinlist);
    feed(hash_state, node.by_crossing_arc);
    feed(hash_state, node.ste_delta);   
    feed(hash_state, node.edge_value);   
    feed(hash_state, node.path_value);   
}

}
#endif
