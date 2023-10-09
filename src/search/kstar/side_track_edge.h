#ifndef KSTAR_SIDE_TRACK_EDGE_H
#define KSTAR_SIDE_TRACK_EDGE_H

#include "../state_id.h"
#include "../operator_id.h"
#include "../utils/hash.h"
#include "../state_registry.h"
#include "../search_space.h"

#include <fstream>
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>


namespace kstar {

struct SideTrackEdge {
    StateID from;
    StateID to;
    OperatorID op;
    int delta;
    int g_from;
    int g_to;
    int cost_op;

    StateRegistry* state_registry;

    SideTrackEdge(const SideTrackEdge& other) 
        : from(other.from), to(other.to), op(other.op), delta(other.delta), 
        g_from(other.g_from), g_to(other.g_to), cost_op(other.cost_op),
        state_registry(other.state_registry) {
    }

    SideTrackEdge(SideTrackEdge& other) 
        : from(other.from), to(other.to), op(other.op), delta(other.delta), 
        g_from(other.g_from), g_to(other.g_to), cost_op(other.cost_op),
        state_registry(other.state_registry) {
    }
    
    SideTrackEdge(StateID from, StateID to, OperatorID op)
        : from(from), to(to), op(op), delta(0), 
        g_from(0), g_to(0), cost_op(0),
        state_registry(nullptr) {   
    }

    SideTrackEdge(StateID from, StateID to, OperatorID op, StateRegistry* state_registry)
        : from(from), to(to), op(op), delta(0), 
        g_from(0), g_to(0), cost_op(0),
        state_registry(state_registry) {    
    }

    ~SideTrackEdge() {
    }

    StateID get_from() const { return this->from; }

    int get_g_from() const { return this->g_from; }

    StateID get_to() const { return this->to; }

    int get_g_to() const { return this->g_to; }

    OperatorID get_op() const { return this->op; }

    int get_op_cost() const { return this->cost_op; }

    void set_state_registry(StateRegistry* state_registry) { 
        this->state_registry = state_registry;
    }

    SideTrackEdge operator=(const SideTrackEdge& other) {        
        this->from = other.from;
        this->to = other.to;
        this->op = other.op;
        this->delta = other.delta;
        this->g_from = other.g_from;
        this->g_to = other.g_to;
        this->state_registry = other.state_registry;
        return *this;
    }

    bool operator<(const SideTrackEdge& other) const {
        if (this->delta != other.delta)
            return this->delta < other.delta;
        else if (this->g_from != other.g_from)
            return this->g_from < other.g_from;
        else
            return this->from < other.from;
    }
    
    bool operator<=(const SideTrackEdge& other) const {
        return this->delta <= other.delta;
    }

    bool operator==(const SideTrackEdge& other) const {
        return (this->from == other.from) && (this->to == other.to) && (this->op == other.op);
    }

    bool operator!=(const SideTrackEdge& other) const {
        return (this->from != other.from) || (this->to != other.to) || (this->op != other.op);
    }

    std::size_t hash() const {
        return utils::get_hash(*this);
    }

    void update_delta(int delta) {
        this->delta = delta;
    }

    void update_g_from(int g_from) {
        this->g_from = g_from;
    }   

    void update_g_to(int g_to) {
        this->g_to = g_to;
    }       

    void update_cost_op(int cost_op) {
        this->cost_op = cost_op;
    }

    void compute_delta() {
        this->delta = this->g_from + this->cost_op - this->g_to;
        assert (this->delta >= 0);
    }

    int precompute_delta() {
        return this->g_from + this->cost_op - this->g_to;
    }

    int get_delta() {
        return this->delta;
    }

    void write(std::ofstream &file, const TaskProxy &task_proxy) const {
        file << "    s" << from << " -> s" << to << " [label=\""
             << task_proxy.get_operators()[op].get_name() << "\"];" << std::endl;
    }
};


inline std::ostream &operator<<(std::ostream &os, SideTrackEdge& ste) {
    os << "ste(" << ste.get_from() << ", " << ste.get_op() << ", " << ste.get_to() << ")";
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const SideTrackEdge& ste) {
    os << "ste(" << ste.get_from() << ", " << ste.get_op() << ", " << ste.get_to() << ")";
    return os;
}


struct SideTrackEdgeHasher {
    std::size_t operator() (const SideTrackEdge& ste) const {
        return ste.hash();
    }
};


struct SideTrackEdgeHandle{
    std::shared_ptr<SideTrackEdge> ste_ptr;
    SideTrackEdgeHandle(const std::shared_ptr<SideTrackEdge>& ptr): ste_ptr(ptr) {}

    static SideTrackEdgeHandle create(const SideTrackEdge& ste) {
        return std::make_shared<SideTrackEdge>(ste);
    }

    bool operator< (const SideTrackEdgeHandle& other) const {
        return *ste_ptr < *other.ste_ptr;
    }

    bool operator== (const SideTrackEdgeHandle& other) const {
        return *ste_ptr == *other.ste_ptr;
    }

    bool operator<= (const SideTrackEdgeHandle& other) const {
        return *ste_ptr <= *other.ste_ptr;
    }

    bool operator!= (const SideTrackEdgeHandle& other) const {
        return *ste_ptr != *other.ste_ptr;
    }

    std::size_t hash() const {
        return utils::get_hash(*this);
    }

};


inline std::ostream &operator<<(std::ostream &os, SideTrackEdgeHandle& ste) {
    os << "ste(" << ste.ste_ptr->get_from() << ", " << ste.ste_ptr->get_op() 
       << ", " << ste.ste_ptr->get_to() << ", delta=" << ste.ste_ptr->get_delta() << ")";
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const SideTrackEdgeHandle& ste) {
    os << "ste(" << ste.ste_ptr->get_from() << ", " << ste.ste_ptr->get_op() 
       << ", " << ste.ste_ptr->get_to() << ", delta=" << ste.ste_ptr->get_delta() << ")";       
    return os;
}

struct SideTrackEdgeHandleHasher {
    std::size_t operator() (const SideTrackEdgeHandle& ste_handle) const {
        return ste_handle.hash();
    }
};
}

namespace utils {
inline void feed(HashState &hash_state, const kstar::SideTrackEdge &ste) {
    State s_from = ste.state_registry->lookup_state(ste.from);      
    State s_to = ste.state_registry->lookup_state(ste.to);
    s_from.unpack();
    s_to.unpack();
    feed(hash_state, s_from);
    feed(hash_state, s_to);
    feed(hash_state, ste.op);
}

inline void feed(HashState &hash_state, const kstar::SideTrackEdgeHandle &ste_handle) {
    kstar::SideTrackEdge ste = *ste_handle.ste_ptr;
    feed(hash_state, ste);
}

}

#endif