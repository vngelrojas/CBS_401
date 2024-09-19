//
// Created by YIMIN TANG on 3/20/23.
//

#ifndef CBS_REMAKE_CBSNODE_HPP
#define CBS_REMAKE_CBSNODE_HPP

#include "CBS_parallel.hpp"
#include "../dynamic_hungarian_assignment.hpp"

class CBSNode{


public:

    CBSNode();
    explicit CBSNode(shared_ptr<CBSNode> curnode);
    explicit CBSNode(shared_ptr<CBSNode> curnode, int idx);
    void create_cost_matrix(CBS* pInstance);
    bool get_first_conflict(Conflict& conflict);
    bool update_cost_matrix(CBS* pInstance, int agent_id);
    int get_all_conflict_number();
    int cost, idx, conflict_num;
    vector<shared_ptr<Path > > cost_matrix;
    vector<shared_ptr<Constraints > > constraint_sets;
};

struct CBSNodePtrCmp{
    bool operator()(const shared_ptr<CBSNode>& a, const shared_ptr<CBSNode>& b) const {
        if (a->cost == b->cost && a->conflict_num == b->conflict_num) return a->idx < b->idx;
        if (a->cost == b->cost) return a->conflict_num > b->conflict_num;
        return a->cost > b->cost;
    }
};



#endif //CBS_REMAKE_CBSNODE_HPP
