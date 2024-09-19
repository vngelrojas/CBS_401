//
// Created by YIMIN TANG on 3/20/23.
//

#ifndef CBS_REMAKE_CBSNODE_HPP
#define CBS_REMAKE_CBSNODE_HPP

#include "CBS.hpp"
#include "../dynamic_hungarian_assignment.hpp"

class CBSNode{


public:

    CBSNode();
    explicit CBSNode(shared_ptr<CBSNode> curnode);

    void create_cost_matrix(CBS* pInstance);
    bool get_first_conflict(Conflict& conflict);
    bool update_cost_matrix(CBS* pInstance, int agent_id);

    int cost;
    vector<shared_ptr<Path > > cost_matrix;
    vector<shared_ptr<Constraints > > constraint_sets;
};

struct CBSNodePtrCmp{
    bool operator()(const shared_ptr<CBSNode>& a, const shared_ptr<CBSNode>& b) const {
        return a->cost > b->cost;
    }
};



#endif //CBS_REMAKE_CBSNODE_HPP
