//
// Created by YIMIN TANG on 3/20/23.
//

#ifndef ECBS_REMAKE_ECBSNODE_HPP
#define ECBS_REMAKE_ECBSNODE_HPP

#include "../dynamic_hungarian_assignment.hpp"

class ECBS;

class ECBSNode{


public:

    ECBSNode();
    explicit ECBSNode(shared_ptr<ECBSNode> curnode);

    void create_cost_matrix(ECBS* pInstance);
    bool update_cost_matrix(ECBS* pInstance, int agent_id);
    int get_LB();


    int cost, LB, focal_score;
    Conflict first_conflict;
    vector<int > fmin;
    vector<shared_ptr<Path > > cost_matrix;
    vector<shared_ptr<Constraints > > constraint_sets;
};

struct ECBSNodePtrCmp{
    bool operator()(const shared_ptr<ECBSNode>& a, const shared_ptr<ECBSNode>& b) const {
        return a->cost > b->cost;
    }
};


typedef typename boost::heap::d_ary_heap<shared_ptr<ECBSNode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ECBSNodePtrCmp> >
        high_openSet_t;
using ECBSNodeHandle = high_openSet_t::handle_type;



struct ECBSNodePtrLBCmp{
    bool operator()(const shared_ptr<ECBSNode>& a, const shared_ptr<ECBSNode>& b) const {
        return a->LB > b->LB;
    }
};

typedef typename boost::heap::d_ary_heap<shared_ptr<ECBSNode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ECBSNodePtrLBCmp> >
        high_LBSet_t;
using ECBSNodeLBHandle = high_LBSet_t::handle_type;

struct compareFocalHeuristic {
    bool operator()(const ECBSNodeHandle& h1, const ECBSNodeHandle& h2) const {
        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1)->focal_score != (*h2)->focal_score) {
            return (*h1)->focal_score > (*h2)->focal_score;
        }
        return (*h1)->cost > (*h2)->cost;
    }
};

typedef typename boost::heap::d_ary_heap<
        ECBSNodeHandle, boost::heap::arity<2>, boost::heap::mutable_<true>,
        boost::heap::compare<compareFocalHeuristic> >
        high_focalSet_t;



#endif //ECBS_REMAKE_ECBSNODE_HPP
