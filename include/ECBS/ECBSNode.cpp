//
// Created by YIMIN TANG on 3/20/23.
//
#include "ECBSNode.hpp"
#include "ECBS.hpp"


ECBSNode::ECBSNode(shared_ptr<ECBSNode> curnode) {
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    cost_matrix = curnode->cost_matrix;
    LB = curnode->LB;
    fmin = curnode->fmin;
}

ECBSNode::ECBSNode(){
    LB = 0;
    focal_score = 0;
}



void ECBSNode::create_cost_matrix(ECBS* pInstance) {
    constraint_sets.resize(pInstance->agent_n, shared_ptr<Constraints>(nullptr));
    cost_matrix.resize(pInstance->agent_n,shared_ptr<Path >(nullptr));
    fmin.resize(pInstance->agent_n, 0);
    cost = 0;
    for (int i=0; i< pInstance->agent_n;i++)
    {
        constraint_sets[i] = shared_ptr<Constraints>(new Constraints);
        pInstance->lowlevel_search_timer.reset();
        cost_matrix[i] = pInstance->findPath_a_star(this->constraint_sets[i], i, this->fmin);
        cost += cost_matrix[i]->back().gScore;
        pInstance->lowlevel_search_timer.stop();
        pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    }
    LB = this->get_LB();
}



int ECBSNode::get_LB()
{
    int fmin_val = 0;
    int n = this->cost_matrix.size();
    for (int i=0;i<n;i++)
        fmin_val += fmin[i];
    return fmin_val;
}

bool ECBSNode::update_cost_matrix(ECBS* pInstance, int agent_id) {
    pInstance->lowlevel_search_timer.reset();
    this->cost_matrix[agent_id] = pInstance->findPath_a_star_eps(this->cost_matrix, this->constraint_sets[agent_id], agent_id, this->fmin);
//    pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id, this->fmin);
//    this->cost_matrix[agent_id] = pInstance->findPath_a_star_eps2(this->cost_matrix, this->constraint_sets[agent_id], agent_id, this->fmin);
    pInstance->lowlevel_search_timer.stop();
    pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    if (this->cost_matrix[agent_id] == nullptr) return 0;
    this->cost = 0;
    for (int i=0;i<pInstance->agent_n;i++) {
        this->cost += this->cost_matrix[i]->back().gScore;
    }
    LB = this->get_LB();
    return 1;
}



