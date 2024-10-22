//
// Created by YIMIN TANG on 3/20/23.
//
#include "ECBSNode_parallel.hpp"
#include "ECBS_parallel.hpp"

/**
 * @brief Constructor for ECBSNode class that creates a copy from an existing ECBSNode.
 *
 * This constructor initializes new ECBSNode by copying the cost, constraint sets, cost matrix,
 * lower bound, and minimum cost from existing ECBSNode.
 *
 * @param curnode A shared pointer to the ECBSNode that is being copied.
 */
ECBSNode::ECBSNode(shared_ptr<ECBSNode> curnode) {
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    cost_matrix = curnode->cost_matrix;
    LB = curnode->LB;
    fmin = curnode->fmin;
}

/**
 * @brief Default constructor for ECBSNode class.
 *
 * Initializes new ECBSNode object with default values for lower bound and focal score.
 */
ECBSNode::ECBSNode(){
    LB = 0;
    focal_score = 0;
}

/**
 * @brief Creates the cost matrix for the current ECBSNode.
 *
 * This function initializes the constraint sets and cost matrix for all agents in the ECBS instance.
 * Performs a low-level A* search for each agent to find their optimal paths while considering their
 * constraints. The total cost is accumulated, and lower bound is calculated based on the
 * minimum cost values for each agent
 *
 * @param pInstance A pointer to the ECBS instance containing the agents and search parameters.
 */
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

/**
 * @brief Computes the lower bound for the current ECBSNode.
 *
 * This function calculates the lower bound by summing the minimum cost values for all agents.
 * The LB is used as an estimate of the best possible cost for the node in the search tree.
 *
 * @return The lower boundfor the current ECBSNode.
 */
int ECBSNode::get_LB()
{
    int fmin_val = 0;
    int n = this->cost_matrix.size();
    for (int i=0;i<n;i++)
        fmin_val += fmin[i];
    return fmin_val;
}

/**
 * @brief Updates the cost matrix for a given agent in the ECBSNode.
 *
 * This function resets the low-level search timer, updates the cost matrix for the specified agent
 * using the A* epsilon search algorithm, and then stops the timer. It also accumulates the total 
 * low-level search time. If the updated cost matrix for the agent is null, the function returns false.
 * Otherwise, it calculates the total cost for all agents and updates the lower bound of the node.
 *
 * @param pInstance Pointer to the ECBS instance.
 * @param agent_id The ID of the agent for which the cost matrix is being updated.
 * @return True if the cost matrix was successfully updated, false otherwise.
 */
bool ECBSNode::update_cost_matrix(ECBS* pInstance, int agent_id) {
    Timer lowlevel_search_timer;
    lowlevel_search_timer.reset();
    this->cost_matrix[agent_id] = pInstance->findPath_a_star_eps(this->cost_matrix, this->constraint_sets[agent_id], agent_id, this->fmin);
//    pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id, this->fmin);
//    this->cost_matrix[agent_id] = pInstance->findPath_a_star_eps2(this->cost_matrix, this->constraint_sets[agent_id], agent_id, this->fmin);
    lowlevel_search_timer.stop();
    pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    if (this->cost_matrix[agent_id] == nullptr) return 0;
    this->cost = 0;
    for (int i=0;i<pInstance->agent_n;i++) {
        this->cost += this->cost_matrix[i]->back().gScore;
    }
    LB = this->get_LB();
    return 1;
}