//
// Created by YIMIN TANG on 3/20/23.
//
#include "CBSNode.hpp"
#include "CBS.hpp"


CBSNode::CBSNode(shared_ptr<CBSNode> curnode) {
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    cost_matrix = curnode->cost_matrix;
}

CBSNode::CBSNode(){
}


/**
 * @brief Creates the cost matrix for the CBS node.
 *
 * This function initializes the constraint sets and cost matrix for each agent in the CBS instance.
 * It iterates over all agents, resets the low-level search timer, finds the path using A* search,
 * updates the total cost of the node, and accumulates the low-level search time.
 *
 * @param pInstance Pointer to the CBS instance containing the agents and search parameters info.
 */
void CBSNode::create_cost_matrix(CBS* pInstance) {
    constraint_sets.resize(pInstance->num_of_agents, shared_ptr<Constraints>(nullptr));
    cost_matrix.resize(pInstance->num_of_agents,shared_ptr<Path >(nullptr));
    
    // For every agent, find the path using A* search
    for (int i=0; i< pInstance->num_of_agents;i++)
    {
        constraint_sets[i] = shared_ptr<Constraints>(new Constraints);
        pInstance->lowlevel_search_timer.reset();
        cost_matrix[i] = pInstance->findPath_a_star(this->constraint_sets[i], i);
        this->cost += cost_matrix[i]->back().gScore;
        pInstance->lowlevel_search_timer.stop();
        pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    }
}

bool npuzzle = 0;

/**
 * @brief Finds the first conflict in the cost matrix.
 *
 * This function iterates through the cost matrix to find the first conflict
 * between agents. It checks for vertex collisions, edge (swap) collisions.
 * IDK what npuzzle is.
 *
 * @param result A reference to a Conflict object where the details of the first
 * conflict will be stored if found.
 * @return true if a conflict is found, false otherwise.
 */
bool CBSNode::get_first_conflict(Conflict &result) {
    int num_of_agents = this->cost_matrix.size();
    int max_t = 0;
    for (const auto &sol: this->cost_matrix) {
        max_t = std::max<int>(max_t, sol->size());
    }

    for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        for (size_t i = 0; i < num_of_agents; ++i) {
            State state1 = getState(this->cost_matrix[i], t);
            for (size_t j = i + 1; j < num_of_agents; ++j) {
                State state2 = getState(this->cost_matrix[j], t);
                if (state1.equalExceptTime(state2)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.type = Conflict::Vertex;
                    result.x1 = state1.x;
                    result.y1 = state1.y;
                    return true;
                }
            }
        }
        // drive-drive edge (swap)
        for (size_t i = 0; i < num_of_agents; ++i) {
            State state1a = getState(this->cost_matrix[i], t);
            State state1b = getState(this->cost_matrix[i], t + 1);
            for (size_t j = i + 1; j < num_of_agents; ++j) {
                State state2a = getState(this->cost_matrix[j], t);
                State state2b = getState(this->cost_matrix[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.type = Conflict::Edge;
                    result.x1 = state1a.x;
                    result.y1 = state1a.y;
                    result.x2 = state1b.x;
                    result.y2 = state1b.y;
                    return true;
                }
            }
        }
        if (npuzzle)
            for (size_t i = 0; i < num_of_agents; ++i) {
                State state1 = getState(this->cost_matrix[i], t);
                State state12 = getState(this->cost_matrix[i], t+1);
                for (size_t j = i + 1; j < num_of_agents; ++j) {
                    State state2 = getState(this->cost_matrix[j], t);
                    if (state12.equalExceptTime(state2)) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Npuzzle;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
                        result.x2 = state2.x;
                        result.y2 = state2.y;
                        return true;
                    }
                }
            }
    }

    return false;
}

/**
 * @brief Updates the cost matrix for a specific agent in the CBSNode.
 *
 * This function resets the low-level search timer, finds the path for the given agent using the A* algorithm,
 * and updates the cost matrix for that agent. It then stops the timer and adds the elapsed time to the total
 * low-level search time. The overall cost of the node is recalculated based on the gScores of the paths in the
 * cost matrix.
 *
 * @param pInstance Pointer to the CBS instance.
 * @param agent_id The ID of the agent for which the cost matrix is being updated.
 * @return True (1) if the cost matrix for the agent is successfully updated, False (0) if the path is not found.
 */
bool CBSNode::update_cost_matrix(CBS* pInstance, int agent_id) {
    pInstance->lowlevel_search_timer.reset();
    this->cost_matrix[agent_id] = pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id);
    pInstance->lowlevel_search_timer.stop();
    pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    this->cost = 0;
    // if nullptr, then A* didnt find a path
    if (this->cost_matrix[agent_id] == nullptr) 
        return 0;
    
    for (int i=0;i<pInstance->num_of_agents;i++) {
        this->cost += this->cost_matrix[i]->back().gScore;
    }
    return 1;
}



