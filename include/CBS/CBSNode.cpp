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

void CBSNode::create_cost_matrix(CBS* pInstance) {
    constraint_sets.resize(pInstance->agent_n, shared_ptr<Constraints>(nullptr));
    cost_matrix.resize(pInstance->agent_n,shared_ptr<Path >(nullptr));
    for (int i=0; i< pInstance->agent_n;i++)
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

bool CBSNode::get_first_conflict(Conflict &result) {
    int agent_n = this->cost_matrix.size();
    int max_t = 0;
    for (const auto &sol: this->cost_matrix) {
        max_t = std::max<int>(max_t, sol->size());
    }

    for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        for (size_t i = 0; i < agent_n; ++i) {
            State state1 = getState(this->cost_matrix[i], t);
            for (size_t j = i + 1; j < agent_n; ++j) {
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
        for (size_t i = 0; i < agent_n; ++i) {
            State state1a = getState(this->cost_matrix[i], t);
            State state1b = getState(this->cost_matrix[i], t + 1);
            for (size_t j = i + 1; j < agent_n; ++j) {
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
            for (size_t i = 0; i < agent_n; ++i) {
                State state1 = getState(this->cost_matrix[i], t);
                State state12 = getState(this->cost_matrix[i], t+1);
                for (size_t j = i + 1; j < agent_n; ++j) {
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

bool CBSNode::update_cost_matrix(CBS* pInstance, int agent_id) {
    pInstance->lowlevel_search_timer.reset();
    this->cost_matrix[agent_id] = pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id);
    pInstance->lowlevel_search_timer.stop();
    pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
    this->cost = 0;
    if (this->cost_matrix[agent_id] == nullptr) return 0;
    for (int i=0;i<pInstance->agent_n;i++) {
        this->cost += this->cost_matrix[i]->back().gScore;
    }
    return 1;
}



