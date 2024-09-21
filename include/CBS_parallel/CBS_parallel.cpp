//
// Created by YIMIN TANG on 3/19/23.
//

#include "CBS_parallel.hpp"
#include "CBSNode_parallel.hpp"
#include <parlay/sequence.h>

boost::mutex mtx;
const int MAX_SUB_QUEUE_SIZE=32;
using openlist = boost::heap::d_ary_heap<shared_ptr<CBSNode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<CBSNodePtrCmp> >;
CBS::~CBS() = default;

void CBS::clear() {
    solution_found.store(false);
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->firstconflict_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;
}

class PairCompare {
public:
    bool operator()(const pair<int, Location>& p1, const pair<int, Location>& p2) const {
        return p1.first > p2.first;
    }
};


CBS::CBS(int row_number, int col_number, unordered_set<Location>& obstacles,
               vector<Location>& goals, vector<State>& start_states) {
    this->row_number = row_number;
    this->col_number = col_number;
    this->map_size = row_number * col_number;
    this->start_states = start_states;
    this->obstacles = obstacles;
    this->goals = goals;

    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;

    this->map2d_obstacle.resize(this->row_number, vector<int>(this->col_number, 0));
    for (auto obstacle : obstacles) {
        int x = obstacle.x;
        int y = obstacle.y;
        this->map2d_obstacle[x][y] = 1;
    }

    this->agent_n = this->start_states.size();

    printf("Init shortest path\n");
    for (int i=0;i<this->agent_n;i++)
    {
        vector<vector<int> > prior_map;
        prior_map.resize(this->row_number, vector<int>(this->col_number, INF7f));
        Location start_loc = this->goals[i];
        prior_map[start_loc.x][start_loc.y] = 0;
        std::priority_queue<pair<int, Location>, vector<pair<int, Location> >, PairCompare > pq;
        pq.push(make_pair(0, start_loc));
        while (!pq.empty())
        {
            pair<int, Location> x = pq.top(); pq.pop();
            int c = x.first;
            Location cur_loc = x.second;

            for (int k = 0; k<4; k++)
            {
                Location new_loc(cur_loc.x + dx[k], cur_loc.y + dy[k]);
                if (new_loc.x < 0 || new_loc.x >= this->row_number || new_loc.y < 0 || new_loc.y >= this->col_number) continue;
                if (this->map2d_obstacle[new_loc.x][new_loc.y]) continue;
                if (c + 1 >= prior_map[new_loc.x][new_loc.y]) continue;
                prior_map[new_loc.x][new_loc.y] = c + 1;
                pq.push(make_pair(c+1, new_loc));
            }
        }
        this->prior_hmap[i] = std::move(prior_map);
    }
    printf("Init Done\n");
}




int CBS::heuristic(int x1, int y1, int agent_idx) {
    return this->prior_hmap[agent_idx][x1][y1];
}

/**
 * @brief Checks if a given state is valid for the search node.
 *
 * This function validates a new state based on several criteria including
 * boundary conditions, time constraints, obstacle presence, closed set membership,
 * and agent-specific constraints.
 *
 * @param closedSet A set of states that have already been evaluated.
 * @param agent_constraint_set A shared pointer to the constraints specific to the agent.
 * @param new_state The new state to be validated.
 * @param org_state The original state from which the transition is being made.
 * @return true if the new state is valid, false otherwise.
 */
bool CBS::searchNodeIsValid(unordered_set<State, boost::hash<State> >& closedSet, shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state) {
    if (new_state.x < 0 || new_state.x >= this->row_number || new_state.y < 0 || new_state.y >= this->col_number) return false;
    if (new_state.time >= MAX_TIMESTEP) return false;
    if (this->map2d_obstacle[new_state.x][new_state.y]) return false;
    if (closedSet.find(new_state) != closedSet.end()) return false;
    if (!agent_constraint_set->stateValid(new_state)) { return false;}
    if (!agent_constraint_set->transitionValid(org_state, new_state)) return false;
    return true;
}

/**
 * @brief Finds a path for a given agent using the A* algorithm considering constraints.
 * 
 * This function performs an A* search to find a path for the specified agent while considering
 * the provided constraints. It uses an open set for nodes to be evaluated and a closed set for
 * nodes that have already been evaluated. 
 * 
 * @param agent_constraint_set A shared pointer to the set of constraints for the agent.
 * @param agent_idx The index of the agent for which the path is being found.
 * @return A shared pointer to the found path, or nullptr if no path is found.
 */
shared_ptr<Path> CBS::findPath_a_star(shared_ptr<Constraints>& agent_constraint_set, int agent_idx)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State> > closedSet;
    closedSet.clear();
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;
    openSet_t openSet;

    State start_state = this->start_states[agent_idx];
    Location goal_location = this->goals[agent_idx];

    for (const auto &vc: agent_constraint_set->vertexConstraints) {
        if (vc.x == goal_location.x && vc.y == goal_location.y) {
            m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
    }

    shared_ptr<PathEntry> startNode(new PathEntry(start_state,
                                                    this->heuristic(start_state.x, start_state.y, agent_idx),
                                                    0,
                                                    nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));

    while (!openSet.empty() && search_max > 0) {
        this->lowLevelExpanded ++;
        search_max--;
        shared_ptr<PathEntry> current = openSet.top();

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {
            shared_ptr<Path> path(new Path);
            while (current != nullptr) {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        openSet.pop();
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++) {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(closedSet, agent_constraint_set, new_state, current->state)) continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end()) {  // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);
                shared_ptr<PathEntry> nextNode(new PathEntry(new_state, fScore,
                                                             tentative_gScore,
                                                             current));
                auto handle = openSet.push(nextNode);
                stateToHeap.insert(std::move(make_pair(new_state, handle)));

            } else {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)  continue;
                int delta = (*handle)->gScore - tentative_gScore;
                (*handle)->gScore = tentative_gScore;
                (*handle)->fScore -= delta;
                (*handle)->parent = current;
                openSet.increase(handle);
            }
        }
    }
    return nullptr;
}


/**
 * @brief Processes a node in the CBS (Conflict-Based Search) algorithm.
 *
 * This function processes a given node in the CBS algorithm, handling conflicts and generating new nodes
 * based on constraints derived from conflicts. It updates the solution if a valid solution is found.
 *
 * @param deal_idx The index of the node to be processed.
 * @param pInstance Pointer to the CBS instance.
 * @param in_node Shared pointer to the input CBS node to be processed.
 * @param out_node_queue Reference to a sequence of shared pointers to CBS nodes, where the output nodes will be stored.
 * @return True if a solution is found, otherwise false.
 */
bool deal_with_node(size_t deal_idx, CBS* pInstance, shared_ptr<CBSNode> in_node, parlay::sequence<shared_ptr<CBSNode> >& out_node_queue) {
    openlist open_deal;
    int begin_cost = in_node->cost;
    int node_idx = in_node->idx + 1;
    in_node->conflict_num = in_node->get_all_conflict_number();
    open_deal.push(in_node);
    while (open_deal.size() < MAX_SUB_QUEUE_SIZE) {
        if (open_deal.top()->cost != begin_cost) break;
        shared_ptr<CBSNode> cur_node = open_deal.top();open_deal.pop();
        Conflict conflict;
        if (pInstance->solution_found.load()) return true;
        bool done = !cur_node->get_first_conflict(conflict);
        if (done) {
            boost::lock_guard<boost::mutex> lock(mtx);
            pInstance->cost = cur_node->cost;
            pInstance->out_solution = cur_node->cost_matrix;
            pInstance->solution_found.store(true);
            return true;
        }

        unordered_map<size_t, Constraints> tmp;
        createConstraintsFromConflict(conflict, tmp);

        for (auto &[key, value]: tmp)
        {
            shared_ptr<CBSNode> new_node;
            new_node = shared_ptr<CBSNode>(new CBSNode(cur_node));

            assert(!new_node->constraint_sets[key]->overlap(value));
            new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
            new_node->constraint_sets[key]->add(value);
            bool b = new_node->update_cost_matrix(pInstance, key);
            if (!b) continue;
            new_node->idx = node_idx; node_idx++;
            open_deal.push(new_node);
        }
    }
    int n = open_deal.size();
    for (int i=n-1;i>=0;i--)
    {
        auto tmp = open_deal.top();
        out_node_queue[deal_idx*MAX_SUB_QUEUE_SIZE+i] = tmp;
        open_deal.pop();
    }
    return false;
}


int CBS::solve() {
    shared_ptr<CBSNode> start_node(new CBSNode());

    start_node->create_cost_matrix(this);
    this->first_cost_matrix = start_node->cost_matrix;
    openlist open;
    int node_idx=0;
    open.push(start_node);

    while (!open.empty())
    {
        parlay::sequence<shared_ptr<CBSNode> > cur_node_queue;
        shared_ptr<CBSNode> cur_node = open.top(); open.pop();
        cur_node_queue.push_back(cur_node);

        // 1000 size constraint based on number of cores or some shit
        // We only want to process the nodes in parallel when they have the same cost. it helps ensure that the algorithm explores nodes with similar priority 
        while (!open.empty() && open.top()->cost == cur_node_queue[0]->cost && cur_node_queue.size() < 1000)
        {
            cur_node = open.top(); open.pop();
            cur_node_queue.push_back(cur_node);
        }

        this->cbsnode_num += cur_node_queue.size();

        // Queue to hold the output nodes of cur_node_queue above
        // The max amount that can be generated is probably cur_node_queue.size()*MAX_SUB_QUEUE_SIZE+1 thats why reserve that size?
        parlay::sequence<shared_ptr<CBSNode> > out_node_queue(cur_node_queue.size()*MAX_SUB_QUEUE_SIZE+1, nullptr);
        parlay::parallel_for(0, cur_node_queue.size(), [&](size_t i){
            auto cur_node = cur_node_queue[i];
            deal_with_node(i, this, cur_node, out_node_queue);
        });
        if (this->solution_found.load())
        {
            std::cout << "done; cost: " << this->cost << std::endl;
            return true;
        }
        // Push the nodes that were generated by parallel queue into the open set
        for (auto& x:out_node_queue) {
            if (x != nullptr)
            {
                x->idx = node_idx;
                node_idx++;
                open.push(x);
            }
        }
    }
    return false;
}

