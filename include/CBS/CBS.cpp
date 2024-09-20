//
// Created by YIMIN TANG on 3/19/23.
//

#include "CBS.hpp"
#include "CBSNode.hpp"


CBS::~CBS() = default;

/**
 * @brief Resets the CBS (Conflict-Based Search) solver state.
 * 
 * This function clears the internal state of the CBS solver by resetting
 * various runtime metrics and counters to their initial values. It sets
 * the solution_found flag to false and resets the total runtime, low-level
 * search time, first conflict detection time, number of CBS nodes, number
 * of low-level expansions, and the number of target assignments.
 */
void CBS::clear() {
    solution_found = false;
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

    this->num_of_agents = this->start_states.size();

    printf("Init shortest path\n");
    for (int i=0;i<this->num_of_agents;i++)
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




/**
 * @brief Computes the heuristic value for a given position and agent.
 *
 * This function retrieves the precomputed heuristic value from the prior_hmap
 * for the specified agent at the given coordinates (x1, y1).
 *
 * @param x1 The x-coordinate of the position.
 * @param y1 The y-coordinate of the position.
 * @param agent_idx The index of the agent for which the heuristic is being computed.
 * @return The heuristic value for the specified position and agent.
 */
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
bool CBS::searchNodeIsValid(shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state) {
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
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state)) continue;
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


int CBS::solve() {
    shared_ptr<CBSNode> start_node(new CBSNode());
    // Create intial cost matrix
    start_node->create_cost_matrix(this);
    this->first_cost_matrix = start_node->cost_matrix;
    // Open heap where CBSNodes can have 2 children
    typename boost::heap::d_ary_heap<shared_ptr<CBSNode>,
                boost::heap::mutable_<true>,
                boost::heap::arity<2>,
                boost::heap::compare<CBSNodePtrCmp> >
            open;

    open.push(start_node);

    while (!open.empty())
    {
        this->cbsnode_num ++;
        shared_ptr<CBSNode> cur_node = open.top(); open.pop();
        Conflict conflict;
        this->firstconflict_timer.reset();
        bool done = !cur_node->get_first_conflict(conflict);
        this->firstconflict_timer.stop();
        this->firstconflict_time += this->firstconflict_timer.elapsedSeconds();

        // If no conflicts in the cost matrix, then we have a solution
        if (done)
        {
            std::cout << "done; cost: " << cur_node->cost << std::endl;
            this->out_solution = cur_node->cost_matrix;
            if (!check_ans_valid(this->out_solution))
                std::cout << "INVALID ANS PATH!!!!" << std::endl;
            this->cost = cur_node->cost;
            this->solution_found = true;
            this->constraint_sets = cur_node->constraint_sets;
            return true;
        }
        
        unordered_map<size_t, Constraints> agent_to_constraints;

        // Create constraints from the conflict and add them to map
        createConstraintsFromConflict(conflict, agent_to_constraints);
       
        for (unsigned short cur_i = 0; auto &[agent, constraints]: agent_to_constraints)
        {
            shared_ptr<CBSNode> new_node;

            // Idk what the point of this if/else is
            if (cur_i == 1) // Make new node point to same node as cur_node
                new_node = cur_node;
            else  // make a deep copy of the current node
                new_node = shared_ptr<CBSNode>(new CBSNode(cur_node));
            // Check that no overlap between the new constraints and the agent's constraint set
            assert(!new_node->constraint_sets[agent]->overlap(constraints));
            // Ensure a deep copy of the origianl constraints? idk
            new_node->constraint_sets[agent] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[agent])));
            // Add the new constraints to the agent's constraint set
            new_node->constraint_sets[agent]->add(constraints);
            // idk
            cur_i ++;

            bool b = new_node->update_cost_matrix(this, agent);

            // if the path is not found, then skip adding this node to the open heap
            if (!b) 
                continue;
            
            open.push(new_node);
        }
    }
    return false;
}

