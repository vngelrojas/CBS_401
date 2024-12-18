//
// Created by YIMIN TANG on 3/19/23.
//

#include "ECBS_parallel.hpp"
#include "ECBSNode_parallel.hpp"
#include <parlay/sequence.h>

// Destructor for our ECBS Class
ECBS::~ECBS() = default;

/**
 * @brief Clears the ECBS solver state.
 *
 * This function resets various runtime metrics and flags to their default values.
 * It prepares the ECBS solver for a new search iteration by resetting the solution flag
 * and performance counters such as total_runtime, lowlevel_search_time, first conflict detection time,
 * number of CBS nodes expanded, and number of low-level expansions.
 */
void ECBS::clear()
{
    solution_found = false;
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->firstconflict_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;
}

/**
 * @class PairCompare
 *
 * A comparator class used for ordering pairs in the priority queue.
 * It compares pairs based on the first value (path cost), using greater-than comparison.
 * This helps in managing the open set in the pathfinding process.
 */
class PairCompare
{
public:
    bool operator()(const pair<int, Location> &p1, const pair<int, Location> &p2) const
    {
        return p1.first > p2.first;
    }
};

/**
 * @brief Constructor for ECBS class.
 *
 * This constructor initializes the ECBS solver with a given map size, obstacles, goals, start states, and suboptimality weight (l_weight).
 * It also prepares internal data structures like the obstacle map and heuristic maps for pathfinding.
 *
 * @param row_number The number of rows in the grid.
 * @param col_number The number of columns in the grid.
 * @param obstacles An unordered set of obstacle locations.
 * @param goals A vector of goal locations for each agent.
 * @param start_states A vector of initial states for each agent.
 * @param l_weight A float representing the suboptimality bound for the ECBS algorithm.
 */
ECBS::ECBS(int row_number, int col_number, unordered_set<Location> &obstacles,
           vector<Location> &goals, vector<State> &start_states, float l_weight)
{
    this->row_number = row_number;
    this->col_number = col_number;
    this->map_size = row_number * col_number;
    this->start_states = start_states;
    this->obstacles = obstacles;
    this->goals = goals;
    this->l_weight = l_weight;

    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;

    this->map2d_obstacle.resize(this->row_number, vector<int>(this->col_number, 0));
    for (auto obstacle : obstacles)
    {
        int x = obstacle.x;
        int y = obstacle.y;
        this->map2d_obstacle[x][y] = 1;
    }

    this->agent_n = this->start_states.size();

    printf("Init shortest path\n");
    for (int i = 0; i < this->agent_n; i++)
    {
        vector<vector<int>> prior_map;
        prior_map.resize(this->row_number, vector<int>(this->col_number, INF7f));
        Location start_loc = this->goals[i];
        prior_map[start_loc.x][start_loc.y] = 0;
        std::priority_queue<pair<int, Location>, vector<pair<int, Location>>, PairCompare> pq;
        pq.push(make_pair(0, start_loc));
        while (!pq.empty())
        {
            pair<int, Location> x = pq.top();
            pq.pop();
            int c = x.first;
            Location cur_loc = x.second;

            for (int k = 0; k < 4; k++)
            {
                Location new_loc(cur_loc.x + dx[k], cur_loc.y + dy[k]);
                if (new_loc.x < 0 || new_loc.x >= this->row_number || new_loc.y < 0 || new_loc.y >= this->col_number)
                    continue;
                if (this->map2d_obstacle[new_loc.x][new_loc.y])
                    continue;
                if (c + 1 >= prior_map[new_loc.x][new_loc.y])
                    continue;
                prior_map[new_loc.x][new_loc.y] = c + 1;
                pq.push(make_pair(c + 1, new_loc));
            }
        }
        this->prior_hmap[i] = std::move(prior_map);
    }
    printf("Init Done\n");
}

/**
 * @brief Heuristic function for estimating the remaining cost to the goal.
 *
 * This function returns the heuristic value (i.e., the shortest precomputed distance) for the specified agent
 * from the given coordinates (x1, y1) to the goal.
 *
 * @param x1 The x-coordinate of the current position.
 * @param y1 The y-coordinate of the current position.
 * @param agent_idx The index of the agent for which the heuristic is being computed.
 * @return The heuristic value for the specified position and agent.
 */
int ECBS::heuristic(int x1, int y1, int agent_idx)
{
    return this->prior_hmap[agent_idx][x1][y1];
}

/**
 * @brief Checks whether a new state is valid based on several constraints.
 *
 * This function validates a new state during the search by ensuring that the state is within grid boundaries,
 * does not violate any time limits, avoids obstacles, is not already evaluated, and complies with agent-specific constraints.
 *
 * @param agent_constraint_set A shared pointer to the constraints specific to the agent.
 * @param new_state The new state to be validated.
 * @param org_state The original state from which the transition is being made.
 * @param closedSet A set of states that have already been expanded.
 * @return True if the new state is valid; false otherwise.
 */
bool ECBS::searchNodeIsValid(shared_ptr<Constraints> &agent_constraint_set, const State &new_state, const State &org_state, unordered_set<State, boost::hash<State>> *closedSet)
{

    if (new_state.x < 0 || new_state.x >= this->row_number || new_state.y < 0 || new_state.y >= this->col_number)
        return false;
    if (this->map2d_obstacle[new_state.x][new_state.y])
        return false;
    if (closedSet->find(new_state) != closedSet->end())
        return false;
    if (new_state.time >= MAX_TIMESTEP)
        return false;
    if (!agent_constraint_set->stateValid(new_state))
    {
        return false;
    }
    if (!agent_constraint_set->transitionValid(org_state, new_state))
        return false;
    return true;
}

/**
 * @brief Finds a path for a specific agent using an epsilon-greedy A* search.
 *
 * This function performs an A* search with epsilon greedy selection, considering various constraints for each agent.
 * It uses open and focal sets for node expansion and updates both sets dynamically during the search process.
 * Searches for a valid path for an agent that balances speed and optimality by exploring paths that are suboptimal
 * but within the bound set by l_weight.
 *
 * Is called within threads
 *
 * @param cost_matrix A vector of shared pointers to the cost matrices of all agents.
 * @param agent_constraint_set A shared pointer to the set of constraints for the agent.
 * @param agent_idx The index of the agent for which the path is being found.
 * @param fmin A reference to a vector holding the minimum cost for each agent.
 * @return A shared pointer to the found path, or nullptr if no path is found.
 */
shared_ptr<Path> ECBS::findPath_a_star_eps(vector<shared_ptr<Path>> &cost_matrix, shared_ptr<Constraints> &agent_constraint_set,
                                           int agent_idx,
                                           vector<int> &fmin)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State>> closedSet;
    unordered_map<State, PathEntryHandle, boost::hash<State>> stateToHeap;

    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal> lastPointCheckMap;
    unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    get_block_map(pointCheckMap, lastPointCheckMap, edgeCheckMap, cost_matrix, agent_idx);

    low_openSet_t openSet;
    low_focalSet_t focalSet;

    // never changed i think so not race cond
    State start_state = this->start_states[agent_idx];
    Location goal_location = this->goals[agent_idx];

    for (const auto &vc : agent_constraint_set->vertexConstraints)
    {
        if (vc.x == goal_location.x && vc.y == goal_location.y)
        {
            m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
    }

    shared_ptr<PathEntry> startNode(new PathEntry(start_state,
                                                  this->heuristic(start_state.x, start_state.y, agent_idx),
                                                  0,
                                                  0,
                                                  nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));
    focalSet.push(handle);

    int best_fScore = startNode->fScore;

    while (!openSet.empty() && search_max > 0)
    {
        this->lowLevelExpanded++;
        search_max--;

        int old_best_fScore = best_fScore;
        best_fScore = openSet.top()->fScore;
        if (best_fScore > old_best_fScore)
        {
            for (auto iter = openSet.ordered_begin(); iter != openSet.ordered_end(); iter++)
            {
                int val = (*iter)->fScore;
                if (val > old_best_fScore * this->l_weight && val <= best_fScore * l_weight)
                {
                    auto handle_iter = stateToHeap.find((*iter)->state);
                    focalSet.push(handle_iter->second);
                }
                if (val > best_fScore * l_weight)
                    break;
            }
        }

        auto current_handler = focalSet.top();
        shared_ptr<PathEntry> current = (*current_handler);

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {

            fmin[agent_idx] = best_fScore;
            shared_ptr<Path> path(new Path);
            while (current != nullptr)
            {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        focalSet.pop();
        openSet.erase(current_handler);
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++)
        {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet))
                continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end())
            { // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);

                this->focal_score_timer.reset();

                auto idx = make_tuple(new_state.x, new_state.y, new_state.time);
                int conflict_num = pointCheckMap[idx];
                auto idx2 = make_tuple(new_state.x, new_state.y);
                if (lastPointCheckMap.find(idx2) != lastPointCheckMap.end())
                {
                    int t = lastPointCheckMap[idx2];
                    if (new_state.time > t)
                        conflict_num++;
                }
                auto ky_tuple = make_tuple(current->state.x, current->state.y, current->state.time, new_state.x, new_state.y);
                conflict_num += edgeCheckMap[ky_tuple];
                int focalScore = current->focalScore + conflict_num;
                this->focal_score_timer.stop();
                this->focal_score_time += this->focal_score_timer.elapsedSeconds();

                shared_ptr<PathEntry> nextNode(new PathEntry(new_state,
                                                             fScore,
                                                             tentative_gScore,
                                                             focalScore,
                                                             current));

                auto handle = openSet.push(nextNode);
                stateToHeap.insert(std::move(make_pair(new_state, handle)));
                if (fScore <= best_fScore * this->l_weight)
                    focalSet.push(handle);
            }
            else
            {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)
                    continue;
                int last_fScore = (*handle)->fScore;

                int delta = (*handle)->gScore - tentative_gScore;
                (*handle)->gScore = tentative_gScore;
                (*handle)->fScore -= delta;
                (*handle)->parent = current;
                openSet.increase(handle);
                if ((double)(*handle)->fScore <= (double)best_fScore * this->l_weight &&
                    (double)last_fScore > (double)best_fScore * this->l_weight)
                {
                    focalSet.push(handle);
                }
            }
        }
    }
    return nullptr;
}

/**
 * @brief Performs epsilon-greedy A* search for pathfinding.
 *
 * This function performs an epsilon-greedy variant of the A* search to find a path for an agent.
 * It accounts for the agent's constraints, and updates its search based on focal and heuristic values
 * as well as precomputed conflict maps to handle collisions. The function also returns the found path or nullptr.
 *
 * @param cost_matrix A vector of shared pointers to the cost matrices of all agents.
 * @param agent_constraint_set A shared pointer to the set of constraints for the agent.
 * @param agent_idx The index of the agent for which the path is being found.
 * @param fmin A reference to a vector holding the minimum cost for each agent.
 * @return A shared pointer to the found path, or nullptr if no path is found.
 */
shared_ptr<Path> ECBS::findPath_a_star_eps2(vector<shared_ptr<Path>> &cost_matrix, shared_ptr<Constraints> &agent_constraint_set,
                                            int agent_idx,
                                            vector<int> &fmin)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State>> closedSet;
    unordered_map<State, PathEntryHandle2, boost::hash<State>> stateToHeap;

    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal> lastPointCheckMap;
    unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    get_block_map(pointCheckMap, lastPointCheckMap, edgeCheckMap, out_solution, agent_idx);

    low_openSet_t2 openSet;

    State start_state = this->start_states[agent_idx];

    Location goal_location = this->goals[agent_idx];

    for (const auto &vc : agent_constraint_set->vertexConstraints)
    {
        if (vc.x == goal_location.x && vc.y == goal_location.y)
        {
            m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
    }

    shared_ptr<PathEntry> startNode(new PathEntry(start_state,
                                                  this->heuristic(start_state.x, start_state.y, agent_idx),
                                                  0,
                                                  0,
                                                  nullptr));
    {
        auto handle = openSet.push(startNode);
        stateToHeap.insert(make_pair(start_state, handle));
    }

    while (!openSet.empty() && search_max > 0)
    {
        this->lowLevelExpanded++;
        search_max--;

        shared_ptr<PathEntry> current = openSet.top();

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {
            shared_ptr<Path> path(new Path);
            while (current != nullptr)
            {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        openSet.pop();
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++)
        {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet))
                continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end())
            {
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);

                this->focal_score_timer.reset();

                auto idx = make_tuple(new_state.x, new_state.y, new_state.time);
                int conflict_num = pointCheckMap[idx];
                auto idx2 = make_tuple(new_state.x, new_state.y);
                if (lastPointCheckMap.find(idx2) != lastPointCheckMap.end())
                {
                    int t = lastPointCheckMap[idx2];
                    if (new_state.time > t)
                        conflict_num++;
                }
                auto ky_tuple = make_tuple(current->state.x, current->state.y, current->state.time, new_state.x,
                                           new_state.y);
                conflict_num += edgeCheckMap[ky_tuple];
                int focalScore = current->focalScore + conflict_num;
                this->focal_score_timer.stop();
                this->focal_score_time += this->focal_score_timer.elapsedSeconds();

                shared_ptr<PathEntry> nextNode(new PathEntry(new_state,
                                                             fScore,
                                                             tentative_gScore,
                                                             focalScore,
                                                             current));
                if (fScore <= fmin[agent_idx] * this->l_weight)
                {
                    auto handle = openSet.push(nextNode);
                    stateToHeap.insert(std::move(make_pair(new_state, handle)));
                }
            }
            else
            {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)
                    continue;

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
 * @brief Finds a path for an agent using A* search while respecting constraints.
 *
 * This function performs an A* search for a specific agent while considering constraints that limit
 * the agent's movement. The open set is used to expand nodes, while the closed set keeps track of
 * visited nodes. The function returns a valid path or nullptr if no valid path is found.
 *
 * @param agent_constraint_set A shared pointer to the set of constraints specific to the agent.
 * @param agent_idx The index of the agent for which the path is being found.
 * @param fmin A reference to a vector holding the minimum cost for each agent.
 * @return A shared pointer to the found path, or nullptr if no path is found.
 */
shared_ptr<Path> ECBS::findPath_a_star(shared_ptr<Constraints> &agent_constraint_set, int agent_idx, vector<int> &fmin)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State>> closedSet;
    unordered_map<State, PathEntryHandle, boost::hash<State>> stateToHeap;
    low_openSet_t openSet;

    State start_state = this->start_states[agent_idx];
    Location goal_location = this->goals[agent_idx];

    for (const auto &vc : agent_constraint_set->vertexConstraints)
    {
        if (vc.x == goal_location.x && vc.y == goal_location.y)
        {
            m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
    }

    shared_ptr<PathEntry> startNode(new PathEntry(start_state,
                                                  this->heuristic(start_state.x, start_state.y, agent_idx),
                                                  0,
                                                  nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));

    while (!openSet.empty() && search_max > 0)
    {
        this->lowLevelExpanded++;
        search_max--;
        shared_ptr<PathEntry> current = openSet.top();
        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {
            fmin[agent_idx] = current->fScore;
            shared_ptr<Path> path(new Path);
            while (current != nullptr)
            {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        openSet.pop();
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++)
        {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet))
                continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end())
            { // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);
                shared_ptr<PathEntry> nextNode(new PathEntry(new_state, fScore,
                                                             tentative_gScore,
                                                             current));
                auto handle = openSet.push(nextNode);
                stateToHeap.insert(std::move(make_pair(new_state, handle)));
            }
            else
            {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)
                    continue;
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
 * @brief Processes a single node and deals with conflicts.
 *
 * This function handles node expansion for a specific index. It checks for conflicts, generates constraints,
 * and updates the cost matrix and focal score for newly generated nodes. The function utilizes parallelization
 * to process nodes in batches for better performance in multi-core systems.
 *
 * @param deal_idx The index of the node to process.
 * @param ecbs Pointer to the ECBS instance.
 * @param cur_node A shared pointer to the current ECBSNode being processed.
 * @param out_node_queue A sequence where newly generated nodes are stored for further processing.
 */
void deal_with_node(size_t deal_idx,
                    ECBS *ecbs,
                    shared_ptr<ECBSNode> cur_node, parlay::sequence<shared_ptr<ECBSNode>> &out_node_queue)
{
    bool done = cur_node->focal_score == 0;
    if (done)
    {
        std::unique_lock<std::mutex> lock(ecbs->m);
        ecbs->out_solution = cur_node->cost_matrix;
        ecbs->cost = cur_node->cost;
        if (!check_ans_valid(ecbs->out_solution))
            std::cout << "INVALID ANS PATH!!!!" << std::endl;
        ecbs->solution_found.store(true);
        return;
    }

    // Map of agent index to constraints
    unordered_map<size_t, Constraints> tmp;
    createConstraintsFromConflict(cur_node->first_conflict, tmp);

    for (unsigned short cur_i = 0; auto &[key, value] : tmp)
    {
        shared_ptr<ECBSNode> new_node = shared_ptr<ECBSNode>(new ECBSNode(cur_node));
        assert(!new_node->constraint_sets[key]->overlap(value));
        new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
        new_node->constraint_sets[key]->add(value);
        // Try to find updates path for this agent with the new constraints. if false, then no path was found
        // and continue to next iteration

        bool b = new_node->update_cost_matrix(ecbs, key);
        if (!b) // continue if no valid path found
            continue;

        // update focal score for current node
        new_node->focal_score = high_focal_score_v4(new_node->cost_matrix, new_node->first_conflict);

        // add to ouutput buffer queue. will be added back into open set in the main fuction
        out_node_queue[deal_idx * 2 + (size_t)cur_i] = new_node;
        cur_i++;
    }
    return;
}

/**
 * @brief Pops the top node from the focal set and removes it from the open set and tracking maps.
 *
 * This function retrieves and removes the top node (the node with the highest priority) from the focal set.
 * It also removes the corresponding entry from the open set and the CTnode2open_handle map, which tracks
 * the association between ECBS nodes and their handles.
 *
 * @param open A reference to the high-level open set that stores all non-expanded nodes.
 * @param focal A reference to the focal set, which stores nodes currently being processed.
 * @param CTnode2open_handle A map from shared pointers to ECBS nodes to their respective handles in the open set.
 * @return A shared pointer to the ECBSNode that was popped from the focal set.
 */
shared_ptr<ECBSNode> pop_node(high_openSet_t &open, high_focalSet_t &focal, unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode>>> &CTnode2open_handle)
{
    auto current_handler = focal.top();
    shared_ptr<ECBSNode> cur_node = *current_handler; // I guess dereferencing the handle gets the ECBSnode
    focal.pop();
    // Since this node is being expanded, it is also removed from the open heap (where all non-expanded nodes are stored).
    open.erase(current_handler);
    // The node is also removed from the CTnode2open_handle hashmap, which stores a mapping from the actual node (cur_node) to its handle in the open heap.
    CTnode2open_handle.erase(cur_node);
    return cur_node;
}

/**
 * @brief Solves the multi-agent pathfinding problem using ECBS.
 *
 * This function implements the main search loop for ECBS. It initializes the root node,
 * manages open and focal sets, and processes nodes in parallel using parlay to optimize for multi-core execution.
 * The search terminates when a valid solution is found or all nodes have been expanded.
 *
 * @return Returns 1 (true) if a solution is found, otherwise 0 (false).
 */
int ECBS::solve()
{

    // make root node to start off the search at the initial state
    shared_ptr<ECBSNode> start_node(new ECBSNode());
    start_node->create_cost_matrix(this);
    start_node->focal_score = high_focal_score_v4(start_node->cost_matrix, start_node->first_conflict);
    start_node->in_focal = true;

    // initialize open and focal sets. also initialize ecbs node set for fast node lookup
    high_openSet_t open;
    high_focalSet_t focal;
    unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode>>> CTnode2open_handle;

    // push the starting node into the open and focal sets.
    auto handle = open.push(start_node);
    focal.push(handle);
    CTnode2open_handle.insert(make_pair(start_node, handle));

    int iteration_num = 0; // variable just to keep track of # iterations program has done
    while (!open.empty())
    {

        // create a sequence for nodes to be processed in this iteration and cap the number of nodes processed to be under nodes_per_iteration
        int nodes_per_iteration = 10000;
        parlay::sequence<shared_ptr<ECBSNode>> cur_node_queue;

        cout << "iter num " << iteration_num << " " << "focal len " << focal.size() << endl;

        // add nodes into the cur_node_queue to be processed this iteration until either focal is empty or we hit the limit of # nodes to process this iteration
        while (!focal.empty() && !cur_node_queue.size() < nodes_per_iteration)
        {
            shared_ptr<ECBSNode> cur_node = pop_node(open, focal, CTnode2open_handle);
            cur_node_queue.push_back(cur_node);
        }
        // update the total number of nodes seen in algorithm
        this->cbsnode_num += cur_node_queue.size(); // keep total of nodes search

        // initialize a buffer list for threads to add nodes into as they individually find conflicts
        parlay::sequence<shared_ptr<ECBSNode>> out_node_queue(cur_node_queue.size() * 2 + 1, nullptr);

        // use parlay's parallel for to process each node in the cur_node_queue
        // in a parallized fashion to utilize multiple cpu cores.
        // the data has been partitioned such that there will be no race conditions between threads
        // loops from 0 -> n calling the deal_with_node function on each node in the cur_node_queue
        parlay::parallel_for(0, cur_node_queue.size(), [&](size_t i)
                             {
            auto cur_node = cur_node_queue[i];
            deal_with_node(i, this, cur_node, out_node_queue); });

        // if we found a solution, output some values and return
        if (this->solution_found.load())
        {
            std::cout << "done; cost: " << this->cost << std::endl;
            std::cout << "nodes searched: " << this->cbsnode_num << std::endl;
            return true;
        }

        // move nodes from the output node queue buffer into the open set
        for (auto &new_node : out_node_queue)
        {
            if (new_node != nullptr)
            {
                auto handle = open.push(new_node);
                CTnode2open_handle.insert(make_pair(new_node, handle));
            }
        }
        // update the lower bound
        int best_LB = open.top()->LB;
        // update the focal set
        // If the best LB improves, the focal set is rebuilt by adding nodes from open that meet a certain heuristic threshold (based on l_weight)
        for (auto iter = open.ordered_begin(); iter != open.ordered_end(); iter++)
        {
            int val = (*iter)->cost;
            // if the current node hasn't been added to the focal set yet but is also small enough to be inserted into focal
            if (!(*iter)->in_focal && (double)val <= (double)best_LB * this->l_weight)
            {
                // add node to focal set
                auto handle_iter = CTnode2open_handle.find(*iter);
                focal.push(handle_iter->second);
                (*iter)->in_focal = true;
            }
            // break once we see a node with value above the minimum threshold
            if ((double)val > (double)best_LB * this->l_weight)
                break;
        }

        iteration_num++; // debug variable
        // cout << "ending focal len" << focal.size() << endl;
    }

    return 0;
}
