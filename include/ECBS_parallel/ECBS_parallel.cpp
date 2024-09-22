//
// Created by YIMIN TANG on 3/19/23.
//


#include "ECBS_parallel.hpp"
#include "ECBSNode_parallel.hpp"



ECBS::~ECBS() = default;

void ECBS::clear() {
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


ECBS::ECBS(int row_number, int col_number, unordered_set<Location>& obstacles,
               vector<Location>& goals, vector<State>& start_states, float l_weight) {
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




int ECBS::heuristic(int x1, int y1, int agent_idx) {
    return this->prior_hmap[agent_idx][x1][y1];
}

bool ECBS::searchNodeIsValid(shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state, unordered_set<State, boost::hash<State> > *closedSet) {
    std::unique_lock<std::mutex> lock(this->m);

    if (new_state.x < 0 || new_state.x >= this->row_number || new_state.y < 0 || new_state.y >= this->col_number) return false;
    if (this->map2d_obstacle[new_state.x][new_state.y]) return false;
    if (closedSet->find(new_state) != closedSet->end()) return false;
    if (new_state.time >= MAX_TIMESTEP) return false;
    if (!agent_constraint_set->stateValid(new_state)) { return false;}
    if (!agent_constraint_set->transitionValid(org_state, new_state)) return false;
    return true;
}


//is called within threads
shared_ptr<Path> ECBS::findPath_a_star_eps(vector<shared_ptr<Path > >& cost_matrix, shared_ptr<Constraints>& agent_constraint_set,
                                               int agent_idx,
                                               vector<int>& fmin)
{

    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State> > closedSet;
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;

    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal> lastPointCheckMap;
    unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    get_block_map(pointCheckMap, lastPointCheckMap,  edgeCheckMap, cost_matrix, agent_idx);

    low_openSet_t openSet;
    low_focalSet_t focalSet;

    //never changed i think so not race cond
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
                                                  0,
                                                  nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));
    focalSet.push(handle);

    int best_fScore = startNode->fScore;

    while (!openSet.empty() && search_max > 0) {
        this->lowLevelExpanded ++;
        search_max--;

        int old_best_fScore = best_fScore;
        best_fScore = openSet.top()->fScore;
        if (best_fScore > old_best_fScore)
        {
            for (auto iter = openSet.ordered_begin(); iter!=openSet.ordered_end(); iter++)
            {
                int val = (*iter)->fScore;
                if (val > old_best_fScore * this->l_weight && val <= best_fScore * l_weight)
                {
                    auto handle_iter = stateToHeap.find((*iter)->state);
                    focalSet.push(handle_iter->second);
                }
                if (val > best_fScore * l_weight) break;
            }
        }

        auto current_handler = focalSet.top();
        shared_ptr<PathEntry> current = (*current_handler);

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {

            fmin[agent_idx] = best_fScore;
            shared_ptr<Path> path(new Path);
            while (current != nullptr) {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        focalSet.pop();
        openSet.erase(current_handler);
        stateToHeap.erase(current->state);
        this->m.lock();
        closedSet.insert(current->state);
        this->m.unlock();

        for (int i = 0; i < 5; i++) {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet)) continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end()) {  // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);

                this->focal_score_timer.reset();

                auto idx = make_tuple(new_state.x, new_state.y, new_state.time);
                int conflict_num =  pointCheckMap[idx];
                auto idx2 = make_tuple(new_state.x, new_state.y);
                if (lastPointCheckMap.find(idx2) != lastPointCheckMap.end())
                {
                    int t = lastPointCheckMap[idx2];
                    if (new_state.time > t) conflict_num++;
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
            } else {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)  continue;
                int last_fScore = (*handle)->fScore;

                int delta = (*handle)->gScore - tentative_gScore;
                (*handle)->gScore = tentative_gScore;
                (*handle)->fScore -= delta;
                (*handle)->parent = current;
                openSet.increase(handle);
                if ((double) (*handle)->fScore <= (double) best_fScore * this->l_weight &&
                    (double)last_fScore > (double) best_fScore * this->l_weight) {
                    focalSet.push(handle);
                }
            }
        }
    }
    return nullptr;
}

shared_ptr<Path> ECBS::findPath_a_star_eps2(vector<shared_ptr<Path > >& cost_matrix, shared_ptr<Constraints>& agent_constraint_set,
                                           int agent_idx,
                                           vector<int>& fmin)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State> > closedSet;
    unordered_map<State, PathEntryHandle2 , boost::hash<State> > stateToHeap;

    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal> lastPointCheckMap;
    unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    get_block_map(pointCheckMap, lastPointCheckMap,  edgeCheckMap, out_solution, agent_idx);

    low_openSet_t2 openSet;

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
                                                  0,
                                                  nullptr));
    {
        auto handle = openSet.push(startNode);
        stateToHeap.insert(make_pair(start_state, handle));
    }

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
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet)) continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end()) {
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, agent_idx);

                this->focal_score_timer.reset();

                auto idx = make_tuple(new_state.x, new_state.y, new_state.time);
                int conflict_num =  pointCheckMap[idx];
                auto idx2 = make_tuple(new_state.x, new_state.y);
                if (lastPointCheckMap.find(idx2) != lastPointCheckMap.end())
                {
                    int t = lastPointCheckMap[idx2];
                    if (new_state.time > t) conflict_num++;
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
                if (fScore <= fmin[agent_idx] * this->l_weight) {
                    auto handle = openSet.push(nextNode);
                    stateToHeap.insert(std::move(make_pair(new_state, handle)));
                }
            }
            else {
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

shared_ptr<Path> ECBS::findPath_a_star(shared_ptr<Constraints>& agent_constraint_set, int agent_idx, vector<int>& fmin)
{
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    unordered_set<State, boost::hash<State> > closedSet;
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;
    low_openSet_t openSet;

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
            fmin[agent_idx] = current->fScore;
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
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state, &closedSet)) continue;
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




/*
def main_solver():

    #init start node
    shared_ptr<ECBSNode> start_node(new ECBSNode());
    start_node->create_cost_matrix(this);
    start_node->focal_score = high_focal_score_v4(start_node->cost_matrix, start_node->first_conflict);

    init node heaps
    high_openSet_t open;
    high_focalSet_t focal;
    high_LBSet_t LBset;

    unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode> > > CTnode2open_handle;
    unordered_map<shared_ptr<ECBSNode>, ECBSNodeLBHandle, boost::hash<shared_ptr<ECBSNode> > > CTnode2LB_handle;
    auto handle = open.push(start_node);
    auto handle2 = LBset.push(start_node);
    focal.push(handle);
    CTnode2open_handle.insert(make_pair(start_node, handle));
    CTnode2LB_handle.insert(make_pair(start_node, handle2));
    int best_LB = start_node->LB;

    int avail_threads = 8
    while(true):
        block until avail_threads > 0
        
        for i:1 -> avail threads:
            thread(solve())
        
        block until received finished signal


    def solve():
        lock
        if open set empty:
            sleep for x time
            #TODO need a way to quit app in no solution case
        get best focal node node
        remove from all 3 sets and handlers
        unlock
        check if done by chhecking if ffocal score is 0
            if done send message to main thread manager to say we found an answer
            
        compute conflict and stuff;
        remember to lock when mod global vars and combine all set operations together
        to avoid race

*/


std::mutex b;
void solver_thread(ECBS *ecbs, 
                    std::shared_ptr<high_openSet_t> open, 
                    std::shared_ptr<high_focalSet_t> focal,
                    std::shared_ptr<high_LBSet_t> LBset, 
                    unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode> > > *CTnode2open_handle, 
                    unordered_map<shared_ptr<ECBSNode>, ECBSNodeLBHandle, boost::hash<shared_ptr<ECBSNode> > > *CTnode2LB_handle,
                    int *best_LB)
{
    while (true)
    {
        std::unique_lock<std::mutex> asdfas(b);
        std::unique_lock<std::mutex> l(ecbs->m);
        std::cout <<"started iter " << std::endl;
        std::cout << "open len: " << open->size() << std::endl;
        while (open->empty()) {
            ecbs->waiting_nodes++;
            std::cout <<"waiting" << std::endl;
            ecbs->cv.wait(l);
            if (ecbs->solution_found) {
                // No need to manually unlock, 'l' will unlock automatically
                return;
            }
            if (ecbs->waiting_nodes == 4) {
                // No need to manually unlock here either
                return;
            }
            ecbs->waiting_nodes--;
        }
        //ecbs->m.unlock();

        ecbs->cbsnode_num ++;
        int old_best_LB = *best_LB;

        // Best lower bound of all nodes in LBset is at the top of the heap
        *best_LB = LBset->top()->LB;
        //If the best LB improves, the focal set is rebuilt by adding nodes from open that meet a certain heuristic threshold (based on l_weight)
        if (*best_LB > old_best_LB)
        {
            for (auto iter = open->ordered_begin(); iter != open->ordered_end(); iter++)
            {
                int val = (*iter)->cost;
                //if val was too big before but is small enough to be inserted into focal
                if ((double)val > (double) old_best_LB * ecbs->l_weight && (double) val <= (double) *best_LB * ecbs->l_weight)
                {
                    auto handle_iter = CTnode2open_handle->find(*iter);
                    focal->push(handle_iter->second);
                }
                if ((double)val > (double) *best_LB * ecbs->l_weight) break;
            }
        }
        

        //The best node (handler) from the focal set is selected for expansion. The node is removed from both the focal and open sets
        auto current_handler = focal->top();
        shared_ptr<ECBSNode> cur_node = *current_handler; // I guess dereferencing the handle gets the ECBSnode
        focal->pop();
        //Since this node is being expanded, it is also removed from the open heap (where all non-expanded nodes are stored).
        open->erase(current_handler);
        // The node is also removed from the CTnode2open_handle hashmap, which stores a mapping from the actual node (cur_node) to its handle in the open heap.
        CTnode2open_handle->erase(cur_node);
        /*
        The LBset (Lower Bound Set) also contains this node. First, the handle for this node in LBset is retrieved from the CTnode2LB_handle hashmap.
        Then, the node is removed from LBset using its handle (LB_handler).
        The node is also erased from the CTnode2LB_handle hashmap, which tracks the mapping between the node and its handle in the LBset
        */
        auto LB_handler = (*CTnode2LB_handle)[cur_node];
        LBset->erase(LB_handler);
        CTnode2LB_handle->erase(cur_node);
        ecbs->m.unlock();

        // If conflicts == 0
        bool done = cur_node->focal_score == 0;
        if (done)
        {
            std::unique_lock<std::mutex> lock(ecbs->m);
            std::cout << "done; cost: " << cur_node->cost << std::endl;
            ecbs->out_solution = cur_node->cost_matrix;
            ecbs->cost = cur_node->cost;
            if (!check_ans_valid(ecbs->out_solution))
                std::cout << "INVALID ANS PATH!!!!" << std::endl;
            ecbs->solution_found = true;
            return;
        }

        // Map of agent index to constraints
        //idt there are any race conds here
        unordered_map<size_t, Constraints> tmp;
        createConstraintsFromConflict(cur_node->first_conflict, tmp);
        
        Timer newnode_timer; //made timer local bc not needed to be shared across multiple threads

        for (unsigned short cur_i = 0; auto &[key, value]: tmp)
        {
            ecbs->m.lock();
            newnode_timer.reset();
            shared_ptr<ECBSNode> new_node = shared_ptr<ECBSNode>(new ECBSNode(cur_node));
            newnode_timer.stop();
            ecbs->newnode_time += newnode_timer.elapsedSeconds();
            ecbs->m.unlock();

            assert(!new_node->constraint_sets[key]->overlap(value));
            new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
            new_node->constraint_sets[key]->add(value);
            cur_i ++;
            // Try to find updates path for this agent with the new constraints. if false, then no path was found 
            // and continue to next iteration
            bool b = new_node->update_cost_matrix(ecbs, key);
            if (!b) 
                continue;

            Timer conflict_num_timer;
            conflict_num_timer.reset();
            new_node->focal_score = high_focal_score_v4(new_node->cost_matrix, new_node->first_conflict);
            conflict_num_timer.stop();
            ecbs->conflict_num_time += conflict_num_timer.elapsedSeconds();
            //so handlers are becoming null in seperate threads but they arent being removed
            //
            // Add node to unexpanded set(open) and heap based off of LB
            ecbs->m.lock();
            auto handle = open->push(new_node);
            auto handle2 = LBset->push(new_node);
            // Then map the nodes to their handles
            CTnode2open_handle->insert(make_pair(new_node, handle));
            CTnode2LB_handle->insert(make_pair(new_node, handle2));
            // If the new node has a cost less than the best lower bound * some weight, then add it to the focal set
            if (new_node->cost <= *best_LB * ecbs->l_weight) {
                focal->push(handle);
            }
            ecbs->cv.notify_one();
            ecbs->m.unlock();
        
        }
    }
    return;

}

int ECBS::paralized_solver_main(){
    
    //make root node for search
    shared_ptr<ECBSNode> start_node(new ECBSNode());
    start_node->create_cost_matrix(this);
    start_node->focal_score = high_focal_score_v4(start_node->cost_matrix, start_node->first_conflict);

    std::shared_ptr<high_openSet_t> open = std::make_shared<high_openSet_t>();
    std::shared_ptr<high_focalSet_t> focal = std::make_shared<high_focalSet_t>();
    std::shared_ptr<high_LBSet_t> LBset = std::make_shared<high_LBSet_t>();
    unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode> > > CTnode2open_handle;
    unordered_map<shared_ptr<ECBSNode>, ECBSNodeLBHandle, boost::hash<shared_ptr<ECBSNode> > > CTnode2LB_handle;

    auto handle = open->push(start_node);
    auto handle2 = LBset->push(start_node);
    focal->push(handle);
    CTnode2open_handle.insert(make_pair(start_node, handle));
    CTnode2LB_handle.insert(make_pair(start_node, handle2));
    int best_LB = start_node->LB;

    int numThreads = 2;
    std::vector<std::thread> threads;


    this->m.lock();
    for (int i=0; i < numThreads; i++){
        threads.emplace_back(std::thread(solver_thread, this, open, focal, LBset, &CTnode2open_handle, &CTnode2LB_handle, &best_LB));
    }
    this->m.unlock();

    //just stall until found a solution or open is empty and all the threads are waiting
    while(true){
        if (this->solution_found){
            std::cout << "done; cost: " << this->cost << std::endl;
            
            for (auto& thread : threads){
                thread.join();  //join with all the threads created

            }
            
            return true;
        }
        usleep(500);
    }

}


  