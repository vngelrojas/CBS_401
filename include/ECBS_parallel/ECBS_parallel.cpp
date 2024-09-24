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
    this->waiting_nodes = 0;

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
    this->waiting_nodes = 0;

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
    //std::unique_lock<std::mutex> lock(this->m); TODO

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
        closedSet.insert(current->state);

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
std::mutex open_set_mtx;





class Solver{
public: 
//idk if the type actually is shared_ptr<ecbsnode> but hope it is
    std::vector<shared_ptr<ECBSNode>> _buffer;
    high_openSet_t _open;
    high_focalSet_t _focal;
    //high_LBSet_t _LBset;
    //std::shared_ptr<high_LBSet_t> LBset = std::make_shared<high_LBSet_t>();

    unordered_map<shared_ptr<ECBSNode>, ECBSNodeHandle, boost::hash<shared_ptr<ECBSNode> > > _CTnode2open_handle;
    unordered_map<shared_ptr<ECBSNode>, ECBSNodeLBHandle, boost::hash<shared_ptr<ECBSNode> > > _CTnode2LB_handle;

    int _id;
    std::shared_ptr<std::thread> _thread;
    std::mutex _m;
    std::condition_variable _cv;

    int push_from_buffer(){
        while (!_buffer.empty()){
            if (_buffer.back() == nullptr) {
                std::cout << "dying on command" << std::endl;
                return 1;
            }
            std::unique_lock<std::mutex> a(open_set_mtx);
            auto handle = _open.push(_buffer.back());
            _CTnode2open_handle.insert(make_pair(_buffer.back(), handle));
            _buffer.pop_back();
        }
        return 0;
    }

    int wait_for_work(){
        std::unique_lock<std::mutex> l(_m);
        int isDone = push_from_buffer();
        if (isDone){
            return 1;
        }
        while (_open.empty() ) {
            _cv.wait(l);
            int isDone = push_from_buffer();
            if (isDone){
                return 1;
            }
        }
        
        return 0;


    }

    void add_work(shared_ptr<ECBSNode> new_node){
        std::unique_lock<std::mutex> l(_m);
        _buffer.push_back(new_node);
        _cv.notify_all();

    }



};

int find_best_lb(std::vector<shared_ptr<Solver>> &solvers){
    std::lock_guard<std::mutex> lock(open_set_mtx); // Lock the mutex to make the function atomic

    
    int best = INT_MAX;
    for (shared_ptr<Solver> solver : solvers){
        if (!solver->_open.empty()){
            if (solver->_open.top()->LB < best){
                best = solver->_open.top()->LB;
            }
        }
    }
    if (best == INT_MAX){
        cout << "all open sets empty " << std::endl;
    }
    return best;
}

void solver_thread(ECBS *ecbs, 
                    shared_ptr<Solver> solver,
                    shared_ptr<std::vector<shared_ptr<Solver>>> solvers)
{
    ecbs->m.lock();
    ecbs->m.unlock();
    while (true)
    {
        //std::cout <<"started iter " << std::endl;
        

        int status = solver->wait_for_work();
        if (status == 1){
            return;
        }

        ecbs->m.lock();
        ecbs->cbsnode_num++;
        ecbs->m.unlock();
        


        // Best lower bound of all nodes in LBset is at the top of the heap
        int new_best_LB = find_best_lb(*solvers);
        //If the best LB improves, the focal set is rebuilt by adding nodes from open that meet a certain heuristic threshold (based on l_weight)
        open_set_mtx.lock();
        for (auto iter = solver->_open.ordered_begin(); iter != solver->_open.ordered_end(); iter++)
        {
            int val = (*iter)->cost;
            //if val was too big before but is small enough to be inserted into focal
            if (!(*iter)->in_focal && (double) val <= (double) new_best_LB * ecbs->l_weight)
            {
                auto handle_iter = solver->_CTnode2open_handle.find(*iter);
                solver->_focal.push(handle_iter->second);
                (*iter)->in_focal = true;
            }
            if ((double)val > (double) new_best_LB * ecbs->l_weight) break;
        }

     
        std::cout << solver->_id << " open size: " << solver->_open.size() << " buffer size: " << solver->_buffer.size() << " focal size: " << solver->_focal.size() << std::endl;


        //The best node (handler) from the focal set is selected for expansion. The node is removed from both the focal and open sets
        if (solver->_focal.empty()){
            open_set_mtx.unlock();
            continue;

        }
        auto current_handler = solver->_focal.top();
        shared_ptr<ECBSNode> cur_node = *current_handler; // I guess dereferencing the handle gets the ECBSnode
        solver->_focal.pop();
        //Since this node is being expanded, it is also removed from the open heap (where all non-expanded nodes are stored).
        solver->_open.erase(current_handler);
        // The node is also removed from the CTnode2open_handle hashmap, which stores a mapping from the actual node (cur_node) to its handle in the open heap.
        solver->_CTnode2open_handle.erase(cur_node);
        open_set_mtx.unlock();
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
        solver->_m.unlock();


        // Map of agent index to constraints
        unordered_map<size_t, Constraints> tmp;
        createConstraintsFromConflict(cur_node->first_conflict, tmp);
        
        Timer newnode_timer; //made timer local bc not needed to be shared across multiple threads

        for (unsigned short cur_i = 0; auto &[key, value]: tmp)
        {
            //std::unique_lock<std::mutex> locktemp(ecbs->m); //todo

            //newnode_timer.reset();
            shared_ptr<ECBSNode> new_node = shared_ptr<ECBSNode>(new ECBSNode(cur_node));
            //newnode_timer.stop();
            ecbs->newnode_time += newnode_timer.elapsedSeconds();
            

            assert(!new_node->constraint_sets[key]->overlap(value));
            new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
            new_node->constraint_sets[key]->add(value);
            // Try to find updates path for this agent with the new constraints. if false, then no path was found 
            // and continue to next iteration
            
            bool b = new_node->update_cost_matrix(ecbs, key);
            //ecbs->m.unlock();
            if (!b) 
                continue;
            //ecbs->m.lock();

            Timer conflict_num_timer;
            conflict_num_timer.reset();
            new_node->focal_score = high_focal_score_v4(new_node->cost_matrix, new_node->first_conflict);
            conflict_num_timer.stop();
            ecbs->conflict_num_time += conflict_num_timer.elapsedSeconds();
            //std::cout << cur_i << std::endl;
            (*solvers)[(solver->_id + cur_i) % solvers->size()]->add_work(new_node);
            cur_i ++;

        

        
        }
    }
    return;

}



int ECBS::paralized_solver_main(){
    
    //make root node for search
    shared_ptr<ECBSNode> start_node(new ECBSNode());
    start_node->create_cost_matrix(this);
    start_node->focal_score = high_focal_score_v4(start_node->cost_matrix, start_node->first_conflict);

    shared_ptr<Solver> first_solver = shared_ptr<Solver>(new Solver());
    first_solver->add_work(start_node);
    shared_ptr<std::vector<shared_ptr<Solver>>> solvers = shared_ptr<std::vector<shared_ptr<Solver>>>(new std::vector<shared_ptr<Solver>>());
    solvers->push_back(first_solver);


    int numThreads = this->NUM_THREADS;
    

    this->m.lock();
    first_solver->_thread = std::make_shared<std::thread>(std::thread(solver_thread, this, first_solver, solvers));
    first_solver->_id = 0;
    for (int i=1; i < numThreads; i++){
        shared_ptr<Solver> new_solver = shared_ptr<Solver>(new Solver());
        new_solver->_thread = std::make_shared<std::thread>(std::thread(solver_thread, this, new_solver, solvers));
        new_solver->_id = i;
        solvers->push_back(new_solver);
    }
    this->m.unlock();

    //just stall until found a solution or open is empty and all the threads are waiting
    while(true){
        this->m.lock();
        if (this->solution_found){
            this->m.unlock();
            std::cout << "done; cost: " << this->cost << std::endl;
            for (auto& solver : *solvers){
                solver->add_work(nullptr); // thread dies when sees this

            }
            for (auto& solver : *solvers){
                solver->_thread->join();  //join with all the threads created

            }
            
            return true;
        }
        this->m.unlock();
        usleep(500);
    }

}

//replace lb heap with a function that finds the lb based on all open lists



//create thread class
//contains open, focal,  CTnode2open_handle, buffer

