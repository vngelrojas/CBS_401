//
// Created by YIMIN TANG on 3/19/23.
//

#ifndef ECBS_REMAKE_ECBS_HPP
#define ECBS_REMAKE_ECBS_HPP

#include "../common.hpp"

class ECBS
{

public:


    bool solution_found;
    int agent_n;
    vector<State> start_states;
    unordered_set<Location> obstacles;
    vector<Location> goals;
    vector<vector<int> > map2d_obstacle;
    unordered_map<int, vector<vector<int> > > prior_hmap;
    vector<shared_ptr<Path > > out_solution;
    int cost, map_size, cbsnode_num, lowLevelExpanded, num_ta;
    int row_number, col_number;
    float l_weight;


    Timer total_timer, lowlevel_search_timer, firstconflict_timer, focal_score_timer, newnode_timer, conflict_num_timer;
    double total_runtime,  lowlevel_search_time, firstconflict_time, focal_score_time, newnode_time, conflict_num_time;

    ~ECBS();
    ECBS(int row_number, int col_number, unordered_set<Location>& obstacles,
           vector<Location>& goals, vector<State>& startStates, float l_weight);
    void clear();
    int solve();
    int heuristic(int x1, int y1, int agent_idx);
    bool searchNodeIsValid(shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state, unordered_set<State, boost::hash<State> > *closedSet);

    shared_ptr<Path> findPath_a_star_eps(vector<shared_ptr<Path > >& cost_matrix, shared_ptr<Constraints>& agent_constraint_set, int agent_idx, vector<int >& fmin);
    shared_ptr<Path> findPath_a_star_eps2(vector<shared_ptr<Path > >& cost_matrix, shared_ptr<Constraints>& agent_constraint_set, int agent_idx, vector<int >& fmin);
    shared_ptr<Path> findPath_a_star(shared_ptr<Constraints>&  agent_constraint_set, int agent_idx, vector<int >& fmin);
    int high_focal_score(const vector<shared_ptr<Path > >& out_solution, Conflict& result);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            low_openSet_t;
    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare2> >
            low_openSet_t2;

    typedef typename low_openSet_t::handle_type PathEntryHandle;
    typedef typename low_openSet_t2::handle_type PathEntryHandle2;

    struct FocalPathEntryCompare{
        bool operator()(const PathEntryHandle& a, const PathEntryHandle& b) const {
            if ((*a)->focalScore != (*b)->focalScore) return (*a)->focalScore > (*b)->focalScore;
            else {
                if ((*a)->fScore != (*b)->fScore) return (*a)->fScore > (*b)->fScore;
            }
            return (*a)->gScore < (*b)->gScore;
        }
    };

    typedef typename boost::heap::d_ary_heap<
            PathEntryHandle, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<FocalPathEntryCompare> >
            low_focalSet_t;
    using FocalPathEntryHandle = low_focalSet_t::handle_type;

    
    int paralized_solver_main();
    std::mutex m;
    std::condition_variable cv;
    int waiting_nodes = 0;

};

#endif //ECBS_REMAKE_ECBS_HPP
