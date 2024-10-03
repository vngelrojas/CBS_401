//
// Created by YIMIN TANG on 3/19/23.
//

#ifndef CBS_REMAKE_CBS_HPP
#define CBS_REMAKE_CBS_HPP

#include "../common.hpp"
#include "CBSNode.hpp"

class CBS
{

public:

    bool solution_found{false};
    int num_of_agents;
    int max_nodes;
    int world_rank;
    vector<shared_ptr<CBSNode>> nodes_to_distribute;
    vector<State> start_states;
    unordered_set<Location> obstacles;
    vector<Location> goals;
    vector<vector<int> > map2d_obstacle;
    unordered_map<int, vector<vector<int> > > prior_hmap;
    unordered_set<State, boost::hash<State> > closedSet;
    vector<shared_ptr<Path > > out_solution;
    vector<shared_ptr<Path > > first_cost_matrix;
    int cost, map_size, cbsnode_num, lowLevelExpanded, num_ta;
    int row_number, col_number;

    vector<shared_ptr<Constraints > > constraint_sets;

    Timer total_timer, lowlevel_search_timer, firstconflict_timer;
    double total_runtime,  lowlevel_search_time, firstconflict_time;
    CBS(int row_number, int col_number, unordered_set<Location>& obstacles,
        vector<Location>& goals, vector<State>& startStates,int max_nodes,int world_rank);
    void clear();
    ~CBS();
   
    int solve(shared_ptr<CBSNode> root_node = nullptr);
    int heuristic(int x1, int y1, int agent_idx);
    bool searchNodeIsValid(shared_ptr<Constraints>& agent_constraint_set, const State& new_state, const State& org_state);

    shared_ptr<Path> findPath_a_star(shared_ptr<Constraints>&  agent_constraint_set, int agent_idx);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            openSet_t;
    using PathEntryHandle = openSet_t::handle_type;
};

#endif //CBS_REMAKE_CBS_HPP
