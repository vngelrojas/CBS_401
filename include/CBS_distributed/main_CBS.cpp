#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "CBS.hpp"
#include "../common.hpp"
#include <mpi.h>

namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<Location> goals;
vector<State> start_states;
po::variables_map vm; // usedd to store the values of command-line options after they have been parsed.
int row_number,col_number;
string outputFile;

MPI_Datatype MPI_Location;

int init_MPI_Location() {
    int lengths[2] = {1, 1};
    MPI_Aint displacements[2];
    Location loc;
    MPI_Aint base_address;
    MPI_Get_address(&loc, &base_address);
    MPI_Get_address(&loc.x, &displacements[0]);
    MPI_Get_address(&loc.y, &displacements[1]);
    displacements[0] -= base_address;
    displacements[1] -= base_address;
    MPI_Datatype types[2] = {MPI_INT, MPI_INT};
    MPI_Type_create_struct(2, lengths, displacements, types, &MPI_Location);
    MPI_Type_commit(&MPI_Location);
    return 0;
}

MPI_Datatype MPI_State;

int init_MPI_State() {
    int state_lengths[3] = {1, 1, 1};
    MPI_Aint state_displacements[3];
    State state;
    MPI_Aint state_base_address;
    MPI_Get_address(&state, &state_base_address);
    MPI_Get_address(&state.time, &state_displacements[0]);
    MPI_Get_address(&state.x, &state_displacements[1]);
    MPI_Get_address(&state.y, &state_displacements[2]);
    state_displacements[0] -= state_base_address;
    state_displacements[1] -= state_base_address;
    state_displacements[2] -= state_base_address;
    MPI_Datatype state_types[3] = {MPI_INT, MPI_INT, MPI_INT};
    MPI_Type_create_struct(3, state_lengths, state_displacements, state_types, &MPI_State);
    MPI_Type_commit(&MPI_State);
    return 0;
}

void send_data(int row_number, int col_number, std::unordered_set<Location>& obstacles,
               std::vector<Location>& goals, std::vector<State>& start_states, 
               int max_nodes, int world_rank, int dest, int tag) {
    
    // Send basic integers
    MPI_Send(&row_number, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(&col_number, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(&max_nodes, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(&world_rank, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);

    // Send obstacle set
    std::vector<Location> obstacle_list(obstacles.begin(), obstacles.end());
    int num_obstacles = obstacle_list.size();
    MPI_Send(&num_obstacles, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(obstacle_list.data(), num_obstacles, MPI_Location, dest, tag, MPI_COMM_WORLD);

    // Send goals vector
    int num_goals = goals.size();
    MPI_Send(&num_goals, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(goals.data(), num_goals, MPI_Location, dest, tag, MPI_COMM_WORLD);

    // Send start_states vector
    int num_start_states = start_states.size();
    MPI_Send(&num_start_states, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
    MPI_Send(start_states.data(), num_start_states, MPI_State, dest, tag, MPI_COMM_WORLD);
}





int init_map(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    string inputFile;
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
            ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
            ("nodeLimit", po::value<int>()->default_value(MAX_NODES), "node limit")
            ("seed,d", po::value<int>()->default_value(0), "random seed")
            ("stats", po::value<bool>()->default_value(false), "write to files some statistics")
            ("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)");

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
            std::cout << desc << "\n";
            return -1;
        }
    } catch (po::error &e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    YAML::Node config = YAML::LoadFile(inputFile);

    if (config["map"].IsMap()) {
        const auto &dim = config["map"]["dimensions"];
        row_number = dim[0].as<int>();
        col_number = dim[1].as<int>();
        for (const auto &node: config["map"]["obstacles"]) {
            obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
        }
    }
    else {
        const auto& file_name = config["map"].as<string>();
        std::filesystem::path fullPath(inputFile);
        std::filesystem::path folderPath = fullPath.parent_path();
        std::filesystem::path map_file_path = folderPath / file_name;
        cout<<map_file_path<<endl;
        vector<vector<bool> > ret_map;
        read_map_file(map_file_path, ret_map);
        row_number = ret_map.size();
        col_number = ret_map[0].size();
        for (int i=0;i<row_number;i++)
            for (int j=0;j<col_number;j++)
                if (ret_map[i][j])
                    obstacles.insert(Location(i, j));
    }

    for (const auto &node: config["agents"]) {
        const auto &start = node["start"];
        start_states.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
        if (node["potentialGoals"]) {
            if (node["potentialGoals"].size() > 1) printf("[ERROR] Multiple Goal locations for one agent!\n");
            for (const auto &goal: node["potentialGoals"]) {
                Location x = Location(goal[0].as<int>(), goal[1].as<int>());
                goals.emplace_back(x);
                break;
            }
        }
        else {
            auto goal = node["goal"];
            Location x = Location(goal[0].as<int>(), goal[1].as<int>());
            goals.emplace_back(x);
        }
    }
    return 0;
}



int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);

    int world_rank, world_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);

    if(world_rank == 0) // 0 is master 
    {
        if (init_map(argc, argv) < 0)
        {
            std::cout<< "Error Map" <<std::endl;
            return 0;
        }
        std::cout<< "Load Map Done" <<std::endl;
        CBS cbs(row_number, col_number, obstacles, goals, start_states,world_size-1,world_rank);

        // Will return once max_nodes have been generated
        cbs.solve();

        // distribute nodes to other workers
        int target_rank = 1;
        for (auto node : cbs.nodes_to_distribute) {
            if(target_rank >= world_size) break;
            //MPI_Send(&node, sizeof(Node), MPI_BYTE, target_rank, 0, MPI_COMM_WORLD);
            target_rank++;
        }

        // Wait for a worker to finish and send the solution info
        //MPI_Recv();
        // Print the solution

    }
    else // worker 
    {   
        // Wait for node from master, create cbs object and solve
        //MPI_Recv();
        //CBS cbs(info from master);
        //csb.solve(); This solve functtion will broadcast to all workers if it finds a solution, it will then send solution to master
    }
    

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cbs.cost << std::endl;
    out << "  firstconflict_runtime: " << cbs.firstconflict_time << std::endl;
    out << "  runtime: " << cbs.total_runtime << std::endl;
    out << "  lowlevel_search_time: " << cbs.lowlevel_search_time << std::endl;
    out << "  total_lowlevel_node: " << cbs.cbsnode_num << std::endl;
    out << "  lowLevelExpanded: " << cbs.lowLevelExpanded << std::endl;
    out << "  numTaskAssignments: " << cbs.num_ta << std::endl;

    out << "schedule:" << std::endl;
    for (size_t a = 0; a < cbs.out_solution.size(); ++a) {
        out << "  agent" << a << ":" << std::endl;
        for (const auto &state: *(cbs.out_solution[a])) {
            out << "    - x: " << state.state.x << std::endl
                << "      y: " << state.state.y << std::endl
                << "      t: " << state.state.time << std::endl;
        }
    }
    return 0;
}
