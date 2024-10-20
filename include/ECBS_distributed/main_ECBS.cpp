#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "ECBS.hpp"
#include "ECBSNode.hpp"
#include "../common.hpp"
//distributed 
#include "/workspaces/CBS_401/protos/ecbs_node.pb.h"
#include <mpi.h>
// #include "../../protos/cbs.pb.h"
#include "/workspaces/CBS_401/protos/ecbs.pb.h"

namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<Location> goals;
vector<State> start_states;
po::variables_map vm;
int row_number,col_number;
string outputFile;

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
            ("weight,w", po::value<float>()->default_value(1), "weight for ECBS")
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

// int main(int argc, char** argv) {

//     if (init_map(argc, argv) < 0)
//     {
//         std::cout<< "Error Map" <<std::endl;
//         return 0;
//     }
//     std::cout<< "Load Map Done" <<std::endl;
//     ECBS ecbs(row_number, col_number, obstacles, goals, start_states, vm["weight"].as<float>());

//     int runs = vm["restart"].as<int>();
//     for (int i = 0; i < runs; i++) {
//         ecbs.clear();
//         ecbs.total_timer.reset();
//         ecbs.solve();
//         ecbs.total_timer.stop();
//         ecbs.total_runtime = ecbs.total_timer.elapsedSeconds();
//         if (ecbs.solution_found) break;
//     }
//     std::ofstream out(outputFile);
//     out << "statistics:" << std::endl;
//     out << "  cost: " << ecbs.cost << std::endl;
//     out << "  newnode_runtime: " << ecbs.newnode_time << std::endl;
//     out << "  focal_score_time: " << ecbs.focal_score_time << std::endl;
//     out << "  firstconflict_runtime: " << ecbs.conflict_num_time << std::endl;
//     out << "  runtime: " << ecbs.total_runtime << std::endl;
//     out << "  lowlevel_search_time: " << ecbs.lowlevel_search_time << std::endl;
//     out << "  total_lowlevel_node: " << ecbs.cbsnode_num << std::endl;
//     out << "  lowLevelExpanded: " << ecbs.lowLevelExpanded << std::endl;
//     out << "  numTaskAssignments: " << ecbs.num_ta << std::endl;

//     out << "schedule:" << std::endl;
//     for (size_t a = 0; a < ecbs.out_solution.size(); ++a) {
//         out << "  agent" << a << ":" << std::endl;
//         for (const auto &state: *(ecbs.out_solution[a])) {
//             out << "    - x: " << state.state.x << std::endl
//                 << "      y: " << state.state.y << std::endl
//                 << "      t: " << state.state.time << std::endl;
//         }
//     }
//     return 0;
// }

ECBSNode initializeECBSNode() {
    // Create a new ECBSNode object
    ECBSNode ecbs_node;

    // Set the cost to a random value
    ecbs_node.cost = 150; // Example cost value
    ecbs_node.LB = 0.432;
    ecbs_node.focal_score = 32;
    // Initialize the cost_matrix with hardcoded data
    // Create a Path and add some PathEntries
    Path path;
    
    // Adding random PathEntries to the Path
    for (int i = 0; i < 3; i++) {  // Let's assume 3 path entries
        State state(i * 10, i + 1, i + 2);  // Random values for state (time, x, y)
        PathEntry path_entry(state, 50 + i, 25 + i, 30 + i, nullptr);
        path.push_back(path_entry);  // Adding to the path
    }

    // Adding the path to the cost_matrix
    ecbs_node.cost_matrix.push_back(boost::make_shared<Path>(path)); // Using boost::make_shared

    // Initialize the constraint_sets with hardcoded data
    Constraints constraints;

    // Add VertexConstraint
    constraints.vertexConstraints.insert(VertexConstraint(5, 1, 2));  // Random values
    constraints.vertexConstraints.insert(VertexConstraint(10, 3, 4, 1));  // Random values with for_who = 1

    // Add EdgeConstraint
    constraints.edgeConstraints.insert(EdgeConstraint(7, 1, 2, 3, 4));  // Random values
    constraints.edgeConstraints.insert(EdgeConstraint(15, 5, 6, 7, 8, 2));  // Random values with for_who = 2

    // Add constraints to constraint_sets
    ecbs_node.constraint_sets.push_back(boost::make_shared<Constraints>(constraints)); // Using boost::make_shared

    return ecbs_node;
}

ECBSProto::ECBS serializeToProtobuf(ECBS& cbs) {
   ECBSProto::ECBS proto_cbs;

    // Set basic fields
    proto_cbs.set_num_of_rows(cbs.row_number);
    proto_cbs.set_num_of_cols(cbs.col_number);
    proto_cbs.set_world_size(cbs.max_nodes);
    proto_cbs.set_world_rank(cbs.world_rank);
    proto_cbs.set_l_weight(cbs.l_weight);
    // Set obstacles
    for (const auto& loc : cbs.obstacles) {
        auto* proto_loc = proto_cbs.add_obstacles();
        proto_loc->set_x(loc.x);
        proto_loc->set_y(loc.y);
    }

    // Set goals
    for (const auto& goal : cbs.goals) {
        auto* proto_goal = proto_cbs.add_goals();
        proto_goal->set_x(goal.x);
        proto_goal->set_y(goal.y);
    }

    // Set start_states
    for (const auto& state : cbs.start_states) {
        auto* proto_state = proto_cbs.add_start_states();
        proto_state->set_time(state.time);
        proto_state->set_x(state.x);
        proto_state->set_y(state.y);
    }

    return proto_cbs;
}

void convertProtoToECBSNode(const ECBSProto::ECBSNode& proto_cbs_node, ECBSNode& cbs_node) {
    // Convert path entries
    for (int i = 0; i < proto_cbs_node.cost_matrix_size(); ++i) {
        const auto& proto_path = proto_cbs_node.cost_matrix(i);
        auto path = boost::make_shared<Path>();
        for (int j = 0; j < proto_path.path_size(); ++j) {
            const ECBSProto::ECBSNode::PathEntry& proto_entry = proto_path.path(j);
            PathEntry entry;
            entry.state.time = proto_entry.state().time();
            entry.state.x = proto_entry.state().x();
            entry.state.y = proto_entry.state().y();
            entry.fScore = proto_entry.fscore();
            entry.gScore = proto_entry.gscore();
            entry.focalScore = proto_entry.focalscore();
            path->push_back(entry);
        }
        cbs_node.cost_matrix.push_back(path);
    }

    // Convert constraint sets
    for (int i = 0; i < proto_cbs_node.constraint_sets_size(); ++i) {
        const ECBSProto::ECBSNode::Constraints& proto_constraint_set = proto_cbs_node.constraint_sets(i);
        Constraints constraint_set;

        // Convert vertex constraints
        for (int j = 0; j < proto_constraint_set.vertex_constraints_size(); ++j) {
            const ECBSProto::ECBSNode::Constraints::VertexConstraint& proto_vc = proto_constraint_set.vertex_constraints(j);
            VertexConstraint vc(proto_vc.time(), proto_vc.x(), proto_vc.y(), proto_vc.for_who());
            constraint_set.vertexConstraints.insert(vc);
        }

        // Convert edge constraints
        for (int j = 0; j < proto_constraint_set.edge_constraints_size(); ++j) {
            const ECBSProto::ECBSNode::Constraints::EdgeConstraint& proto_ec = proto_constraint_set.edge_constraints(j);
            EdgeConstraint ec(proto_ec.time(),  proto_ec.x1(), proto_ec.y1(), proto_ec.x2(), proto_ec.y2(), proto_ec.for_who());
            constraint_set.edgeConstraints.insert(ec);
        }

        cbs_node.constraint_sets.push_back(boost::make_shared<Constraints>(constraint_set));
    }
}


//code to test that the ecbs serializer works and can be sent over network
int main(int argc, char** argv)
{
    MPI_Init(&argc, &argv);

     int world_rank, world_size;
     MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
     MPI_Comm_size(MPI_COMM_WORLD, &world_size);
     cout << "World size: " << world_size << endl; // Add this line

    //  if (init_map(argc, argv) < 0)
    //  {
    //       std::cout<< "Error Map" <<std::endl;
    //       MPI_Finalize();
    //       return 0;
    //  }
     std::cout<< "Load Map Done" <<std::endl;
     if(world_rank == 0)
     {
            ECBS cbs(row_number, col_number, obstacles, goals, start_states,0.324, world_size-1,world_rank);
            cbs.cost = 231;
            
            std::cout << "solution_found: " << cbs.solution_found << std::endl;
            std::cout << "num_of_agents: " << cbs.agent_n << std::endl;
            std::cout << "max_nodes: " << cbs.max_nodes << std::endl;
            std::cout << "world_rank: " << cbs.world_rank << std::endl;
            std::cout << "cost: " << cbs.cost << std::endl;
            std::cout << "map_size: " << cbs.map_size << std::endl;
            std::cout << "cbsnode_num: " << cbs.cbsnode_num << std::endl;
            std::cout << "lowLevelExpanded: " << cbs.lowLevelExpanded << std::endl;
            std::cout << "num_ta: " << cbs.num_ta << std::endl;
            std::cout << "row_number: " << cbs.row_number << std::endl;
            std::cout << "col_number: " << cbs.col_number << std::endl;
            std::cout << "total_runtime: " << cbs.total_runtime << std::endl;
            std::cout << "lowlevel_search_time: " << cbs.lowlevel_search_time << std::endl;
            std::cout << "firstconflict_time: " << cbs.firstconflict_time << std::endl;
            auto proto_cbs = serializeToProtobuf(cbs);

            std::string serialized_cbs;
            proto_cbs.SerializeToString(&serialized_cbs);
            cout << "Serialized CBS size: " << serialized_cbs.size() << endl;
            int size = serialized_cbs.size();
            for (int target_rank = 1; target_rank < world_size; ++target_rank) {
                    MPI_Send(&size, 1, MPI_INT, target_rank, 0, MPI_COMM_WORLD);
                    MPI_Send(serialized_cbs.c_str(), size, MPI_CHAR, target_rank, 0, MPI_COMM_WORLD);
            }

            // ECBSNode ecbs_node = initializeECBSNode();
            // cout << "Original CBS Node:" << endl;
            // printECBSNode(ecbs_node);
            // auto proto_ecbs_node = serializeToProtobuf(ecbs_node);
            
            // std::string serialized_ecbs_node;
            // proto_ecbs_node.SerializeToString(&serialized_ecbs_node);
            // cout << "Serialized CBS Node size: " << serialized_ecbs_node.size() << endl;
            // int node_size = serialized_ecbs_node.size();
            // for (int target_rank = 1; target_rank < world_size; ++target_rank) {

            //     MPI_Send(&node_size, 1, MPI_INT, target_rank, 0, MPI_COMM_WORLD);
            //     MPI_Send(serialized_ecbs_node.c_str(), node_size, MPI_CHAR, target_rank, 0, MPI_COMM_WORLD);
            // }
            // cout << "Complete Send" << endl;
     }
     else
     {  
            // Receive the size of the serialized protobuf message
            int size;
            MPI_Recv(&size, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            // Receive the serialized protobuf message
            std::vector<char> buffer(size);
            MPI_Recv(buffer.data(), size, MPI_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            // ECBSProto::ECBSNode proto_ecbs_node;
            // if (!proto_ecbs_node.ParseFromArray(buffer.data(), size)) {
            //     std::cerr << "Failed to parse CBS protobuf message." << std::endl;
            //     MPI_Finalize();
            //     return -1;
            // }
           
            // cout << "Resulting CBS Node:" << endl;
            // ECBSNode ecbs_node;
            // convertProtoToECBSNode(proto_ecbs_node, ecbs_node);
            // printECBSNode(ecbs_node);

          // Deserialize the protobuf message
          ECBSProto::ECBS proto_cbs;
          if (!proto_cbs.ParseFromArray(buffer.data(), size)) {
                std::cerr << "Failed to parse CBS protobuf message." << std::endl;
                MPI_Finalize();
                return -1;
          }

          // Convert the protobuf message to a CBS object
        cout << "!!!!!!!!!!!!!Received CBS object from master.!!!!!!!!!!!!!!!!!!" << endl;
        ECBS cbs(row_number, col_number, obstacles, goals, start_states, 0.123, world_size - 1, world_rank);
        cbs.row_number = proto_cbs.num_of_rows();
        cbs.col_number = proto_cbs.num_of_cols();
        cbs.max_nodes = proto_cbs.world_size();
        cbs.world_rank = proto_cbs.world_rank();
        cbs.l_weight = proto_cbs.l_weight();
        cbs.cost = proto_cbs.cost();
        cbs.obstacles.clear();
        for (int i = 0; i < proto_cbs.obstacles_size(); ++i) {
            const ECBSProto::ECBS::Location& proto_loc = proto_cbs.obstacles(i);
            cbs.obstacles.insert(Location(proto_loc.x(), proto_loc.y()));
        }

        cbs.goals.clear();
        for (int i = 0; i < proto_cbs.goals_size(); ++i) {
            const ECBSProto::ECBS::Location& proto_goal = proto_cbs.goals(i);
            cbs.goals.push_back(Location(proto_goal.x(), proto_goal.y()));
        }

        cbs.start_states.clear();
        for (int i = 0; i < proto_cbs.start_states_size(); ++i) {
            const ECBSProto::ECBS::State& proto_state = proto_cbs.start_states(i);
            cbs.start_states.push_back(State(proto_state.time(), proto_state.x(), proto_state.y()));
        }

        std::cout << "Successfully received and deserialized CBS object from master." << std::endl;
        std::cout << "solution_found: " << cbs.solution_found << std::endl;
        std::cout << "num_of_agents: " << cbs.agent_n << std::endl;
        std::cout << "max_nodes: " << cbs.max_nodes << std::endl;
        std::cout << "world_rank: " << cbs.world_rank << std::endl;
        std::cout << "cost: " << cbs.cost << std::endl;
        std::cout << "map_size: " << cbs.map_size << std::endl;
        std::cout << "cbsnode_num: " << cbs.cbsnode_num << std::endl;
        std::cout << "lowLevelExpanded: " << cbs.lowLevelExpanded << std::endl;
        std::cout << "num_ta: " << cbs.num_ta << std::endl;
        std::cout << "row_number: " << cbs.row_number << std::endl;
        std::cout << "col_number: " << cbs.col_number << std::endl;
        std::cout << "total_runtime: " << cbs.total_runtime << std::endl;
        std::cout << "lowlevel_search_time: " << cbs.lowlevel_search_time << std::endl;
        std::cout << "firstconflict_time: " << cbs.firstconflict_time << std::endl;
        
     }

     MPI_Finalize();
     return 0;
}