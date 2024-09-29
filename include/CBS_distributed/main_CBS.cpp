#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "CBS.hpp"
#include "../common.hpp"
#include "CBSNode.hpp"
#include "/workspaces/CBS_401/protos/cbs_node.pb.h"
#include <mpi.h>
// #include "../../protos/cbs.pb.h"
#include "/workspaces/CBS_401/protos/cbs.pb.h"
namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<Location> goals;
vector<State> start_states;
po::variables_map vm; // usedd to store the values of command-line options after they have been parsed.
int row_number,col_number;
string outputFile;


// Helper function to deserialize a single PathEntry
// PathEntry deserializePathEntry(const CBSProto::CBSNode::PathEntry& proto_entry) {
//     State state(proto_entry.state().time(), proto_entry.state().x(), proto_entry.state().y());
//     shared_ptr<PathEntry> parent;

//     if (proto_entry.has_parent()) {
//         parent = make_shared<PathEntry>(deserializePathEntry(proto_entry.parent()));
//     }

//     PathEntry entry(state, proto_entry.fscore(), proto_entry.gscore(), proto_entry.focalscore(), parent);
//     return entry;
// }

CBSProto::CBS serializeToProtobuf(CBS& cbs) {
   CBSProto::CBS proto_cbs;

    // Set basic fields
    proto_cbs.set_num_of_rows(cbs.row_number);
    proto_cbs.set_num_of_cols(cbs.col_number);
    proto_cbs.set_world_size(cbs.max_nodes);
    proto_cbs.set_world_rank(cbs.world_rank);

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

// void sendCbsToWorkers(CBS& cbs, int world_size) {
//     CBSProto::CBS proto_cbs = serializeToProtobuf(cbs);
    
//     // Serialize the protobuf message to a string
//     std::string serialized_cbs;
//     proto_cbs.SerializeToString(&serialized_cbs);

//     // Send the serialized string to all workers
//     for (int target_rank = 1; target_rank < world_size; ++target_rank) {
//         // Send the size of the serialized data first
//         int size = serialized_cbs.size();
//         MPI_Send(&size, 1, MPI_INT, target_rank, 0, MPI_COMM_WORLD);
        
//         // Then send the serialized protobuf data
//         MPI_Send(serialized_cbs.c_str(), size, MPI_CHAR, target_rank, 0, MPI_COMM_WORLD);
//     }
// }

// CBS receiveCbsFromMaster() {
//     CBS cbs(0, 0, obstacles, goals, start_states, 0, 0);
//     // 1. Receive the size of the serialized protobuf data
//     int size;
//     MPI_Recv(&size, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

//     // 2. Receive the serialized protobuf data
//     std::vector<char> buffer(size);
//     MPI_Recv(buffer.data(), size, MPI_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

//     // 3. Deserialize the protobuf message
//     CBSProto::CBS proto_cbs;
//     if (!proto_cbs.ParseFromArray(buffer.data(), size)) {
//         std::cerr << "Failed to parse CBS protobuf message." << std::endl;
//         return;
//     }

//     // 4. Convert the protobuf message to the CBS object

//     // Set basic fields
//     cbs.row_number = proto_cbs.num_of_rows();
//     cbs.col_number = proto_cbs.num_of_cols();
//     cbs.max_nodes = proto_cbs.world_size();
//     cbs.world_rank = proto_cbs.world_rank();

//     // Clear the existing data in the CBS object to prevent duplication
//     cbs.obstacles.clear();
//     cbs.goals.clear();
//     cbs.start_states.clear();

//     // Set obstacles
//     for (int i = 0; i < proto_cbs.obstacles_size(); ++i) {
//         const CBSProto::CBS::Location& proto_loc = proto_cbs.obstacles(i);
//         cbs.obstacles.insert(Location(proto_loc.x(), proto_loc.y()));
//     }

//     // Set goals
//     for (int i = 0; i < proto_cbs.goals_size(); ++i) {
//         const CBSProto::CBS::Location& proto_goal = proto_cbs.goals(i);
//         cbs.goals.push_back(Location(proto_goal.x(), proto_goal.y()));
//     }

//     // Set start states
//     for (int i = 0; i < proto_cbs.start_states_size(); ++i) {
//         const CBSProto::CBS::State& proto_state = proto_cbs.start_states(i);
//         cbs.start_states.push_back(State(proto_state.time(), proto_state.x(), proto_state.y()));
//     }

//     std::cout << "Successfully received and deserialized CBS object from master." << std::endl;
// }

// CBSProto::CBSNode serializeCbsNodeToProtobuf(const CBSNode& node) {
//     CBSProto::CBSNode proto_node;

//     // Serialize the cost matrix (paths)
//     for (const auto& path_ptr : node.cost_matrix) {
//         CBSProto::CBSNode::Path* proto_path = proto_node.add_cost_matrix();
//         for (const auto& path_entry : *path_ptr) {
//             CBSProto::CBSNode::PathEntry* proto_entry = proto_path->add_path();
//             // Convert each PathEntry state to protobuf
//             proto_entry->mutable_state()->set_time(path_entry.state.time);
//             proto_entry->mutable_state()->set_x(path_entry.state.x);
//             proto_entry->mutable_state()->set_y(path_entry.state.y);
//             proto_entry->set_fscore(path_entry.fScore);
//             proto_entry->set_gscore(path_entry.gScore);
//             proto_entry->set_focalscore(path_entry.focalScore);

//             // Parent is a shared pointer, convert recursively (if not null)
//             if (path_entry.parent) {
//                 auto* proto_parent = new CBSProto::CBSNode::PathEntry();
//                 proto_parent->mutable_state()->set_time(path_entry.parent->state.time);
//                 proto_parent->mutable_state()->set_x(path_entry.parent->state.x);
//                 proto_parent->mutable_state()->set_y(path_entry.parent->state.y);
//                 proto_entry->set_allocated_parent(proto_parent);
//             }
//         }
//     }

//     // Serialize the constraint sets (vertex and edge constraints)
//     for (const auto& constraints_ptr : node.constraint_sets) {
//         CBSProto::CBSNode::Constraints* proto_constraints = proto_node.add_constraint_sets();

//         // Vertex constraints
//         for (const auto& vertex_constraint : constraints_ptr->vertexConstraints) {
//             CBSProto::CBSNode::Constraints::VertexConstraint* proto_vc = proto_constraints->add_vertex_constraints();
//             proto_vc->set_time(vertex_constraint.time);
//             proto_vc->set_x(vertex_constraint.x);
//             proto_vc->set_y(vertex_constraint.y);
//             proto_vc->set_for_who(vertex_constraint.for_who);
//         }

//         // Edge constraints
//         for (const auto& edge_constraint : constraints_ptr->edgeConstraints) {
//             CBSProto::CBSNode::Constraints::EdgeConstraint* proto_ec = proto_constraints->add_edge_constraints();
//             proto_ec->set_time(edge_constraint.time);
//             proto_ec->set_x1(edge_constraint.x1);
//             proto_ec->set_y1(edge_constraint.y1);
//             proto_ec->set_x2(edge_constraint.x2);
//             proto_ec->set_y2(edge_constraint.y2);
//             proto_ec->set_for_who(edge_constraint.for_who);
//         }
//     }

//     return proto_node;
// }

// void sendCbsNodeToWorkers(CBSNode& node, int target_rank) {
//     // Serialize the CBSNode to a protobuf message
//     CBSProto::CBSNode proto_node = serializeCbsNodeToProtobuf(node);
    
//     // Serialize the protobuf message to a string
//     std::string serialized_node;
//     proto_node.SerializeToString(&serialized_node);

//     // Send the serialized string 
//     // Send the size of the serialized data first
//     int size = serialized_node.size();
//     MPI_Send(&size, 1, MPI_INT, target_rank, 0, MPI_COMM_WORLD);
    
//     // Then send the serialized protobuf data
//     MPI_Send(serialized_node.c_str(), size, MPI_CHAR, target_rank, 0, MPI_COMM_WORLD);
    
// }

// CBSNode deserializeCbsNodeFromProtobuf(const CBSProto::CBSNode& proto_node) {
//     CBSNode node;

//     // Deserialize cost matrix (paths)
//     for (const auto& proto_path : proto_node.cost_matrix()) {
//         boost::shared_ptr<Path> path = boost::make_shared<Path>();
        
//         // Deserialize each PathEntry
//         for (const auto& proto_entry : proto_path.path()) {
//             shared_ptr<PathEntry> entry = boost::make_shared<PathEntry>();
//             entry->state.time = proto_entry.state().time();
//             entry->state.x = proto_entry.state().x();
//             entry->state.y = proto_entry.state().y();
//             entry->fScore = proto_entry.fscore();
//             entry->gScore = proto_entry.gscore();
//             entry->focalScore = proto_entry.focalscore();

//             // Handle parent pointer (recursively)
//             if (proto_entry.has_parent()) {
//                 entry->parent = boost::make_shared<PathEntry>(deserializePathEntry(proto_entry.parent()));
//             }

//             path->push_back(*entry);
//         }
//         node.cost_matrix.push_back(path);
//     }

//     // Deserialize constraint sets
//     for (const auto& proto_constraints : proto_node.constraint_sets()) {
//         shared_ptr<Constraints> constraints = boost::make_shared<Constraints>();

//         // Deserialize VertexConstraints
//         for (const auto& proto_vertex_constraint : proto_constraints.vertex_constraints()) {
//             VertexConstraint vc(
//                 proto_vertex_constraint.time(),
//                 proto_vertex_constraint.x(),
//                 proto_vertex_constraint.y(),
//                 proto_vertex_constraint.for_who()
//             );
//             constraints->vertexConstraints.insert(vc);
//         }

//         // Deserialize EdgeConstraints
//         for (const auto& proto_edge_constraint : proto_constraints.edge_constraints()) {
//             EdgeConstraint ec(
//                 proto_edge_constraint.time(),
//                 proto_edge_constraint.x1(),
//                 proto_edge_constraint.y1(),
//                 proto_edge_constraint.x2(),
//                 proto_edge_constraint.y2(),
//                 proto_edge_constraint.for_who()
//             );
//             constraints->edgeConstraints.insert(ec);
//         }

//         node.constraint_sets.push_back(constraints);
//     }

    

//     return node;
// }



// void receiveCbsNodeFromMaster(CBSNode& node) {
//     // Receive the size of the serialized protobuf message
//     int size;
//     MPI_Recv(&size, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

//     // Receive the serialized protobuf message
//     std::vector<char> buffer(size);
//     MPI_Recv(buffer.data(), size, MPI_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

//     // Deserialize the protobuf message
//     CBSProto::CBSNode proto_node;
//     if (!proto_node.ParseFromArray(buffer.data(), size)) {
//         std::cerr << "Failed to parse CBSNode protobuf message." << std::endl;
//         return;
//     }

//     // Convert the protobuf message to a CBSNode object
//     node = deserializeCbsNodeFromProtobuf(proto_node);
// }



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

int main(int argc, char** argv)
{
    MPI_Init(0,nullptr);

     int world_rank, world_size;
     MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
     MPI_Comm_size(MPI_COMM_WORLD, &world_size);
     if (init_map(argc, argv) < 0)
     {
          std::cout<< "Error Map" <<std::endl;
          MPI_Finalize();
          return 0;
     }
     std::cout<< "Load Map Done" <<std::endl;
     if(world_rank == 0)
     {
          CBS cbs(row_number, col_number, obstacles, goals, start_states,world_size-1,world_rank);
          std::cout << "solution_found: " << cbs.solution_found << std::endl;
          std::cout << "num_of_agents: " << cbs.num_of_agents << std::endl;
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
     }
     else
     {
          // Receive the size of the serialized protobuf message
          int size;
          MPI_Recv(&size, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

          // Receive the serialized protobuf message
          std::vector<char> buffer(size);
          MPI_Recv(buffer.data(), size, MPI_CHAR, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

          // Deserialize the protobuf message
          CBSProto::CBS proto_cbs;
          if (!proto_cbs.ParseFromArray(buffer.data(), size)) {
                std::cerr << "Failed to parse CBS protobuf message." << std::endl;
                MPI_Finalize();
                return -1;
          }

          // Convert the protobuf message to a CBS object
        cout << "!!!!!!!!!!!!!Received CBS object from master.!!!!!!!!!!!!!!!!!!" << endl;
          CBS cbs(row_number, col_number, obstacles, goals, start_states, world_size - 1, world_rank);
          cbs.row_number = proto_cbs.num_of_rows();
          cbs.col_number = proto_cbs.num_of_cols();
          cbs.max_nodes = proto_cbs.world_size();
          cbs.world_rank = proto_cbs.world_rank();

          cbs.obstacles.clear();
          for (int i = 0; i < proto_cbs.obstacles_size(); ++i) {
                const CBSProto::CBS::Location& proto_loc = proto_cbs.obstacles(i);
                cbs.obstacles.insert(Location(proto_loc.x(), proto_loc.y()));
          }

          cbs.goals.clear();
          for (int i = 0; i < proto_cbs.goals_size(); ++i) {
                const CBSProto::CBS::Location& proto_goal = proto_cbs.goals(i);
                cbs.goals.push_back(Location(proto_goal.x(), proto_goal.y()));
          }

          cbs.start_states.clear();
          for (int i = 0; i < proto_cbs.start_states_size(); ++i) {
                const CBSProto::CBS::State& proto_state = proto_cbs.start_states(i);
                cbs.start_states.push_back(State(proto_state.time(), proto_state.x(), proto_state.y()));
          }

          std::cout << "Successfully received and deserialized CBS object from master." << std::endl;
          std::cout << "solution_found: " << cbs.solution_found << std::endl;
          std::cout << "num_of_agents: " << cbs.num_of_agents << std::endl;
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

/*
 ./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
 mpirun -np 2 ./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
 */
/*
int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);

    int world_rank, world_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    CBS cbs(row_number, col_number, obstacles, goals, start_states,world_size-1,world_rank);
    // if(world_rank == 0) // 0 is master 
    // {
    //     // if (init_map(argc, argv) < 0)
    //     // {
    //     //     std::cout<< "Error Map" <<std::endl;
    //     //     return 0;
    //     // }
    //     // std::cout<< "Load Map Done" <<std::endl;
    //     // CBS cbs(row_number, col_number, obstacles, goals, start_states,world_size-1,world_rank);

    //     // // Will return once max_nodes have been generated
    //     // cbs.solve();

    //     // distribute nodes to other workers
    //     // int target_rank = 1;
    //     // for (auto node : cbs.nodes_to_distribute) {
    //     //     if(target_rank >= world_size) break;
    //     //     //MPI_Send(&node, sizeof(Node), MPI_BYTE, target_rank, 0, MPI_COMM_WORLD);
    //     //     target_rank++;
    //     // }

    //     // Wait for a worker to finish and send the solution info
    //     //MPI_Recv();
    //     // Print the solution

    // }
    // else // worker 
    // {   
    //     // Wait for node from master, create cbs object and solve
    //     //MPI_Recv();
    //     //CBS cbs(info from master);
    //     //csb.solve(); This solve functtion will broadcast to all workers if it finds a solution, it will then send solution to master
    // }
    

    // std::ofstream out(outputFile);
    // out << "statistics:" << std::endl;
    // out << "  cost: " << cbs.cost << std::endl;
    // out << "  firstconflict_runtime: " << cbs.firstconflict_time << std::endl;
    // out << "  runtime: " << cbs.total_runtime << std::endl;
    // out << "  lowlevel_search_time: " << cbs.lowlevel_search_time << std::endl;
    // out << "  total_lowlevel_node: " << cbs.cbsnode_num << std::endl;
    // out << "  lowLevelExpanded: " << cbs.lowLevelExpanded << std::endl;
    // out << "  numTaskAssignments: " << cbs.num_ta << std::endl;

    // out << "schedule:" << std::endl;
    // for (size_t a = 0; a < cbs.out_solution.size(); ++a) {
    //     out << "  agent" << a << ":" << std::endl;
    //     for (const auto &state: *(cbs.out_solution[a])) {
    //         out << "    - x: " << state.state.x << std::endl
    //             << "      y: " << state.state.y << std::endl
    //             << "      t: " << state.state.time << std::endl;
    //     }
    // }
    return 0;
}

*/
