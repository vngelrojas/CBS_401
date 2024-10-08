#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "ECBS.hpp"
#include "../common.hpp"

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

int main(int argc, char** argv) {

    if (init_map(argc, argv) < 0)
    {
        std::cout<< "Error Map" <<std::endl;
        return 0;
    }
    std::cout<< "Load Map Done" <<std::endl;
    ECBS ecbs(row_number, col_number, obstacles, goals, start_states, vm["weight"].as<float>());

    int runs = vm["restart"].as<int>();
    for (int i = 0; i < runs; i++) {
        ecbs.clear();
        ecbs.total_timer.reset();
        ecbs.solve();
        ecbs.total_timer.stop();
        ecbs.total_runtime = ecbs.total_timer.elapsedSeconds();
        if (ecbs.solution_found) break;
    }
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << ecbs.cost << std::endl;
    out << "  newnode_runtime: " << ecbs.newnode_time << std::endl;
    out << "  focal_score_time: " << ecbs.focal_score_time << std::endl;
    out << "  firstconflict_runtime: " << ecbs.conflict_num_time << std::endl;
    out << "  runtime: " << ecbs.total_runtime << std::endl;
    out << "  lowlevel_search_time: " << ecbs.lowlevel_search_time << std::endl;
    out << "  total_lowlevel_node: " << ecbs.cbsnode_num << std::endl;
    out << "  lowLevelExpanded: " << ecbs.lowLevelExpanded << std::endl;
    out << "  numTaskAssignments: " << ecbs.num_ta << std::endl;

    out << "schedule:" << std::endl;
    for (size_t a = 0; a < ecbs.out_solution.size(); ++a) {
        out << "  agent" << a << ":" << std::endl;
        for (const auto &state: *(ecbs.out_solution[a])) {
            out << "    - x: " << state.state.x << std::endl
                << "      y: " << state.state.y << std::endl
                << "      t: " << state.state.time << std::endl;
        }
    }
    return 0;
}
