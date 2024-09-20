#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "CBS.hpp"
#include "../common.hpp"

namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<Location> goals;
vector<State> start_states;
po::variables_map vm; // usedd to store the values of command-line options after they have been parsed.
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

    if (init_map(argc, argv) < 0)
    {
        std::cout<< "Error Map" <<std::endl;
        return 0;
    }
    std::cout<< "Load Map Done" <<std::endl;
    CBS cbs(row_number, col_number, obstacles, goals, start_states);

    int runs = vm["restart"].as<int>();
    for (int i = 0; i < runs; i++) {
        cbs.clear();
        cbs.total_timer.reset();
        cbs.solve();
        cbs.total_timer.stop();
        cbs.total_runtime = cbs.total_timer.elapsedSeconds();
        if (cbs.solution_found) break;
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
