#include <iostream>
#include <cxxopts.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/iterator/stream_iterators.hpp>
#include "path_planner.h"

int main(int argc, char* argv[]) {
  cxxopts::Options options("PathFinder - A simple path finder tool with a focus on clean code (and lots of fancy libraries).");
  bool plot = false;
  bool display_help = false;
  options
    .allow_unrecognised_options()
    .add_options()("p,plot","Plot result using python matlotlib bindings", cxxopts::value<bool>(plot))
                  ("h,help", "Display this message", cxxopts::value<bool>(display_help));
  auto result = options.parse(argc, argv);
  if (display_help) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  PathPlanner planner(std::vector<Obstacle>{Obstacle{.radius=2.5, .position=Eigen::Vector2d(5,5)}});
  auto path = planner.find_shortest_path({{0,0,10}}, {{-1,-1,-1}});
  auto print_node_pos = [&planner](Graph::vertex_descriptor v) {State s = planner.get_node_state(v); ranges::copy(s, ranges::ostream_iterator<double>(std::cout, " ")); std::cout << std::endl;};
  ranges::for_each(path, print_node_pos);

  if (plot) { planner.plot_path(path); }
}
