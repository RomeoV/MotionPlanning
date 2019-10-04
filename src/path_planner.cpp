#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>
#include <iterator>
#include <iostream>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/reverse.hpp>
#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/iterator/stream_iterators.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <cppitertools/itertools.hpp>
#include <fmt/format.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "path_planner.h"
#include "matplotlib-cpp/matplotlibcpp.h"

using std::placeholders::_1;

namespace views = ranges::views;
namespace plt = matplotlibcpp;

PathPlanner::PathPlanner(std::vector<Obstacle> obstacles_/*=std::vector<Obstacle>{}*/)
    : env{.domain_bounds = std::array<Interval, 3>{{{0,10},{0,10},{-M_PI,M_PI}}}},
      num_discretization_points({50,50,50}),
      obstacles(obstacles_)
{
    create_discretizations();
    populate_graph();
};

std::vector<Graph::vertex_descriptor> PathPlanner::find_shortest_path(std::array<int, 3> const& from, std::array<int, 3> const& to) const {
    size_t from_i = (from[0] >= 0)? from[0] : this->num_discretization_points[0]+from[0];
    size_t from_j = (from[1] >= 0)? from[1] : this->num_discretization_points[1]+from[1];
    size_t from_k = (from[2] >= 0)? from[2] : this->num_discretization_points[2]+from[2];
    
    size_t to_i = (to[0] >= 0)? to[0] : this->num_discretization_points[0]+to[0];
    size_t to_j = (to[1] >= 0)? to[1] : this->num_discretization_points[1]+to[1];
    size_t to_k = (to[2] >= 0)? to[2] : this->num_discretization_points[2]+to[2];

    return this->find_shortest_path(this->graph_nodes[from_i][from_j][from_k], this->graph_nodes[to_i][to_j][to_k]);
}

std::vector<Graph::vertex_descriptor> PathPlanner::find_shortest_path(Graph::vertex_descriptor const& from, Graph::vertex_descriptor const& to) const {
    boost::vector_property_map<Graph::vertex_descriptor> directions(boost::num_vertices(this->distance_graph));
    boost::dijkstra_shortest_paths(this->distance_graph, from, boost::predecessor_map(&directions[0]));  // The algorithm will use the edge_weight_t property by default.
    std::vector<Graph::vertex_descriptor> path;
    auto current_node = to;
    while (current_node != from) {
        path.push_back(current_node);
        current_node = directions[current_node];
    }
    path.push_back(current_node);

    ranges::reverse(path);
    return path;
}

void PathPlanner::plot_path(std::vector<Graph::vertex_descriptor> const& path) {
    std::vector<double> pos_x;
    std::vector<double> pos_y;

    auto get_vertex_coordinate = [this](Graph::vertex_descriptor const& v, size_t idx) -> double {
        return this->get_node_state(v)[idx];
    };

    ranges::transform(path, ranges::back_inserter(pos_x), std::bind(get_vertex_coordinate, _1, 0));
    ranges::transform(path, ranges::back_inserter(pos_y), std::bind(get_vertex_coordinate, _1, 1));

    plt::figure_size(1000,1000);
    plt::scatter(pos_x, pos_y);
    plt::show();
}

void PathPlanner::create_discretizations() {
    for (auto i : views::iota(size_t{0}, this->discretization.size())) {
        auto N_i = this->num_discretization_points[i];
        Interval I = this->env.domain_bounds[i];
        this->discretization[i] = views::iota(0u, N_i)
                                    | views::transform([N_i, I](int i) {return I.first + i*1.0/(N_i-1) * (I.second - I.first);})
                                    | ranges::to<std::vector<double>>;
        // ranges::copy(this->discretization[i], ranges::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
    }
}

void PathPlanner::populate_graph() {
    this->populate_graph_vertices();
    this->populate_graph_edges();
    this->remove_graph_vertices();
}

void PathPlanner::populate_graph_vertices() {
    this->graph_nodes = std::vector<std::vector<std::vector<Graph::vertex_descriptor>>>(this->num_discretization_points[0]);
    for (auto&& i : iter::range(this->num_discretization_points[0])) {
        this->graph_nodes[i] = std::vector<std::vector<Graph::vertex_descriptor>>(this->num_discretization_points[1]);
        for (auto&& j : iter::range(this->num_discretization_points[0])) {
            this->graph_nodes[i][j] = std::vector<Graph::vertex_descriptor>(this->num_discretization_points[2]);
        }
    }

    for (auto&& [i, j, k] : iter::product(iter::range(this->num_discretization_points[0]),
                                          iter::range(this->num_discretization_points[1]),
                                          iter::range(this->num_discretization_points[2])))
    {
        auto x = this->discretization[0][i];
        auto y = this->discretization[1][j];
        auto z = this->discretization[2][k];
        this->graph_nodes[i][j][k] = boost::add_vertex(Node{State(x, y, z)}, this->distance_graph);
    }
}

void PathPlanner::populate_graph_edges() {
    std::set<std::vector<int>> combinations;
    for (auto&& el : iter::combinations_with_replacement(std::vector<int>{-1,0,1}, 3)) {
        for (auto&& perm : iter::permutations(el)) {
            combinations.insert(std::vector<int>(perm.begin(), perm.end()));
        }
    }
    combinations.erase(combinations.find({0,0,0}));

    for (auto&& [i, j, k] : iter::product(iter::range(this->num_discretization_points[0]),
                                          iter::range(this->num_discretization_points[1]),
                                          iter::range(this->num_discretization_points[2])))
    {
        for (auto shift : combinations) {
            try {
                auto lhs_vertex = this->graph_nodes[i][j][k];
                auto rhs_vertex = this->graph_nodes.at(i+shift[0]).at(j+shift[1]).at(k+shift[2]);
                if (this->vertex_close_to_any_obstacle(lhs_vertex) or
                    this->vertex_close_to_any_obstacle(rhs_vertex)) { continue; }
                auto edge_weight = Weight{PathPlanner::compute_node_distance(lhs_vertex, rhs_vertex)};
                boost::add_edge(lhs_vertex, rhs_vertex, edge_weight, this->distance_graph);
            }
            catch (std::out_of_range& exception) {
                ;  // pass
            }
        }
    }
}

void PathPlanner::remove_graph_vertices() {
    ;
}

double PathPlanner::compute_node_distance(const Graph::vertex_descriptor& lhs_vertex, const Graph::vertex_descriptor& rhs_vertex) const {
    auto lhs = boost::get(Node::tag_type(), this->distance_graph, lhs_vertex);
    auto rhs = boost::get(Node::tag_type(), this->distance_graph, rhs_vertex);

    auto position_distance = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) -> double {
        Eigen::Vector2d lhs_pos = lhs.head(2);
        Eigen::Vector2d rhs_pos = rhs.head(2);
        return (rhs_pos-lhs_pos).norm();
    };

    auto angle_distance = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) -> double {
        double lhs_ang = lhs[2];
        double rhs_ang = rhs[2];
        return (Eigen::Vector2d(std::cos(lhs_ang), std::sin(lhs_ang))
               -Eigen::Vector2d(std::cos(rhs_ang), std::sin(rhs_ang))).norm();
    };

    auto direction_distance = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) -> double {
        // loss based on how much the angle lhs[2] is pointing in the direction lhs[:2]->rhs[:2]
        auto angle_dir = Eigen::Vector2d(std::cos(lhs[2]), std::sin(lhs[2]));
        auto movement_dir = Eigen::Vector2d(rhs.head(2) - lhs.head(2)).normalized();
        return std::max(1. - angle_dir.dot(movement_dir) - 0.001, 0.);  // allows margin of error = rad2deg(np.arccos(1-0.001)) about 2.5deg
    };

    return position_distance(lhs, rhs) + angle_distance(lhs, rhs) + direction_distance(lhs, rhs);
}

bool PathPlanner::vertex_close_to_any_obstacle(Graph::vertex_descriptor const& node) const {
    Eigen::Vector2d node_pos = this->get_node_state(node).head(2);
    for (const auto& obstacle : this->obstacles) {
        if ((obstacle.position-node_pos).norm() < obstacle.radius) {
            return true;
        }
    }
    return false;
}