#include <vector>
#include <array>
#include <Eigen/Dense>
#include "graph_definition.h"

using Pos = Eigen::Vector2d;
using State  = Eigen::Vector3d;
using Interval = std::pair<double, double>;

struct Obstacle {
    double radius;
    Pos position;
};

struct Environment {
    std::array<Interval, 3> domain_bounds;
};

namespace {
    constexpr size_t D = std::tuple_size<decltype(Environment::domain_bounds)>::value;
}

class PathPlanner {
public:
    PathPlanner(std::vector<Obstacle> obstacles_=std::vector<Obstacle>{}); 

    std::vector<Graph::vertex_descriptor> find_shortest_path(std::array<int, 3> const& from, std::array<int, 3> const& to) const;
    std::vector<Graph::vertex_descriptor> find_shortest_path(Graph::vertex_descriptor const& from, Graph::vertex_descriptor const& to) const;

    State get_node_state(Graph::vertex_descriptor const& node) const {
        return boost::get(Node::tag_type(), this->distance_graph, node);
    }

    void plot_path(std::vector<Graph::vertex_descriptor> const& path);
private:
    void create_discretizations();
    void populate_graph();
    void populate_graph_vertices();
    void populate_graph_edges();
    void remove_graph_vertices();

    double compute_node_distance(const Graph::vertex_descriptor& lhs, const Graph::vertex_descriptor& rhs) const;
    bool vertex_close_to_any_obstacle(Graph::vertex_descriptor const& node) const;

private:
    const Environment env;
    const std::vector<Obstacle> obstacles;
    const std::array<size_t, D> num_discretization_points;
    std::array<std::vector<double>, D> discretization;
    std::vector<std::vector<std::vector<Graph::vertex_descriptor>>> graph_nodes;
    Graph distance_graph;
};