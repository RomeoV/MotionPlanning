#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Dense>

using State  = Eigen::Vector3d;
using Node = boost::property<boost::vertex_name_t, State>;
using Weight = boost::property<boost::edge_weight_t, double>;

using Graph = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Node, Weight>;
// using Graph = boost::adjacency_matrix<boost::directedS, VertexProperty, EdgeProperty>;

/*
namespace std {
// Define std::begin/end for easier looping over the graph using range-for
Graph::vertex_iterator begin(std::pair<Graph::vertex_iterator, Graph::vertex_iterator>& p) {
    return p.first;
}

Graph::vertex_iterator end(std::pair<Graph::vertex_iterator, Graph::vertex_iterator>& p) {
    return p.second;
}

Graph::edge_iterator begin(std::pair<Graph::edge_iterator, Graph::edge_iterator>& p) {
    return p.first;
}

Graph::edge_iterator end(std::pair<Graph::edge_iterator, Graph::edge_iterator>& p) {
    return p.second;
}
}
*/