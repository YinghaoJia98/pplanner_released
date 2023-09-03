#ifndef GRAPH_FOR_FRONTIER_H_
#define GRAPH_FOR_FRONTIER_H_

#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <eigen3/Eigen/Dense>
// #include <boost/graph/undirected_graph.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/graph/connected_components.hpp>

class Graph_For_Frontier
{
public:
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                  boost::property<boost::vertex_index_t, int>,
                                  boost::property<boost::edge_weight_t, double>,
                                  boost::no_property>
        GraphType_For_Frontier;

    // GraphType_For_Frontier graph_for_frontier_;
    // std::unordered_map<int, GraphType_For_Frontier::vertex_descriptor> vertex_frontier_descriptors_;

    using VertexDescriptor = typename GraphType_For_Frontier::vertex_descriptor;
    using EdgeDescriptor = typename GraphType_For_Frontier::edge_descriptor;
    using EdgeDescriptorPair = typename std::pair<EdgeDescriptor, bool>;

    Graph_For_Frontier();
    ~Graph_For_Frontier();

    VertexDescriptor addSourceVertex(int id);
    VertexDescriptor addVertex(int id);
    EdgeDescriptorPair addEdge(int u_id, int v_id, double weight);
    void removeEdge(int u_id, int v_id);
    void removeVertex(int id);

    // bool findDijkstraShortestPaths(int src_id, ShortestPathsReport &rep);

    int getVertexID(VertexDescriptor v);
    int getNumVertices();
    int getNumEdges();

    void getEdgeIterator(
        std::pair<GraphType_For_Frontier::edge_iterator, GraphType_For_Frontier::edge_iterator> &ei);
    std::tuple<int, int, double> getEdgeProperty(GraphType_For_Frontier::edge_iterator e);
    void getVertexIterator(
        std::pair<GraphType_For_Frontier::vertex_iterator, GraphType_For_Frontier::vertex_iterator> &vi);
    int getVertexProperty(GraphType_For_Frontier::vertex_iterator v);

    // void printResults(const ShortestPathsReport &rep);
    void clear();

    void getcomponent_and_num(std::vector<int> *component, int *num_components);

    int get_id_global(int id_internal);

private:
    GraphType_For_Frontier graph_;
    VertexDescriptor source_;
    std::unordered_map<int, VertexDescriptor> vertex_descriptors_;
    std::vector<VertexDescriptor> vertices_;
    std::unordered_map<int, int> vertex_internal_id_; // neibu id 2 wai bu id
    int id_internal;
    // int num_vertices_;

    // bool findDijkstraShortestPaths(VertexDescriptor &source,
    //                                std::vector<VertexDescriptor> &shortest_paths,
    //                                std::vector<double> &shortest_distances);
};

#endif
