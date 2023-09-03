#ifndef GRAPH_FOR_PLANNING_H_
#define GRAPH_FOR_PLANNING_H_

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
#include <boost/graph/astar_search.hpp>

struct ShortestPathsReportForPlanning
{
    ShortestPathsReportForPlanning() : status(false), source_id(0) {}
    void reset()
    {
        status = false;
        source_id = 0;
        parent_id_map.clear();
        distance_map.clear();
    }
    bool status;   // False if can not run the shortest path algorithm.
    int source_id; // ID of the source vertex.
    // Direct parent ID related to the shortest path corresponding to each ID in
    // the id_list
    std::unordered_map<int, int> parent_id_map;
    // Shortest distance to source corresponding to each id from id_list
    std::unordered_map<int, double> distance_map;
};

class Graph_For_Planning
{
public:
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                  boost::property<boost::vertex_index_t, int>,
                                  boost::property<boost::edge_weight_t, double>,
                                  boost::no_property>
        GraphType_For_PLANNING;

    using VertexDescriptor = typename GraphType_For_PLANNING::vertex_descriptor;
    using EdgeDescriptor = typename GraphType_For_PLANNING::edge_descriptor;
    using EdgeDescriptorPair = typename std::pair<EdgeDescriptor, bool>;

    Graph_For_Planning();
    ~Graph_For_Planning();

    VertexDescriptor addSourceVertex(int id);
    VertexDescriptor addVertex(int id);
    EdgeDescriptorPair addEdge(int u_id, int v_id, double weight);
    void removeEdge(int u_id, int v_id);
    void removeVertex(int id);

    bool findDijkstraShortestPaths(int src_id, ShortestPathsReportForPlanning &rep);
    bool findAstaerShortestPaths(int start_global_id, int target_global_id);

    int getVertexID(VertexDescriptor v);
    int getNumVertices();
    int getNumEdges();

    void getEdgeIterator(
        std::pair<GraphType_For_PLANNING::edge_iterator, GraphType_For_PLANNING::edge_iterator> &ei);
    std::tuple<int, int, double> getEdgeProperty(GraphType_For_PLANNING::edge_iterator e);
    void getVertexIterator(
        std::pair<GraphType_For_PLANNING::vertex_iterator, GraphType_For_PLANNING::vertex_iterator> &vi);
    int getVertexProperty(GraphType_For_PLANNING::vertex_iterator v);

    void printResults(const ShortestPathsReportForPlanning &rep);
    void clear();

    void getcomponent_and_num(std::vector<int> *component, int *num_components);

    int get_id_global(int IdInternal);
    int get_id_internal(int IdGlobal);

    void get_total_force(const Eigen::Vector3d robot_pos_,
                         const Eigen::Vector3d target_pos_,
                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacles_,
                         Eigen::Vector3d *force_,
                         double attractive_gain,
                         double repulsive_gain,
                         double bound_radiu_);
    void get_cost_map(const GraphType_For_PLANNING graph_,
                      std::vector<double> *cost_map,
                      int robot_global_id_,
                      int target_global_id_,
                      const Eigen::Vector3d robot_pos_,
                      const Eigen::Vector3d target_pos_,
                      double attractive_gain,
                      double repulsive_gain,
                      double bound_radiu_);
    void dijkstra_process(int target_global_id_, std::vector<int> *parent, std::vector<double> *d);

    GraphType_For_PLANNING graph_;
    VertexDescriptor source_;
    std::unordered_map<int, VertexDescriptor> vertex_descriptors_;

private:
    // GraphType_For_PLANNING graph_;
    // VertexDescriptor source_;
    // std::unordered_map<int, VertexDescriptor> vertex_descriptors_;
    std::vector<VertexDescriptor> vertices_;
    std::unordered_map<int, int> vertex_internal_id_; // neibu id 2 wai bu id
    std::unordered_map<int, int> vertex_global_id_;   // global id to internal id
    int id_internal;
    int num_vertices_;

    bool findDijkstraShortestPaths(VertexDescriptor &source,
                                   std::vector<VertexDescriptor> &shortest_paths,
                                   std::vector<double> &shortest_distances);
};

// template <class Vertex>
// class astar_goal_visitor : public boost::default_astar_visitor
// {
// public:
//     astar_goal_visitor(Vertex goal) : m_goal(goal) {}
//     template <class Graph>
//     void examine_vertex(Vertex u, Graph &g)
//     {
//         if (u == m_goal)
//             throw found_goal();
//     }

// private:
//     Vertex m_goal;
// };

// struct found_goal
// {
// }; // exception for termination

#endif
