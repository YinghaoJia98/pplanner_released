#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include "ikd-Tree/ikd_Tree_impl.h"

#include <tf/transform_datatypes.h>

#include "pplanner/graph.h"
#include "pplanner/graph_base.h"
#include "pplanner/params.h"

#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// #include "pplanner/graph_base.h"

using PointType = ikdTree_PointType_new;
using PointVector = KD_TREE<PointType>::PointVector;

class GraphManager
{
public:
    GraphManager();
    void reset();
    int generateSubgraphIndex();
    int generateVertexID();
    void addVertex(Vertex *v);
    void addEdge(Vertex *v, Vertex *u, double weight);
    void removeEdge(Vertex *v, Vertex *u);

    void removeVertex(Vertex *v);

    int getNumVertices();
    int getNumEdges();
    Vertex *getVertex(int id);

    void getLeafVertices(std::vector<Vertex *> &leaf_vertices);
    void findLeafVertices(const ShortestPathsReport &rep);
    bool findShortestPaths(ShortestPathsReport &rep);
    bool findShortestPaths(int source_id, ShortestPathsReport &rep);
    void getShortestPath(int target_id, const ShortestPathsReport &rep,
                         bool source_to_target_order, std::vector<int> &path);
    void getShortestPath(int target_id, const ShortestPathsReport &rep,
                         bool source_to_target_order, std::vector<Vertex *> &path);
    void getShortestPath(int target_id, const ShortestPathsReport &rep,
                         bool source_to_target_order,
                         std::vector<Eigen::Vector3d> &path);
    void getShortestPath(int target_id, const ShortestPathsReport &rep,
                         bool source_to_target_order,
                         std::vector<StateVec> &path);
    double getShortestDistance(int target_id, const ShortestPathsReport &rep);
    int getParentIDFromShortestPath(int target_id,
                                    const ShortestPathsReport &rep);

    bool getNearestVertex(const StateVec *state, Vertex **v_res);
    bool getNearestVertexInRange(const StateVec *state, double range,
                                 Vertex **v_res);
    bool getNearestVertices(const StateVec *state, double range,
                            std::vector<Vertex *> *v_res);
    // bool existVertexInRange(const StateVec *state, double range);

    bool getNearestVerticesInBox(const StateVec *state, double limitx_, double limity_,
                                 double limitz_, std::vector<Vertex *> *v_res);

    void updateVertexTypeInRange(StateVec &state, double range);

    void addVertex_to_ikdtree(Vertex *v);
    void removeVertex_from_ikdtree(Vertex *v);

    void saveGraph(const std::string &path);
    void loadGraph(const std::string &path);

    std::shared_ptr<Graph> Graph_;
    std::unordered_map<int, Vertex *> vertices_map_;
    // std::map<int, std::vector<std::pair<int, double>>> edge_map_; // id:  <neighbor id, edge cost>

    int Nearest_Num0 = 1;
    int Nearest_Num1 = 8;

    double max_dist0 = 3.0;
    double max_dist1 = 5.0;

private:
    // KD_TREE<PointType>::Ptr kdtree_ptr;
    KD_TREE<PointType> *kdtree_ptr;
    // KD_TREE<pcl::PointXYZ> &ikd_Tree = *kdtree_ptr;

    int subgraph_ind_;
    int id_count_;
};

#endif