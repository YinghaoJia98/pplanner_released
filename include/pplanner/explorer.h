#ifndef EXPLORER_H_
#define EXPLORER_H_
#include <ros/ros.h>
#include <string>
#include "pplanner/graph_manager.h"
#include "pplanner/graph_for_planning.h"

class Explorer
{
public:
    Explorer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    bool setGraphManager(GraphManager *Graph_Manager);
    bool setGraphManager(GraphManager Graph_Manager);
    bool setWholeGraph(GraphManager Graph_Manager);
    bool setPlanningBoostGraph(std::shared_ptr<Graph_For_Planning> PlanningBoostGraph);
    bool setRobotGlobalId(int robot_id_global);
    bool setRobotRoughGlobalGraphId(int robot_id_global);
    bool setRobotGlobalIdAndConvertToRoughGlobalGraphId(int robot_id_global);
    bool updateDistance(std::vector<int> point_ids_global);
    bool updateDistanceByRoughGlobalGraph(std::vector<int> point_ids_global);
    bool updateEuclideanDistance(int robot_id_global, std::vector<int> point_ids_global);
    bool updateEuclideanDistanceInZAxeSolely(int robot_id_global, std::vector<int> point_ids_global);
    bool Find_shortest_path_by_dijkstra(int start_id, ShortestPathsReportForPlanning &rep, Graph_For_Planning PlanningBoostGraph);
    bool Find_shortest_path_by_dijkstra(int start_id, ShortestPathsReport &rep, GraphManager PlanningGraphManager);
    void GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                        GraphManager *source_graph_manager,
                                        std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                                        double LimitBoxX,
                                        double LimitBoxY,
                                        double LimitBoxZ_for_edge,
                                        int EdgesOfOnePointMax,
                                        double Local_Attractive_gain,
                                        double Local_Repulsive_gain,
                                        double Local_Bound_radiu);
    void GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                        GraphManager *source_graph_manager,
                                        Graph_For_Planning *planning_boost_graph,
                                        double LimitBoxX,
                                        double LimitBoxY,
                                        double LimitBoxZ_for_edge,
                                        int EdgesOfOnePointMax,
                                        double Local_Attractive_gain,
                                        double Local_Repulsive_gain,
                                        double Local_Bound_radiu);
    bool UpdatePlanningGraphForExploration(int source_id_global, int target_id_global,
                                           double LimitBoxX,
                                           double LimitBoxY,
                                           double LimitBoxZ_for_edge,
                                           int EdgesOfOnePointMax,
                                           double Local_Attractive_gain,
                                           double Local_Repulsive_gain,
                                           double Local_Bound_radiu);

    void get_cost_map(GraphManager *source_graph_manager,
                      const std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                      std::vector<double> *cost_map,
                      std::unordered_map<int, double> *cost_unorder_map,
                      int robot_global_id_,
                      int target_global_id_,
                      const Eigen::Vector3d robot_pos_,
                      const Eigen::Vector3d target_pos_,
                      double attractive_gain,
                      double repulsive_gain,
                      double bound_radiu_,
                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector);

    void get_cost_map(GraphManager *source_graph_manager,
                      Graph_For_Planning *planning_boost_graph,
                      std::vector<double> *cost_map,
                      std::unordered_map<int, double> *cost_unorder_map,
                      int robot_global_id_,
                      int target_global_id_,
                      const Eigen::Vector3d robot_pos_,
                      const Eigen::Vector3d target_pos_,
                      double attractive_gain,
                      double repulsive_gain,
                      double bound_radiu_,
                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector);
    GraphManager getGraphManager();

    bool setRoughGlobalGraph(GraphManager Graph_Manager);

    bool updateObstaclesAttribute(std::vector<int> point_ids_global, double Size_X_2, double Size_Y_2, double Size_Z_2);
    std::unordered_map<int, double> get_ObstaclesAttribute_map();

    void ComputeExplorationGain(std::unordered_map<int, ExplorationGainVec> *IdsAndAttribute,
                                double Lambda_Size_, double Lambda_Distance_, double Lambda_Obstacle_, double Lambda_ZDistance_);

    void reset();
    std::unordered_map<int, double> get_distances_map();
    std::unordered_map<int, double> get_DistanceInZ_map();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    GraphManager BigWholeGraph_;
    GraphManager GraphManager_;
    Graph_For_Planning PlanningBoostGraph_;
    std::unordered_map<int, double> id_distance;
    std::unordered_map<int, double> id_z_distance;
    std::unordered_map<int, double> id_obstacles_distance;
    int robot_id_global_;
    GraphManager RoughGloablGraph_;

    mutable std::mutex UpdateGraphManagerMutex_;
    mutable std::mutex UpdatePlanningBoostGraphMutex_;
    mutable std::mutex UpdateBigWholeGraphMutex_;
    mutable std::mutex UpdateRoughGlobalGraphMutex_;
};

#endif
