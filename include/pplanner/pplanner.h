#ifndef PPLANNER_H_
#define PPLANNER_H_
#include <ros/ros.h>
#include <string>
#include "pplanner/graph_manager.h"
#include "pplanner/graph_for_frontier.h"
#include "pplanner/graph_for_planning.h"
#include "pplanner/ppath_tracker.h"
#include "pplanner/union_find.h"
#include "pplanner/explorer.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/Path.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pplanner_msgs/pplanner_pub_target.h>
#include <std_srvs/Trigger.h>

#include <set>

#include <thread>
#include <boost/graph/graphviz.hpp>

#include <mutex>

// #include <boost/graph/connected_components.hpp>

struct classcomp
{
    bool operator()(const Eigen::Vector3d &lhs, const Eigen::Vector3d &cmp) const
    {

        if (lhs[0] != cmp[0])
        {
            return (lhs[0] < cmp[0]);
        }
        else if (lhs[1] != cmp[1])
        {
            return (lhs[1] < cmp[1]);
        }
        else
            return (lhs[2] < cmp[2]);
    }
};

class PPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void get_param();
    void initialize();
    void iterator_map(grid_map::GridMap traversability_map_fun);
    bool vertex_exist(StateVec state_new, std::vector<Vertex *> *nearest_vertices);
    bool if_point_internal(double x, double y, grid_map::GridMap traversability_map_fun, bool *if_point_valid, bool *if_point_bound);
    bool if_point_in_the_range(double x, double y);
    bool if_point_isNan(grid_map::GridMap traversability_map_fun, const std::string &layer, grid_map::Position position);
    void GraphManager_check(const GraphManager *graph_manager);
    void GraphManager_check(const std::shared_ptr<GraphManager> GraphManager_);
    void visualizeGraph(GraphManager *graph_manager);
    void visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_);
    void visualizeLocalPlanningGraphManager(std::shared_ptr<GraphManager> GraphManager_);
    void visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_, std::shared_ptr<ros::Publisher> TopicPublisher);
    void visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_, std::shared_ptr<ros::Publisher> TopicPublisher, string ns_);

    // bool vertex_exist_in_local_kdtree(StateVec state_new, std::vector<Vertex *> *nearest_vertices);
    bool vertex_exist_in_Graph_Manager(StateVec state_new, std::vector<Vertex *> *nearest_vertices, std::shared_ptr<GraphManager> local_graph);
    bool vertex_exist_in_Graph_Manager(StateVec state_new, std::vector<Vertex *> *nearest_vertices, std::shared_ptr<GraphManager> GraphManagerFun_, double MapResolution_);
    int get_central_point(std::vector<int> ids_global_, std::shared_ptr<GraphManager> frontier_graph, bool *if_success);

    void get_planning_graph(int source_id, int target_id,
                            std::shared_ptr<GraphManager> graph_for_planning,
                            std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                            double x_expand, double y_expand,
                            double z_expand);

    void get_planning_graph(int source_id, int target_id,
                            std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                            double x_expand, double y_expand,
                            double z_expand);

    void get_planning_graph(int source_id, int target_id,
                            std::shared_ptr<GraphManager> graph_for_planning,
                            std::shared_ptr<Graph_For_Planning> planning_boost_graph);

    void get_planning_graph(int source_id, int target_id,
                            std::shared_ptr<Graph_For_Planning> planning_boost_graph);

    void get_cost_map(const std::shared_ptr<Graph_For_Planning> planning_boost_graph,
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

    void get_cost_map(const std::shared_ptr<GraphManager> source_graph_manager,
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
    void a_star_serach(std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                       int start_global_id,
                       std::vector<double> costmap);
    double heuristic_for_planning(Graph_For_Planning::VertexDescriptor u, Graph_For_Planning::VertexDescriptor v);

    bool Find_shortest_path_by_dijkstra(int start_id, ShortestPathsReportForPlanning &rep, std::shared_ptr<Graph_For_Planning> Planning_boost_graph);

    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order, std::vector<int> &path);
    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order, std::vector<Vertex *> &path);
    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order, std::vector<Vertex *> &path,
                         std::shared_ptr<GraphManager> Graph_Manager);
    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order,
                         std::vector<Eigen::Vector3d> &path);
    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order,
                         std::vector<StateVec> &path);

    void getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                         bool source_to_target_order, std::vector<int> *path);
    double getShortestDistance(int target_id, const ShortestPathsReportForPlanning &rep);

    void visualizePath(std::vector<Vertex *> Path);
    void visualizePlanningBoostGraph(std::shared_ptr<Graph_For_Planning> planning_boost_graph);

    bool if_robot_could_around();
    void let_robot_turn_around_z_axe(double yaw_target);
    void let_robot_turn_around_z_axe(Eigen::Vector2d TargetAttitude);

    bool check_if_pass_the_area(grid_map::GridMap traversability_map_fun,
                                Eigen::Vector2d robot_location,
                                double x_expand_,
                                double angle,
                                tf::StampedTransform world_base_transform_for_safety_circumvent);

    void GetPlanningGraphFromLocalGraphManager(int source_id_in_local_planning_graph,
                                               int target_id_in_local_planning_graph,
                                               std::shared_ptr<GraphManager> source_graph_manager,
                                               std::shared_ptr<Graph_For_Planning> planning_boost_graph);

    bool if_find_local_path_(int robot_local_id,
                             int target_local_id,
                             std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                             std::vector<Vertex *> *local_path_,
                             std::shared_ptr<GraphManager> Graph_manager);
    void visualizeLocalPath(std::vector<Vertex *> Path);
    void visualizeLocalPlanningBoostGraph(std::shared_ptr<Graph_For_Planning> planning_boost_graph);
    void iterator_map_for_graph_manager_(grid_map::GridMap traversability_map_fun,
                                         std::shared_ptr<GraphManager> Graph_manager);

    void getGlobalPathAndTrackItByGlobalId(int target_global_id);
    void getGlobalPathAndTrackItByGlobalId(int target_global_id, bool *if_succeed);

    void updateFrontierOfGraphManager(std::shared_ptr<GraphManager> Graph_manager);
    void updateBoundOfGraphManager(std::shared_ptr<GraphManager> Graph_manager);

    void checkPpathTrackerIfCouldTurn(std::shared_ptr<Ppath_Tracker> Ppath_tracker);

    void visualizePointsIssued(std::vector<int> global_ids_issued);

    void visualizePointsIssued(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> global_points_issued);

    void GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                        GraphManager *source_graph_manager,
                                        std::shared_ptr<Graph_For_Planning> planning_boost_graph);
    void UpdateTargetPoint();

    void HandleEmergencyByTurnAround();

    void test_fun(); // TODO It is the function for testing code.

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    GraphManager *global_graph_;
    std::shared_ptr<GraphManager> local_graph_;
    std::shared_ptr<GraphManager> RoughLocalGraph_;
    std::shared_ptr<GraphManager> RoughGlobalGraph_;
    KD_TREE<PointType>::Ptr local_kdtree_ptr;
    std::shared_ptr<GraphManager> frontier_graph_;
    std::shared_ptr<Graph_For_Frontier> frontier_boost_graph_;
    std::shared_ptr<GraphManager> graph_for_planning_;
    // std::vector<int> frontier_id_vector;
    std::shared_ptr<Graph_For_Planning> planning_boost_graph_;
    std::shared_ptr<Ppath_Tracker> ppath_tracker_;

    std::shared_ptr<GraphManager> local_graph_for_safety_with_planning_;
    std::shared_ptr<Graph_For_Planning> planning_boost_graph_for_local_planner_;
    std::shared_ptr<Ppath_Tracker> Local_ppath_tracker_;

    std::shared_ptr<GraphManager> Big_global_graph_with_obstacles_;

    std::shared_ptr<UF> UF_;

    std::shared_ptr<Explorer> Explorer_;
    std::shared_ptr<Graph_For_Planning> planning_boost_graph_for_exploration_;

    std::shared_ptr<Ppath_Tracker> Initial_Path_tracker_;

    // ros::Subscriber pose_subscriber_;
    ros::Subscriber gridmap_traversability_subscriber_;

    ros::Publisher grid_cells_pub_;
    ros::Publisher grid_cells_of_frontier_pub_;
    ros::Publisher grid_cells_of_bound_pub_;
    ros::Publisher pplanner_graph_pub_;
    ros::Publisher frontier_submap_pub;
    ros::Publisher points_for_pre_exploration_pub;
    ros::Publisher GraphManager_pub_;
    ros::Publisher Global_Path_pub_;
    ros::Publisher Planning_Boost_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher LocalPlanningGraphManager_pub_;
    ros::Publisher Local_Path_pub_;
    ros::Publisher Local_Planning_Boost_Graph_pub_;
    ros::Publisher Path_Robot_have_Passed_pub_;
    ros::Publisher points_issued_pub;
    std::shared_ptr<ros::Publisher> WholeGloabalGraphWithObstacles_pub;
    std::shared_ptr<ros::Publisher> RoughGlobalGraph_pub;
    bool if_grid_cells_pub;
    bool if_edge_pub;
    bool if_vertex_marker_also_pub;
    nav_msgs::GridCells gridcells_msg_;
    nav_msgs::GridCells gridcells_frontier_msg_;
    nav_msgs::GridCells gridcells_bound_msg_;

    void traverability_map_Callback(const grid_map_msgs::GridMap &msg);

    grid_map_msgs::GridMap msg_traversability_map;
    grid_map::GridMap traversability_map;
    // grid_map::GridMapIterator iterator;
    grid_map::Position position;
    double tra_map_resolution;
    double tra_map_sizex;
    double tra_map_sizey;
    double tra_map_bound_x_max;
    double tra_map_bound_x_min;
    double tra_map_bound_y_max;
    double tra_map_bound_y_min;
    bool if_map_initialled_;

    double pplaner_map_resolution;
    double PplannerRoughGlobalMapResolution_;

    Eigen::Vector3d *point_in;
    std::set<Eigen::Vector3d, classcomp> *point_set;

    // std::unordered_map<int, std::vector<Eigen::Vector4d>> pplanner_vector_map_;
    // std::unordered_map<int, std::vector<Eigen::Vector4d> *, std::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4d>>> pplanner_vector_map_;
    // std::unordered_map<int, std::vector<Eigen::Vector4d>, std::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Vector4d>>> pplanner_vector_map_;
    std::unordered_map<int, std::vector<Eigen::VectorXd>, std::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::VectorXd>>> pplanner_vector_map_;
    // std::unordered_map<int, std::vector<Eigen::VectorXd>, std::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::VectorXd>>> pplanner_vector_map_for_local_map_;
    //  might need to change the aligned content
    //    std::pair<std::pair<int, int>, std::vector<Eigen::Vector4d>> pplanner_vector_map2_;

    int edges_of_one_point_max;
    // std::set<int> *id_set_;

    double limit_box_x;
    double limit_box_y;
    double limit_box_z;
    double limit_box_z_for_edge;

    std::vector<int> frontier_id_vector_;

    // double traversability_score_threshold;
    double traversability_score_threshold_local_graph;
    double traversability_score_threshold_local_graph_for_frontier;
    double traversability_score_threshold_global_graph;
    double traversability_score_threshold_low_bound;
    int bad_point_threshold;
    bool if_bad_point_threshold_improve_automatically;
    int point_threshold;
    int point_threshold_frontier;
    int worst_point_threshold_;

    std::string Grid_cell_frame_;
    std::string Elevation_layer_;
    std::string Traversability_layer_;
    std::string Step_layer_;
    std::string Traversability_supplementary_layer_;
    std::string Variance_layer_;
    std::string World_frame_;
    std::string Track_frame_;

    double Attractive_gain_;
    double Repulsive_gain_;
    double Bound_radiu_;

    double Local_Attractive_gain_;
    double Local_Repulsive_gain_;
    double Local_Bound_radiu_;

    bool if_plus_;
    double x_plus_;
    double y_plus_;
    double z_plus_;
    int Max_plus_count;

    double lookahead_time_for_safety_circumvent_;

    double x_forward_offset_for_safety_ckeck_;
    double x_backward_offset_for_safety_ckeck_;
    double robot_width;

    bool is_turning;

    bool if_update_local_planning_graph;
    bool if_local_planning_graph_updated;
    bool tracker_wait_switch;

    int global_target_id;

    int bad_point_threshold_for_check_safety;

    bool global_path_tracker_is_paused_by_user;
    bool local_path_tracker_is_paused_by_user;

    bool if_exploration;
    bool if_waiting_for_updating_target_;
    bool if_target_updated_;

    bool IfUsePathDistanceInsteadOfEuclideanDistance_;

    double LambdaSize_;
    double LambdaDistance_;
    double LambdaObstacle_;
    double LambdaZDistance_;

    bool IfAvoidObstaclesByChangeVelocity_;

    bool IfNeedHandleEmergencyByTurnAround_;

    bool IfNeedUpdateFrontierSubmap_;
    bool IfNeedUpdateGlobalGraphForUpdatingTarget_;
    bool IfNeedUpdateFrontireGraphForUpdatingTarget_;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> path_tracked_vector_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> path_tracked_vector_of_local_;

    std::vector<int> global_ids_issued_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> global_points_issued_;

    std::vector<int> BadGlobalTargetIdVector_;

    int TimesOfGlobalTargetIdRepetition_;
    int TimesOfGlobalTargetIdRepetitionThreshold_;

    Eigen::Matrix4d MembershipDegreeM_;
    Eigen::Matrix4d MembershipDegreeN_;

    ros::Timer global_graph_visualize_timer_;
    void visualizeGlobalGraphTimerCallback(const ros::TimerEvent &event);

    bool if_iterator_map_with_map_Callback;
    ros::Timer iterator_map_timer_;
    void iteratorMapTimerCallback(const ros::TimerEvent &event);

    ros::Timer map_of_frontier_update_timer_;
    void update_frontier_map(const ros::TimerEvent &event);

    ros::Timer submap_of_frontier_update_timer_;
    void update_frontier_submap(const ros::TimerEvent &event);

    std::unordered_map<int, std::vector<int>> frontier_sub_graphs;
    std::unordered_map<int, int> centers_of_frontier_sub_graphs_;

    ros::Timer map_of_bound_update_timer_;
    void update_bound_map(const ros::TimerEvent &event);

    ros::Timer path_tracker_orientation_update_timer_;
    void update_tracker_orientation(const ros::TimerEvent &event);

    ros::Timer path_tracker_safety_circumvent_update_timer_;
    void update_tracker_path_safety_circumvent(const ros::TimerEvent &event);

    ros::Timer Local_planner_timer_;
    void LocalPlannerTimerCallBack(const ros::TimerEvent &event);

    ros::Timer UpdateLocalGraphManagerTimer;
    void UpdateLocalGraphManagerTimerCallBack(const ros::TimerEvent &event);

    ros::Timer PathTrackerSwitchTimer;
    void PathTrackerSwitchTimerCallBack(const ros::TimerEvent &event);

    ros::Timer VisualizeRobotPathPassedTimer;
    void VisualizeRobotPathPassedTimerCallBack(const ros::TimerEvent &event);

    ros::Timer VisualizePointsIssuedTimer;
    void VisualizePointsIssuedTimerCallBack(const ros::TimerEvent &event);

    ros::Timer ExplorerTimer;
    void ExplorerTimerCallBack(const ros::TimerEvent &event);

    ros::Timer UpdateTargetTimer;
    void UpdateTargetTimerCallBack(const ros::TimerEvent &event);

    ros::Timer UpdateExplorerGraphManagerTimer;
    void UpdateExplorerGraphManagerTimerCallBack(const ros::TimerEvent &event);

    ros::Timer HandleEmergencyByTurnAroundTimer;
    void HandleEmergencyByTurnAroundTimerCallBack(const ros::TimerEvent &event);

    tf::TransformListener world_base_listener;
    tf::StampedTransform world_base_transform;

    ros::ServiceServer pplanner_gloabal_path_planning_server_;
    bool globalPlannerCallback(pplanner_msgs::pplanner_pub_target::Request &req,
                               pplanner_msgs::pplanner_pub_target::Response &res);

    ros::ServiceServer pplanner_gloabal_path_pause_server_;
    bool stdPausePathTrackerCallback(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_gloabal_path_continue_server_;
    bool stdContinuePathTrackerCallback(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_gloabal_path_stop_server_;
    bool stdStopPathTrackerCallback(std_srvs::Trigger::Request &req,
                                    std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_start_to_explore_server_;
    bool stdStartToExploreCallback(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_stop_exploring_server_;
    bool stdStopExploringCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_initialization_server_;
    bool stdInitializationCallback(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res);

    ros::ServiceServer pplanner_UpdateTargetPoint_server_;
    bool stdUpdateTargetPointCallback(std_srvs::Trigger::Request &req,
                                      std_srvs::Trigger::Response &res);

    std::unordered_map<int, int> point_pathplanning_interl_to_global;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> LocalPlanningBoundObstacleVector_;

    nav_msgs::Path Path_Robot_Have_Passed_Msg_;
    int pos_seq;

    // std::mutex updateMapMutex_;
    mutable std::mutex updateMapMutex_;
    mutable std::mutex planning_boost_graph_Mutex_;
    mutable std::mutex traversability_map_Mutex_;
    mutable std::mutex Local_Planner_Graph_Manager_Mutex_;
    mutable std::mutex Sub_Graph_Of_Frontier_Mutex_;
    mutable std::mutex If_exploration_Mutex_;
    mutable std::mutex Update_Explorer_Mutex_;
    mutable std::mutex BigWholeGrpah_Mutex_;
    mutable std::mutex HandleEmergencyByTurnAround_Mutex_;
    mutable std::mutex UpdateFrontierSubmapForExplorer_Mutex_;
    mutable std::mutex UpdateGlobalGraphForExplorer_Mutex_;
    mutable std::mutex UpdateBadTargetPointVector_Mutex_;
    mutable std::mutex TheStateOfIfWaitingAndUpdatedTarget_Mutex_;
    mutable std::mutex TheRoughGlobalGraph_Mutex_;
    mutable std::mutex UpdateFrontierGraphForExplorer_Mutex_;
};

#endif
