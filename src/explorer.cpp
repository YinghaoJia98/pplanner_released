#include "pplanner/explorer.h"
Explorer::Explorer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    GraphManager_.reset();
    PlanningBoostGraph_.clear();
    id_distance.clear();
    id_z_distance.clear();
    robot_id_global_ = -1;
    id_obstacles_distance.clear();
}

bool Explorer::setGraphManager(GraphManager *Graph_Manager)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    GraphManager_ = *Graph_Manager;
    return true;
}

bool Explorer::setGraphManager(GraphManager Graph_Manager)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    GraphManager_ = Graph_Manager;
    return true;
}

bool Explorer::setWholeGraph(GraphManager Graph_Manager)
{
    std::lock_guard<std::mutex> lock3(UpdateBigWholeGraphMutex_);
    BigWholeGraph_ = Graph_Manager;
    return true;
}

bool Explorer::setPlanningBoostGraph(std::shared_ptr<Graph_For_Planning> PlanningBoostGraph)
{
    std::lock_guard<std::mutex> lock2(UpdatePlanningBoostGraphMutex_);
    PlanningBoostGraph_ = *PlanningBoostGraph;
    return true;
}
bool Explorer::setRobotGlobalId(int robot_id_global)
{
    robot_id_global_ = robot_id_global;
    return true;
}

bool Explorer::setRobotRoughGlobalGraphId(int robot_id_global)
{
    robot_id_global_ = robot_id_global;
    return true;
}

bool Explorer::setRobotGlobalIdAndConvertToRoughGlobalGraphId(int robot_id_global)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    std::lock_guard<std::mutex> lock4(UpdateRoughGlobalGraphMutex_);
    Vertex *VertexInGlobalGraphPtr = GraphManager_.getVertex(robot_id_global);
    StateVec VertexInGlobalGraphState = VertexInGlobalGraphPtr->state;
    Vertex *RobotNearestVerticesInRoughGlobalGraph;
    if (RoughGloablGraph_.getNearestVertex(&VertexInGlobalGraphState, &RobotNearestVerticesInRoughGlobalGraph))
    {
        robot_id_global_ = RobotNearestVerticesInRoughGlobalGraph->id;
        return true;
    }
    else
    {
        ROS_ERROR("Can not get the nearest in rough global graph.");
        return false;
    }
    // robot_id_global_ = robot_id_global;
    return true;
}

bool Explorer::setRoughGlobalGraph(GraphManager Graph_Manager)
{
    std::lock_guard<std::mutex> lock4(UpdateRoughGlobalGraphMutex_);
    RoughGloablGraph_ = Graph_Manager;
    return true;
}

bool Explorer::updateDistance(std::vector<int> point_ids_global)
{
    if (point_ids_global.size() == 0)
    {
        ROS_ERROR("The size of ids is zero when update the distances of potential exploration point.");
        return false;
    }
    else
    {
    }
    ShortestPathsReportForPlanning report_for_distances;
    id_distance.clear();
    Find_shortest_path_by_dijkstra(robot_id_global_, report_for_distances, PlanningBoostGraph_);
    for (int i = 0; i < point_ids_global.size(); i++)
    {
        if (report_for_distances.distance_map.find(point_ids_global[i]) != report_for_distances.distance_map.end())
        {
            id_distance[point_ids_global[i]] = report_for_distances.distance_map[point_ids_global[i]];
        }
        else
        {
            ROS_ERROR("The id %d is not exist in report of dijkstra method.", point_ids_global[i]);
            id_distance[point_ids_global[i]] = 10000;
            // return false;
        }
    }
    return true;
}

bool Explorer::updateDistanceByRoughGlobalGraph(std::vector<int> point_ids_global)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    std::lock_guard<std::mutex> lock4(UpdateRoughGlobalGraphMutex_);
    if (point_ids_global.size() == 0)
    {
        ROS_ERROR("The size of ids is zero when update the distances of potential exploration point.");
        return false;
    }
    else
    {
    }
    ShortestPathsReport report_for_distances;
    id_distance.clear();
    Find_shortest_path_by_dijkstra(robot_id_global_, report_for_distances, RoughGloablGraph_);
    for (int i = 0; i < point_ids_global.size(); i++)
    {
        int target_id_in_rough_global_ = 0;
        Vertex *VertexInGlobalGraphPtr_middle_ = GraphManager_.getVertex(point_ids_global[i]);
        StateVec VertexInGlobalGraphState_middle_ = VertexInGlobalGraphPtr_middle_->state;
        Vertex *RobotNearestVerticesInRoughGlobalGraph_middle_;
        if (RoughGloablGraph_.getNearestVertex(&VertexInGlobalGraphState_middle_, &RobotNearestVerticesInRoughGlobalGraph_middle_))
        {
            target_id_in_rough_global_ = RobotNearestVerticesInRoughGlobalGraph_middle_->id;
        }
        else
        {
            ROS_ERROR("Can not get the nearest point to target in rough global graph.");
            id_distance[point_ids_global[i]] = 10000;
            continue;
        }

        if (report_for_distances.distance_map.find(target_id_in_rough_global_) != report_for_distances.distance_map.end())
        {
            id_distance[point_ids_global[i]] = report_for_distances.distance_map[target_id_in_rough_global_];
        }
        else
        {
            ROS_ERROR("The rough id %d is not exist in report of dijkstra method.", target_id_in_rough_global_);
            id_distance[point_ids_global[i]] = 10000;
            // return false;
        }
    }
    return true;
}

bool Explorer::Find_shortest_path_by_dijkstra(int start_id, ShortestPathsReportForPlanning &rep, Graph_For_Planning PlanningBoostGraph)
{
    return PlanningBoostGraph.findDijkstraShortestPaths(start_id, rep);
}

bool Explorer::Find_shortest_path_by_dijkstra(int start_id, ShortestPathsReport &rep, GraphManager PlanningGraphManager)
{
    return PlanningGraphManager.findShortestPaths(start_id, rep);
}

bool Explorer::updateEuclideanDistance(int robot_id_global, std::vector<int> point_ids_global)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    if (GraphManager_.vertices_map_.find(robot_id_global) == GraphManager_.vertices_map_.end())
    {
        ROS_ERROR("[updateEuclideanDistance_INFO]: Can not get the robot state.");
        return false;
    }
    Vertex *RobotVertex = GraphManager_.getVertex(robot_id_global);
    Eigen::Vector3d RobotState_(RobotVertex->state[0],
                                RobotVertex->state[1],
                                RobotVertex->state[2]);
    id_distance.clear();
    id_z_distance.clear();
    for (int i = 0; i < point_ids_global.size(); i++)
    {
        if (GraphManager_.vertices_map_.find(point_ids_global[i]) == GraphManager_.vertices_map_.end())
        {
            ROS_ERROR("[updateEuclideanDistance_INFO]: Can not get the target state of id %f.", point_ids_global[i]);
            id_distance[point_ids_global[i]] = 10000;
            id_z_distance[point_ids_global[i]] = 10000;
        }
        else
        {
            Vertex *TargetVertex = GraphManager_.getVertex(point_ids_global[i]);
            Eigen::Vector3d TargetState_(TargetVertex->state[0],
                                         TargetVertex->state[1],
                                         TargetVertex->state[2]);
            Eigen::Vector3d Direction_middle_ = TargetState_ - RobotState_;
            double Direction_norm_middle = Direction_middle_.norm();
            id_distance[point_ids_global[i]] = Direction_norm_middle;
            id_z_distance[point_ids_global[i]] = abs(Direction_middle_[2]);
        }
    }
    return true;
}

bool Explorer::updateEuclideanDistanceInZAxeSolely(int robot_id_global, std::vector<int> point_ids_global)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    if (GraphManager_.vertices_map_.find(robot_id_global) == GraphManager_.vertices_map_.end())
    {
        ROS_ERROR("[updateEuclideanDistance_INFO]: Can not get the robot state.");
        return false;
    }
    Vertex *RobotVertex = GraphManager_.getVertex(robot_id_global);
    Eigen::Vector3d RobotState_(RobotVertex->state[0],
                                RobotVertex->state[1],
                                RobotVertex->state[2] - 0.3);

    id_z_distance.clear();
    for (int i = 0; i < point_ids_global.size(); i++)
    {
        if (GraphManager_.vertices_map_.find(point_ids_global[i]) == GraphManager_.vertices_map_.end())
        {
            ROS_ERROR("[updateEuclideanDistance_INFO]: Can not get the target state of id %f.", point_ids_global[i]);
            id_z_distance[point_ids_global[i]] = 10000;
        }
        else
        {
            Vertex *TargetVertex = GraphManager_.getVertex(point_ids_global[i]);
            Eigen::Vector3d TargetState_(TargetVertex->state[0],
                                         TargetVertex->state[1],
                                         TargetVertex->state[2]);
            Eigen::Vector3d Direction_middle_ = TargetState_ - RobotState_;
            id_z_distance[point_ids_global[i]] = abs(Direction_middle_[2]);
        }
    }
    return true;
}

void Explorer::reset()
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    std::lock_guard<std::mutex> lock2(UpdatePlanningBoostGraphMutex_);
    std::lock_guard<std::mutex> lock3(UpdateBigWholeGraphMutex_);
    BigWholeGraph_.reset();
    GraphManager_.reset();
    PlanningBoostGraph_.clear();
    id_distance.clear();
    id_z_distance.clear();
    robot_id_global_ = -1;
    id_obstacles_distance.clear();
}
std::unordered_map<int, double> Explorer::get_distances_map()
{
    return id_distance;
}

std::unordered_map<int, double> Explorer::get_DistanceInZ_map()
{
    return id_z_distance;
}

void Explorer::GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                              GraphManager *source_graph_manager,
                                              std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                                              double LimitBoxX,
                                              double LimitBoxY,
                                              double LimitBoxZ_for_edge,
                                              int EdgesOfOnePointMax,
                                              double Local_Attractive_gain,
                                              double Local_Repulsive_gain,
                                              double Local_Bound_radiu)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ObstacleVector_middle;
    ObstacleVector_middle.clear();
    // ROS_INFO("GetPlanningGraphForExploration_neibu di 1 guan");

    if (planning_boost_graph != NULL)
    {
        planning_boost_graph->clear();
    }

    const Vertex *vertex_source_ = source_graph_manager->getVertex(source_id_global);
    const Vertex *vertex_target_ = source_graph_manager->getVertex(target_id_global);

    const std::unordered_map<int, Vertex *> source_vertices_map_ = source_graph_manager->vertices_map_;

    if (source_vertices_map_.size() > 0)
    {

        for (auto &iter_vertices_map_ : source_vertices_map_)
        {
            int id = iter_vertices_map_.first;
            StateVec state_planning_new(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2], 0);
            // Vertex *vertex_planning_new;

            if (id == 0)
            {
                planning_boost_graph->addSourceVertex(id);
            }
            else if (planning_boost_graph->vertex_descriptors_.find(id) == planning_boost_graph->vertex_descriptors_.end())
            {
                planning_boost_graph->addVertex(id);
            }
            else
            {
            }

            if (iter_vertices_map_.second->is_bound)
            {
                Eigen::Vector3d obstacle(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2]);
                ObstacleVector_middle.push_back(obstacle);
            }
            else
            {
            }

            std::vector<Vertex *> nearest_vertices_in_small_box;
            source_graph_manager->getNearestVerticesInBox(&state_planning_new, LimitBoxX, LimitBoxY,
                                                          LimitBoxZ_for_edge, &nearest_vertices_in_small_box);
            if (nearest_vertices_in_small_box.size() > 9)
            {
                ROS_ERROR("[Explorer]: THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box in GetPlanningGraphForExploration function.");
                for (int ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
                {
                    std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
                    std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
                }
            }
            for (int j = 0; (j < nearest_vertices_in_small_box.size()) && (j < EdgesOfOnePointMax); ++j)
            {
                if (iter_vertices_map_.second->id == nearest_vertices_in_small_box[j]->id)
                {

                    continue;
                }

                Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                          state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                          state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
                double direction_norm = direction.norm();

                planning_boost_graph->removeEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id);
                planning_boost_graph->addEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id, direction_norm);
            }
        }
        // generate cost map
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 2 guan");
        std::vector<double> cost_map;
        std::unordered_map<int, double> cost_unorder_map;
        int robot_global_id_ = source_id_global;
        int target_global_id_ = target_id_global;
        Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
        Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
        double attractive_gain = Local_Attractive_gain;
        double repulsive_gain = Local_Repulsive_gain;
        double bound_radiu_ = Local_Bound_radiu;
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3 guan");

        //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
        cost_map.clear();
        cost_unorder_map.clear();
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3.5 guan");

        get_cost_map(source_graph_manager, planning_boost_graph, &cost_map, &cost_unorder_map,
                     robot_global_id_, target_global_id_,
                     robot_pos_, target_pos_,
                     attractive_gain, repulsive_gain, bound_radiu_,
                     ObstacleVector_middle);

        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 4 guan");

        // update boost edge
        for (auto &iter_vertices_map2_ : source_vertices_map_)
        {
            StateVec state_planning_new(iter_vertices_map2_.second->state[0],
                                        iter_vertices_map2_.second->state[1],
                                        iter_vertices_map2_.second->state[2], 0);
            std::vector<Vertex *> nearest_vertices_in_small_box2;
            source_graph_manager->getNearestVerticesInBox(&state_planning_new, LimitBoxX, LimitBoxY,
                                                          LimitBoxZ_for_edge, &nearest_vertices_in_small_box2);
            if (nearest_vertices_in_small_box2.size() > 9)
            {
                ROS_ERROR("[Explorer]: THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2 in GetPlanningGraphForExploration function.");
                ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
                for (int ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
                {
                    std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
                    std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
                }
            }
            else
            {
            }
            for (int jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < EdgesOfOnePointMax); ++jj)
            {

                if (iter_vertices_map2_.second->id == nearest_vertices_in_small_box2[jj]->id)
                {
                    // std::cout << "same point in small box 2, continue" << std::endl;
                    continue;
                }
                else
                {
                }

                Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                          state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                          state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
                double direction_norm = direction.norm();
                double force_middle_ = (cost_unorder_map[iter_vertices_map2_.second->id] +
                                        cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) /
                                       2;
                double weight_middle = (direction_norm + force_middle_) / 2;

                planning_boost_graph->removeEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id);
                planning_boost_graph->addEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
            }
        }
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 5 guan");
    }
    else
    {
        ROS_ERROR("[Explorer]: There is something wrong when running GetPlanningGraphForExploration.");
    }
}

void Explorer::GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                              GraphManager *source_graph_manager,
                                              Graph_For_Planning *planning_boost_graph,
                                              double LimitBoxX,
                                              double LimitBoxY,
                                              double LimitBoxZ_for_edge,
                                              int EdgesOfOnePointMax,
                                              double Local_Attractive_gain,
                                              double Local_Repulsive_gain,
                                              double Local_Bound_radiu)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> ObstacleVector_middle;
    ObstacleVector_middle.clear();
    // ROS_INFO("GetPlanningGraphForExploration_neibu di 1 guan");

    if (planning_boost_graph != NULL)
    {
        planning_boost_graph->clear();
    }

    const Vertex *vertex_source_ = source_graph_manager->getVertex(source_id_global);
    const Vertex *vertex_target_ = source_graph_manager->getVertex(target_id_global);

    const std::unordered_map<int, Vertex *> source_vertices_map_ = source_graph_manager->vertices_map_;

    if (source_vertices_map_.size() > 0)
    {

        for (auto &iter_vertices_map_ : source_vertices_map_)
        {
            int id = iter_vertices_map_.first;
            StateVec state_planning_new(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2], 0);
            // Vertex *vertex_planning_new;

            if (id == 0)
            {
                planning_boost_graph->addSourceVertex(id);
            }
            else if (planning_boost_graph->vertex_descriptors_.find(id) == planning_boost_graph->vertex_descriptors_.end())
            {
                planning_boost_graph->addVertex(id);
            }
            else
            {
            }

            if (iter_vertices_map_.second->is_bound)
            {
                Eigen::Vector3d obstacle(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2]);
                ObstacleVector_middle.push_back(obstacle);
            }
            else
            {
            }

            std::vector<Vertex *> nearest_vertices_in_small_box;
            source_graph_manager->getNearestVerticesInBox(&state_planning_new, LimitBoxX, LimitBoxY,
                                                          LimitBoxZ_for_edge, &nearest_vertices_in_small_box);
            if (nearest_vertices_in_small_box.size() > 9)
            {
                ROS_ERROR("[Explorer]: THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box in GetPlanningGraphForExploration function.");
                for (int ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
                {
                    std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
                    std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
                }
            }
            for (int j = 0; (j < nearest_vertices_in_small_box.size()) && (j < EdgesOfOnePointMax); ++j)
            {
                if (iter_vertices_map_.second->id == nearest_vertices_in_small_box[j]->id)
                {

                    continue;
                }

                Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                          state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                          state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
                double direction_norm = direction.norm();

                planning_boost_graph->removeEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id);
                planning_boost_graph->addEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id, direction_norm);
            }
        }
        // generate cost map
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 2 guan");
        std::vector<double> cost_map;
        std::unordered_map<int, double> cost_unorder_map;
        int robot_global_id_ = source_id_global;
        int target_global_id_ = target_id_global;
        Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
        Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
        double attractive_gain = Local_Attractive_gain;
        double repulsive_gain = Local_Repulsive_gain;
        double bound_radiu_ = Local_Bound_radiu;
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3 guan");

        //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
        cost_map.clear();
        cost_unorder_map.clear();
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3.5 guan");

        get_cost_map(source_graph_manager, planning_boost_graph, &cost_map, &cost_unorder_map,
                     robot_global_id_, target_global_id_,
                     robot_pos_, target_pos_,
                     attractive_gain, repulsive_gain, bound_radiu_,
                     ObstacleVector_middle);

        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 4 guan");

        // update boost edge
        for (auto &iter_vertices_map2_ : source_vertices_map_)
        {
            StateVec state_planning_new(iter_vertices_map2_.second->state[0],
                                        iter_vertices_map2_.second->state[1],
                                        iter_vertices_map2_.second->state[2], 0);
            std::vector<Vertex *> nearest_vertices_in_small_box2;
            source_graph_manager->getNearestVerticesInBox(&state_planning_new, LimitBoxX, LimitBoxY,
                                                          LimitBoxZ_for_edge, &nearest_vertices_in_small_box2);
            if (nearest_vertices_in_small_box2.size() > 9)
            {
                ROS_ERROR("[Explorer]: THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2 in GetPlanningGraphForExploration function.");
                ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
                for (int ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
                {
                    std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
                    std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
                }
            }
            else
            {
            }
            for (int jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < EdgesOfOnePointMax); ++jj)
            {

                if (iter_vertices_map2_.second->id == nearest_vertices_in_small_box2[jj]->id)
                {
                    // std::cout << "same point in small box 2, continue" << std::endl;
                    continue;
                }
                else
                {
                }

                Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                          state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                          state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
                double direction_norm = direction.norm();
                double force_middle_ = (cost_unorder_map[iter_vertices_map2_.second->id] +
                                        cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) /
                                       2;
                double weight_middle = (direction_norm + force_middle_) / 2;

                planning_boost_graph->removeEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id);
                planning_boost_graph->addEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
            }
        }
        // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 5 guan");
    }
    else
    {
        ROS_ERROR("[Explorer]: There is something wrong when running GetPlanningGraphForExploration.");
    }
}

bool Explorer::UpdatePlanningGraphForExploration(int source_id_global, int target_id_global,
                                                 double LimitBoxX,
                                                 double LimitBoxY,
                                                 double LimitBoxZ_for_edge,
                                                 int EdgesOfOnePointMax,
                                                 double Local_Attractive_gain,
                                                 double Local_Repulsive_gain,
                                                 double Local_Bound_radiu)
{
    std::lock_guard<std::mutex> lock(UpdateGraphManagerMutex_);
    std::lock_guard<std::mutex> lock2(UpdatePlanningBoostGraphMutex_);
    GetPlanningGraphForExploration(source_id_global, target_id_global,
                                   &GraphManager_, &PlanningBoostGraph_,
                                   LimitBoxX,
                                   LimitBoxY,
                                   LimitBoxZ_for_edge,
                                   EdgesOfOnePointMax,
                                   Local_Attractive_gain,
                                   Local_Repulsive_gain,
                                   Local_Bound_radiu);
    return true;
}

void Explorer::get_cost_map(GraphManager *source_graph_manager,
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
                            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector)
{
    // ROS_INFO("get_cost_map_neibu from source graph manager DI YI GUAN");
    std::vector<double> cost_map_;
    std::unordered_map<int, double> cost_unorder_map_;
    cost_unorder_map_.clear();
    cost_unorder_map->clear();
    cost_map_.clear();
    cost_map_.resize(planning_boost_graph->getNumVertices());
    cost_map->resize(planning_boost_graph->getNumVertices());
    int robot_internal_id_ = planning_boost_graph->get_id_internal(robot_global_id_);
    int target_internal_id_ = planning_boost_graph->get_id_internal(target_global_id_);
    // ROS_INFO("get_cost_map_neibu from source graph manager DI ER GUAN");
    //  Calculate the shortest path distance from each vertex to the goal
    std::vector<int> parent(planning_boost_graph->getNumVertices());
    std::vector<double> d(planning_boost_graph->getNumVertices());
    // boost::dijkstra_shortest_paths(graph_, vertex_descriptors_[target_global_id_], boost::predecessor_map(&parent[0]).distance_map(&d[0]));
    planning_boost_graph->dijkstra_process(target_global_id_, &parent, &d);

    // ROS_INFO("get_cost_map_neibu from source graph manager DI SAN GUAN");

    const Vertex *vertex_robot_ = source_graph_manager->getVertex(robot_global_id_);
    const Vertex *vertex_target_ = source_graph_manager->getVertex(target_global_id_);

    // ROS_INFO("get_cost_map_neibu from source graph manager DI SI GUAN");

    // Eigen::Vector3d robot_pos_(vertex_robot_->state[0], vertex_robot_->state[1], vertex_robot_->state[2]);
    // Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);

    std::pair<Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator, Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator> vi;
    planning_boost_graph->getVertexIterator(vi);
    for (Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator it = vi.first; it != vi.second; ++it)
    {
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.1 GUAN");
        int id = planning_boost_graph->get_id_global(planning_boost_graph->getVertexProperty(it));
        // int id = planning_boost_graph->getVertexProperty(it);
        //    What is the id? internal or global?. The following code is in the premise that it is global.
        const Vertex *vertex_temp_ = source_graph_manager->getVertex(id);
        // const Vertex *vertex_temp_ = Big_global_graph_with_obstacles_->getVertex(id);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.2 GUAN");
        // std::cout << "vertex_temp_ is " << vertex_temp_->state;
        Eigen::Vector3d point_pos_(vertex_temp_->state[0], vertex_temp_->state[1], vertex_temp_->state[2]);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.3 GUAN");
        Eigen::Vector3d total_force_;
        planning_boost_graph->get_total_force(point_pos_,
                                              target_pos_,
                                              bound_obstacle_vector,
                                              &total_force_,
                                              attractive_gain,
                                              repulsive_gain,
                                              bound_radiu_);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.4 GUAN");
        double total_force_norm = total_force_.norm();
        int id_internal_temp = planning_boost_graph->get_id_internal(id);
        double distance = d[id_internal_temp];
        cost_unorder_map_[id] = total_force_norm;
        cost_map_[id_internal_temp] = total_force_norm;
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.5 GUAN");
        // std::cout << "id is " << id << " and the total_force_norm is " << total_force_norm << std::endl;

        // cost_unorder_map_[id] = total_force_norm * distance;
        // cost_map_[id_internal_temp] = total_force_norm * distance;
    }
    *cost_unorder_map = cost_unorder_map_;
    *cost_map = cost_map_;
    // ROS_INFO("get_cost_map_neibu from source graph manager DI WU GUAN");
}

void Explorer::get_cost_map(GraphManager *source_graph_manager,
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
                            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector)
{
    // ROS_INFO("get_cost_map_neibu from source graph manager DI YI GUAN");
    std::vector<double> cost_map_;
    std::unordered_map<int, double> cost_unorder_map_;
    cost_unorder_map_.clear();
    cost_unorder_map->clear();
    cost_map_.clear();
    cost_map_.resize(planning_boost_graph->getNumVertices());
    cost_map->resize(planning_boost_graph->getNumVertices());
    int robot_internal_id_ = planning_boost_graph->get_id_internal(robot_global_id_);
    int target_internal_id_ = planning_boost_graph->get_id_internal(target_global_id_);
    // ROS_INFO("get_cost_map_neibu from source graph manager DI ER GUAN");
    //  Calculate the shortest path distance from each vertex to the goal
    std::vector<int> parent(planning_boost_graph->getNumVertices());
    std::vector<double> d(planning_boost_graph->getNumVertices());
    // boost::dijkstra_shortest_paths(graph_, vertex_descriptors_[target_global_id_], boost::predecessor_map(&parent[0]).distance_map(&d[0]));
    planning_boost_graph->dijkstra_process(target_global_id_, &parent, &d);

    // ROS_INFO("get_cost_map_neibu from source graph manager DI SAN GUAN");

    const Vertex *vertex_robot_ = source_graph_manager->getVertex(robot_global_id_);
    const Vertex *vertex_target_ = source_graph_manager->getVertex(target_global_id_);

    // ROS_INFO("get_cost_map_neibu from source graph manager DI SI GUAN");

    // Eigen::Vector3d robot_pos_(vertex_robot_->state[0], vertex_robot_->state[1], vertex_robot_->state[2]);
    // Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);

    std::pair<Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator, Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator> vi;
    planning_boost_graph->getVertexIterator(vi);
    for (Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator it = vi.first; it != vi.second; ++it)
    {
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.1 GUAN");
        int id = planning_boost_graph->get_id_global(planning_boost_graph->getVertexProperty(it));
        // int id = planning_boost_graph->getVertexProperty(it);
        //    What is the id? internal or global?. The following code is in the premise that it is global.
        const Vertex *vertex_temp_ = source_graph_manager->getVertex(id);
        // const Vertex *vertex_temp_ = Big_global_graph_with_obstacles_->getVertex(id);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.2 GUAN");
        // std::cout << "vertex_temp_ is " << vertex_temp_->state;
        Eigen::Vector3d point_pos_(vertex_temp_->state[0], vertex_temp_->state[1], vertex_temp_->state[2]);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.3 GUAN");
        Eigen::Vector3d total_force_;
        planning_boost_graph->get_total_force(point_pos_,
                                              target_pos_,
                                              bound_obstacle_vector,
                                              &total_force_,
                                              attractive_gain,
                                              repulsive_gain,
                                              bound_radiu_);
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.4 GUAN");
        double total_force_norm = total_force_.norm();
        int id_internal_temp = planning_boost_graph->get_id_internal(id);
        double distance = d[id_internal_temp];
        cost_unorder_map_[id] = total_force_norm;
        cost_map_[id_internal_temp] = total_force_norm;
        // ROS_INFO("get_cost_map_neibu from source graph manager DI SI.5 GUAN");
        // std::cout << "id is " << id << " and the total_force_norm is " << total_force_norm << std::endl;

        // cost_unorder_map_[id] = total_force_norm * distance;
        // cost_map_[id_internal_temp] = total_force_norm * distance;
    }
    *cost_unorder_map = cost_unorder_map_;
    *cost_map = cost_map_;
    // ROS_INFO("get_cost_map_neibu from source graph manager DI WU GUAN");
}
GraphManager Explorer::getGraphManager()
{
    return GraphManager_;
}

bool Explorer::updateObstaclesAttribute(std::vector<int> point_ids_global, double Size_X_2, double Size_Y_2, double Size_Z_2)
{
    std::lock_guard<std::mutex> lock3(UpdateBigWholeGraphMutex_);
    id_obstacles_distance.clear();
    if (point_ids_global.size() == 0)
    {
        ROS_ERROR("The size of ids is zero when update the obstacles attribute.");
        return false;
    }
    else
    {
    }
    for (int i = 0; i < point_ids_global.size(); i++)
    {
        int GlobalId_middle_ = point_ids_global[i];
        Vertex *TargetVertex = BigWholeGraph_.getVertex(GlobalId_middle_);
        StateVec TargerState_ = TargetVertex->state;
        std::vector<Vertex *> nearest_vertices_in_BigBig_box;
        BigWholeGraph_.getNearestVerticesInBox(&TargerState_, Size_X_2, Size_Y_2, Size_Z_2, &nearest_vertices_in_BigBig_box);
        int obstacles_size = 0;
        for (int i = 0; i < nearest_vertices_in_BigBig_box.size(); i++)
        {
            if (nearest_vertices_in_BigBig_box[i]->is_obstacle)
            {
                obstacles_size++;
            }
        }
        id_obstacles_distance[GlobalId_middle_] = obstacles_size;
    }
    return true;
}

std::unordered_map<int, double> Explorer::get_ObstaclesAttribute_map()
{
    return id_obstacles_distance;
}

void Explorer::ComputeExplorationGain(std::unordered_map<int, ExplorationGainVec> *IdsAndAttribute,
                                      double Lambda_Size_, double Lambda_Distance_, double Lambda_Obstacle_, double Lambda_ZDistance_)
{
    // Id--Size,Distance,Obstacles.
    for (auto &Iterator_ : *IdsAndAttribute)
    {
        int Id_ = Iterator_.first;
        ExplorationGainVec Attribute_ = Iterator_.second;
        if (Attribute_[0] > 15)
        {
            Attribute_[0] = 15;
        }
        double ExplorationGain_ = Lambda_Size_ * Attribute_[0] * exp((-Lambda_Distance_) * Attribute_[1]) * exp((-Lambda_Obstacle_) * Attribute_[2]) * exp((-Lambda_ZDistance_) * Attribute_[3]);
        IdsAndAttribute->at(Id_)[4] = ExplorationGain_;
    }
}
