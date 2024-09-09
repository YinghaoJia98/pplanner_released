#include "pplanner/pplanner.h"

PPlanner::PPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{

  global_graph_ = new GraphManager();
  local_graph_.reset(new GraphManager());
  RoughLocalGraph_.reset(new GraphManager());
  RoughGlobalGraph_.reset(new GraphManager());
  frontier_graph_.reset(new GraphManager());
  frontier_boost_graph_.reset(new Graph_For_Frontier());
  graph_for_planning_.reset(new GraphManager());
  planning_boost_graph_.reset(new Graph_For_Planning);
  ppath_tracker_.reset(new Ppath_Tracker(nh_, nh_private_));
  Local_ppath_tracker_.reset(new Ppath_Tracker(nh_, nh_private_));

  local_graph_for_safety_with_planning_.reset(new GraphManager());
  planning_boost_graph_for_local_planner_.reset(new Graph_For_Planning);

  Big_global_graph_with_obstacles_.reset(new GraphManager());

  UF_.reset(new UF(1));

  Explorer_.reset(new Explorer(nh_, nh_private_));
  planning_boost_graph_for_exploration_.reset(new Graph_For_Planning);
  Initial_Path_tracker_.reset(new Ppath_Tracker(nh_, nh_private_));
  WholeGloabalGraphWithObstacles_pub.reset(new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("pplanner/WholeGlobalGrpahWithObstacles", 1)));
  RoughGlobalGraph_pub.reset(new ros::Publisher(nh_.advertise<visualization_msgs::Marker>("pplanner/RoughGlobalGraph", 1)));
  // local_kdtree_ptr.reset(new KD_TREE<PointType>(0, 0, 0));
  initialize();
  get_param();
  // point_set = new std::set<Eigen::Vector3d, classcomp>;
  // point_set->clear();

  pplanner_vector_map_.clear();
  // pplanner_vector_map_for_local_map_.clear();
  path_tracked_vector_.clear();
  if_map_initialled_ = false;
  is_turning = false;
  if_update_local_planning_graph = false;
  if_local_planning_graph_updated = false;
  tracker_wait_switch = false;
  global_target_id = 0;
  global_path_tracker_is_paused_by_user = false;
  local_path_tracker_is_paused_by_user = false;
  pos_seq = 0;

  if_exploration = false;
  if_waiting_for_updating_target_ = false;
  if_target_updated_ = false;

  IfNeedHandleEmergencyByTurnAround_ = false;

  IfNeedUpdateFrontierSubmap_ = false;
  IfNeedUpdateGlobalGraphForUpdatingTarget_ = false;
  IfNeedUpdateFrontireGraphForUpdatingTarget_ = false;

  BadGlobalTargetIdVector_.clear();
  TimesOfGlobalTargetIdRepetition_ = 0;
  MembershipDegreeM_ << 1, -0.1, -0.1, -0.1,
      0, 1, 0, 0,
      0, -0.3, 1, 0,
      0, 0 - 0.1, -0.1, 1;
  MembershipDegreeN_ << 0.1, -0.1, -0.1, -0.1,
      0, 0.1, 0, 0,
      0, 0, 0.1, 0,
      0.01, 0.01, 0, 0.1;

  RobotHeight_ = 0.3;

  // global_ids_issued_.clear();
  // global_points_issued_.clear();
}
void PPlanner::get_param()
{

  nh_.param<double>("pplanner_settings/tra_map_sizex", tra_map_sizex, 6.0);
  nh_.param<double>("pplanner_settings/tra_map_sizey", tra_map_sizey, 6.0);
  nh_.param<double>("pplanner_settings/tra_map_resolution", tra_map_resolution, 0.05);
  nh_.param<int>("pplanner_settings/edges_of_one_point_max", edges_of_one_point_max, 10);
  nh_.param<double>("pplanner_settings/pplaner_map_resolution", pplaner_map_resolution, 0.3);
  nh_.param<double>("pplanner_settings/traversability_score_threshold_local_graph",
                    traversability_score_threshold_local_graph, -0.6);
  nh_.param<double>("pplanner_settings/traversability_score_threshold_local_graph_for_frontier",
                    traversability_score_threshold_local_graph_for_frontier, 0.3);
  nh_.param<double>("pplanner_settings/traversability_score_threshold_global_graph",
                    traversability_score_threshold_global_graph, -0.6);
  nh_.param<double>("pplanner_settings/traversability_score_threshold_low_bound",
                    traversability_score_threshold_low_bound, -0.6);
  nh_.param<int>("pplanner_settings/bad_point_threshold", bad_point_threshold, 6);
  nh_.param<int>("pplanner_settings/worst_point_threshold", worst_point_threshold_, 1);
  nh_.param<bool>("pplanner_settings/if_bad_point_threshold_improve_automatically",
                  if_bad_point_threshold_improve_automatically, true);
  nh_.param<int>("pplanner_settings/point_threshold", point_threshold, 6);
  nh_.param<int>("pplanner_settings/point_threshold_frontier", point_threshold_frontier, 6);
  nh_.param<double>("pplanner_settings/limit_box_z", limit_box_z, 0.5);
  nh_.param<double>("pplanner_settings/limit_box_z_for_edge", limit_box_z_for_edge, 0.3);
  nh_.param<bool>("pplanner_settings/if_grid_cells_pub", if_grid_cells_pub, true);
  nh_.param<bool>("pplanner_settings/if_iterator_map_with_map_Callback",
                  if_iterator_map_with_map_Callback, true);
  nh_.param<bool>("pplanner_settings/if_edge_pub", if_edge_pub, false);
  nh_.param<bool>("pplanner_settings/if_vertex_marker_also_pub", if_vertex_marker_also_pub, false);

  nh_.param<double>("pplanner_settings/attractive_gain", Attractive_gain_, 0.0);
  nh_.param<double>("pplanner_settings/repulsive_gain", Repulsive_gain_, 0.01);
  nh_.param<double>("pplanner_settings/bound_radiu_", Bound_radiu_, 0.6);

  nh_.param<bool>("pplanner_settings/if_plus_", if_plus_, true);
  nh_.param<double>("pplanner_settings/x_plus_", x_plus_, 2.0);
  nh_.param<double>("pplanner_settings/y_plus_", y_plus_, 2.0);
  nh_.param<double>("pplanner_settings/z_plus_", z_plus_, 0.3);
  nh_.param<int>("pplanner_settings/Max_plus_count", Max_plus_count, 5);

  nh_.param<double>("pplanner_settings/lookahead_time_for_safety_circumvent_",
                    lookahead_time_for_safety_circumvent_, 1.0);

  nh_.param<double>("pplanner_settings/x_forward_offset_for_safety_ckeck_",
                    x_forward_offset_for_safety_ckeck_, 0.15);
  nh_.param<double>("pplanner_settings/x_backward_offset_for_safety_ckeck_",
                    x_backward_offset_for_safety_ckeck_, 0.35);
  nh_.param<double>("pplanner_settings/robot_width", robot_width, 0.5);

  nh_.param<int>("pplanner_settings/bad_point_threshold_for_check_safety",
                 bad_point_threshold_for_check_safety, 3);

  nh_.param<double>("pplanner_settings/local_attractive_gain", Local_Attractive_gain_, 0.0);
  nh_.param<double>("pplanner_settings/local_repulsive_gain", Local_Repulsive_gain_, 0.01);
  nh_.param<double>("pplanner_settings/local_bound_radiu_", Local_Bound_radiu_, 0.6);

  nh_.param<std::string>("pplanner_settings/world_frame", World_frame_, std::string("world"));
  nh_.param<std::string>("pplanner_settings/track_frame", Track_frame_, std::string("base_link"));

  nh_.param<double>("pplanner_settings/LambdaSize", LambdaSize_, 0.6);
  nh_.param<double>("pplanner_settings/LambdaDistance", LambdaDistance_, 0.6);
  nh_.param<double>("pplanner_settings/LambdaObstacle", LambdaObstacle_, 0.6);
  nh_.param<double>("pplanner_settings/LambdaZDistance", LambdaZDistance_, 0.6);

  nh_.param<bool>("pplanner_settings/IfUsePathDistanceInsteadOfEuclideanDistance", IfUsePathDistanceInsteadOfEuclideanDistance_, false);

  nh_.param<bool>("pplanner_settings/IfAvoidObstaclesByChangeVelocity", IfAvoidObstaclesByChangeVelocity_, false);

  nh_.param<int>("pplanner_settings/TimesOfGlobalTargetIdRepetitionThreshold", TimesOfGlobalTargetIdRepetitionThreshold_, 2);

  nh_.param<double>("pplanner_settings/PplannerRoughGlobalMapResolution", PplannerRoughGlobalMapResolution_, 0.5);

  int bad_point_threshold_from_auto = 2 * pplaner_map_resolution / tra_map_resolution;

  if (if_bad_point_threshold_improve_automatically)
  {
    if (bad_point_threshold != bad_point_threshold_from_auto)
    {
      ROS_INFO("bad_point_threshold has been changed to %d automatically", bad_point_threshold_from_auto);
      bad_point_threshold = bad_point_threshold_from_auto;
    }
  }

  limit_box_x = pplaner_map_resolution + 0.2 * pplaner_map_resolution;
  limit_box_y = pplaner_map_resolution + 0.2 * pplaner_map_resolution;

  tra_map_bound_x_max = tra_map_sizex / 2;  // - tra_map_resolution;
  tra_map_bound_x_min = -tra_map_sizex / 2; // + tra_map_resolution;
  tra_map_bound_y_max = tra_map_sizey / 2;  // - tra_map_resolution;
  tra_map_bound_y_min = -tra_map_sizey / 2; // +tra_map_resolution;

  gridcells_msg_.header.seq = 0;
  gridcells_msg_.header.stamp = ros::Time::now();
  gridcells_msg_.header.frame_id = Grid_cell_frame_.c_str();
  gridcells_msg_.cell_height = pplaner_map_resolution;
  gridcells_msg_.cell_width = pplaner_map_resolution;

  gridcells_frontier_msg_.header.seq = 0;
  gridcells_frontier_msg_.header.stamp = ros::Time::now();
  gridcells_frontier_msg_.header.frame_id = Grid_cell_frame_.c_str();
  gridcells_frontier_msg_.cell_height = pplaner_map_resolution;
  gridcells_frontier_msg_.cell_width = pplaner_map_resolution;

  gridcells_bound_msg_.header.seq = 0;
  gridcells_bound_msg_.header.stamp = ros::Time::now();
  gridcells_bound_msg_.header.frame_id = Grid_cell_frame_.c_str();
  gridcells_bound_msg_.cell_height = pplaner_map_resolution;
  gridcells_bound_msg_.cell_width = pplaner_map_resolution;

  Path_Robot_Have_Passed_Msg_.header.seq = 0;
  Path_Robot_Have_Passed_Msg_.header.stamp = ros::Time::now();
  Path_Robot_Have_Passed_Msg_.header.frame_id = Grid_cell_frame_.c_str();
  Path_Robot_Have_Passed_Msg_.poses.clear();

  if (if_grid_cells_pub)
  {
    double global_grph_publish_fps;
    nh_.param<double>("pplanner_settings/pplanner_map_publish_fps", global_grph_publish_fps, 1.0);
    double duration = 1.0 / (global_grph_publish_fps + 0.00001);
    global_graph_visualize_timer_ = nh_.createTimer(ros::Duration(duration),
                                                    &PPlanner::visualizeGlobalGraphTimerCallback, this);
  }

  if (!if_iterator_map_with_map_Callback)
  {
    double iterator_map_fps;
    nh_.param<double>("pplanner_settings/iterator_map_fps", iterator_map_fps, 1.0);
    double duration = 1.0 / (iterator_map_fps + 0.00001);
    iterator_map_timer_ = nh_.createTimer(ros::Duration(duration),
                                          &PPlanner::iteratorMapTimerCallback, this);
  }

  if (1)
  {
    double map_of_frontier_update_fps;
    nh_.param<double>("pplanner_settings/map_of_frontier_update_fps", map_of_frontier_update_fps, 1.0);
    double duration_frontier_update_ = 1.0 / (map_of_frontier_update_fps + 0.00001);
    map_of_frontier_update_timer_ = nh_.createTimer(ros::Duration(duration_frontier_update_),
                                                    &PPlanner::update_frontier_map, this);
  }
  if (1)
  {
    double submap_of_frontier_update_fps;
    nh_.param<double>("pplanner_settings/submap_of_frontier_update_fps", submap_of_frontier_update_fps, 1.0);
    double duration_frontiersub_update_ = 1.0 / (submap_of_frontier_update_fps + 0.00001);
    submap_of_frontier_update_timer_ = nh_.createTimer(ros::Duration(duration_frontiersub_update_),
                                                       &PPlanner::update_frontier_submap, this);
  }
  if (1)
  {
    double map_of_bound_update_fps;
    nh_.param<double>("pplanner_settings/map_of_bound_update_fps", map_of_bound_update_fps, 1.0);
    double duration_bound_update_ = 1.0 / (map_of_bound_update_fps + 0.00001);
    map_of_bound_update_timer_ = nh_.createTimer(ros::Duration(duration_bound_update_),
                                                 &PPlanner::update_bound_map, this);
  }

  if (1)
  {
    double orientation_of_tracker_update_fps;
    nh_.param<double>("pplanner_settings/Orientation_of_tracker_update_fps",
                      orientation_of_tracker_update_fps, 1.0);
    double duration_tracker_update_ = 1.0 / (orientation_of_tracker_update_fps + 0.00001);
    path_tracker_orientation_update_timer_ = nh_.createTimer(ros::Duration(duration_tracker_update_),
                                                             &PPlanner::update_tracker_orientation, this);
  }

  if (IfAvoidObstaclesByChangeVelocity_)
  {
    double safety_circumvent_of_tracker_update_fps;
    nh_.param<double>("pplanner_settings/Safety_circumvent_of_tracker_update_fps",
                      safety_circumvent_of_tracker_update_fps, 1.0);
    double duration_tracker_safety_circumvent_update_ =
        1.0 / (safety_circumvent_of_tracker_update_fps + 0.00001);
    path_tracker_safety_circumvent_update_timer_ =
        nh_.createTimer(ros::Duration(duration_tracker_safety_circumvent_update_),
                        &PPlanner::update_tracker_path_safety_circumvent, this);
  }

  if (1)
  {
    double Local_Planner_fps;
    nh_.param<double>("pplanner_settings/Local_Planner_fps", Local_Planner_fps, 1.0);
    double duration_local_planner_ = 1.0 / (Local_Planner_fps + 0.00001);
    Local_planner_timer_ = nh_.createTimer(ros::Duration(duration_local_planner_),
                                           &PPlanner::LocalPlannerTimerCallBack, this);
  }

  if (1)
  {
    double UpdateLocalGraphManagerFps;
    nh_.param<double>("pplanner_settings/Update_Local_Graph_Manager_fps", UpdateLocalGraphManagerFps, 1.0);
    double duration_Update_Local_Graph_Manager_ = 1.0 / (UpdateLocalGraphManagerFps + 0.00001);
    UpdateLocalGraphManagerTimer = nh_.createTimer(ros::Duration(duration_Update_Local_Graph_Manager_),
                                                   &PPlanner::UpdateLocalGraphManagerTimerCallBack, this);
  }

  if (1)
  {
    double PathTrackerStateCheckFps;
    nh_.param<double>("pplanner_settings/PathTrackerStateCheckFps", PathTrackerStateCheckFps, 1.0);
    double duration_PathTrackerState_ = 1.0 / (PathTrackerStateCheckFps + 0.00001);
    PathTrackerSwitchTimer = nh_.createTimer(ros::Duration(duration_PathTrackerState_),
                                             &PPlanner::PathTrackerSwitchTimerCallBack, this);
  }

  if (1)
  {
    double VisulaizeRobotPathPassedFps;
    nh_.param<double>("pplanner_settings/VisulaizeRobotPathPassedFps", VisulaizeRobotPathPassedFps, 5.0);
    double duration_VisulaizeRobotPathPassed_ = 1.0 / (VisulaizeRobotPathPassedFps + 0.00001);
    VisualizeRobotPathPassedTimer = nh_.createTimer(ros::Duration(duration_VisulaizeRobotPathPassed_),
                                                    &PPlanner::VisualizeRobotPathPassedTimerCallBack, this);
  }

  if (0)
  {

    double VisualizePointsIssuedFps;
    nh_.param<double>("pplanner_settings/VisualizePointsIssuedFps", VisualizePointsIssuedFps, 5.0);
    double duration_VisualizePointsIssued_ = 1.0 / (VisualizePointsIssuedFps + 0.00001);
    VisualizePointsIssuedTimer = nh_.createTimer(ros::Duration(duration_VisualizePointsIssued_),
                                                 &PPlanner::VisualizePointsIssuedTimerCallBack, this);
  }

  if (1)
  {
    double ExplorerFps;
    nh_.param<double>("pplanner_settings/ExplorerFps", ExplorerFps, 1.0);
    double duration_ExplorerFps_ = 1.0 / (ExplorerFps + 0.00001);
    ExplorerTimer = nh_.createTimer(ros::Duration(duration_ExplorerFps_),
                                    &PPlanner::ExplorerTimerCallBack, this);
  }

  if (1)
  {
    double UpdateTargetFps;
    nh_.param<double>("pplanner_settings/UpdateTargetFps", UpdateTargetFps, 1.0);
    double duration_UpdateTarget_ = 1.0 / (UpdateTargetFps + 0.00001);
    UpdateTargetTimer = nh_.createTimer(ros::Duration(duration_UpdateTarget_),
                                        &PPlanner::UpdateTargetTimerCallBack, this);
  }

  if (1)
  {
    double UpdateExplorerGraphManagerFps;
    nh_.param<double>("pplanner_settings/UpdateExplorerGraphManagerFps", UpdateExplorerGraphManagerFps, 1.0);
    double duration_UpdateExplorerGraphManager_ = 1.0 / (UpdateExplorerGraphManagerFps + 0.00001);
    UpdateExplorerGraphManagerTimer = nh_.createTimer(ros::Duration(duration_UpdateExplorerGraphManager_),
                                                      &PPlanner::UpdateExplorerGraphManagerTimerCallBack, this);
  }

  if (1)
  {
    double HandleEmergencyByTurnAroundFps_;
    nh_.param<double>("pplanner_settings/HandleEmergencyByTurnAroundFps", HandleEmergencyByTurnAroundFps_, 1.0);
    double duration_HandleEmergencyByTurnAroundFps_ = 1.0 / (HandleEmergencyByTurnAroundFps_ + 0.00001);
    HandleEmergencyByTurnAroundTimer = nh_.createTimer(ros::Duration(duration_HandleEmergencyByTurnAroundFps_),
                                                       &PPlanner::HandleEmergencyByTurnAroundTimerCallBack, this);
  }
}

void PPlanner::initialize()
{
  std::string Grid_map_topic_, Grid_cell_topic_, Grid_cells_of_frontier_topic_;
  std::string Grid_cells_of_bound_topic_, cmd_topic_;
  nh_.param<std::string>("pplanner_settings/grid_map_topic", Grid_map_topic_,
                         std::string("/grid_map_simple_demo/grid_map"));
  nh_.param<std::string>("pplanner_settings/grid_cell_topic", Grid_cell_topic_,
                         std::string("/free_grid_cells"));
  nh_.param<std::string>("pplanner_settings/grid_cells_of_frontier_topic_",
                         Grid_cells_of_frontier_topic_, std::string("/frontier_grid_cells"));
  nh_.param<std::string>("pplanner_settings/grid_cells_of_bound_topic_",
                         Grid_cells_of_bound_topic_, std::string("/bound_grid_cells"));

  nh_.param<std::string>("pplanner_settings/grid_cell_frame", Grid_cell_frame_, std::string("world"));
  nh_.param<std::string>("pplanner_settings/elevation_layer", Elevation_layer_, std::string("elevation"));
  nh_.param<std::string>("pplanner_settings/traversability_layer", Traversability_layer_,
                         std::string("traversability"));
  nh_.param<std::string>("pplanner_settings/step_layer", Step_layer_, std::string("step"));
  nh_.param<std::string>("pplanner_settings/traversability_supplementary_layer",
                         Traversability_supplementary_layer_, std::string("traversability"));
  nh_.param<std::string>("pplanner_settings/variance_layer",
                         Variance_layer_, std::string("variance"));

  gridmap_traversability_subscriber_ = nh_.subscribe(Grid_map_topic_.c_str(), 1,
                                                     &PPlanner::traverability_map_Callback, this);
  grid_cells_pub_ = nh_.advertise<nav_msgs::GridCells>(Grid_cell_topic_.c_str(), 1);
  grid_cells_of_frontier_pub_ = nh_.advertise<nav_msgs::GridCells>(Grid_cells_of_frontier_topic_.c_str(), 1);
  grid_cells_of_bound_pub_ = nh_.advertise<nav_msgs::GridCells>(Grid_cells_of_bound_topic_.c_str(), 1);

  pplanner_graph_pub_ = nh_.advertise<visualization_msgs::Marker>("pplanner/marker_graph", 10);
  frontier_submap_pub = nh_.advertise<visualization_msgs::MarkerArray>("pplanner/frontier_submap", 10);
  points_for_pre_exploration_pub =
      nh_.advertise<visualization_msgs::MarkerArray>("pplanner/points_pre_exploration", 10);

  GraphManager_pub_ = nh_.advertise<visualization_msgs::Marker>("pplanner/GraphManager", 10);
  Global_Path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("pplanner/GlobalPath", 10);
  Planning_Boost_pub_ = nh_.advertise<visualization_msgs::Marker>("pplanner/PlanningBoostGraph", 10);

  LocalPlanningGraphManager_pub_ =
      nh_.advertise<visualization_msgs::Marker>("pplanner/LocalPlanningGraphManager", 10);
  Local_Path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("pplanner/LocalPath", 10);
  Local_Planning_Boost_Graph_pub_ = nh_.advertise<visualization_msgs::Marker>("pplanner/LocalBoostGraph", 10);

  Path_Robot_have_Passed_pub_ = nh_.advertise<nav_msgs::Path>("pplanner/RobotPassedPath", 10);

  points_issued_pub = nh_.advertise<visualization_msgs::Marker>("pplanner/points_issued", 10);

  // WholeGloabalGraphWithObstacles_pub = nh_.advertise<visualization_msgs::Marker>("pplanner/WholeGlobalGrpahWithObstacles", 10);

  pplanner_gloabal_path_planning_server_ = nh_.advertiseService(
      "planner_global", &PPlanner::globalPlannerCallback, this);

  pplanner_gloabal_path_pause_server_ = nh_.advertiseService(
      "pause_tracker", &PPlanner::stdPausePathTrackerCallback, this);

  pplanner_gloabal_path_continue_server_ = nh_.advertiseService(
      "continue_tracker", &PPlanner::stdContinuePathTrackerCallback, this);

  pplanner_gloabal_path_stop_server_ = nh_.advertiseService(
      "stop_tracker", &PPlanner::stdStopPathTrackerCallback, this);

  pplanner_start_to_explore_server_ = nh_.advertiseService(
      "start_to_explore", &PPlanner::stdStartToExploreCallback, this);

  pplanner_stop_exploring_server_ = nh_.advertiseService(
      "stop_exploring", &PPlanner::stdStopExploringCallback, this);

  pplanner_initialization_server_ = nh_.advertiseService(
      "pplanner_initialization", &PPlanner::stdInitializationCallback, this);

  pplanner_UpdateTargetPoint_server_ = nh_.advertiseService(
      "pplanner_UpdateTargetPoint", &PPlanner::stdUpdateTargetPointCallback, this);

  nh_.param<std::string>("pplanner_tracker_settings/cmd_topic_", cmd_topic_, "/cmd_vel");
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_.c_str(), 10);
}

void PPlanner::iterator_map(grid_map::GridMap traversability_map_fun)
{
  // ros::Time t_iterator_start = ros::Time::now();
  pplanner_vector_map_.clear();

  if (local_graph_ != NULL)
  {
    local_graph_->reset();
  }

  if (RoughLocalGraph_ != NULL)
  {
    RoughLocalGraph_->reset();
  }
  std::unordered_map<VOXEL_LOC, std::vector<Eigen::VectorXd>, std::hash<VOXEL_LOC>, std::equal_to<VOXEL_LOC>, Eigen::aligned_allocator<std::pair<const VOXEL_LOC, Eigen::VectorXd>>> RoughPplanner_vector_map_;
  RoughPplanner_vector_map_.clear();
  ros::Time t_iterator_local_start = ros::Time::now();

  for (grid_map::GridMapIterator iterator(traversability_map_fun); !iterator.isPastEnd(); ++iterator)
  {
    // ros::Time t_iterator_local_once_start = ros::Time::now();
    traversability_map_fun.getPosition(*iterator, position);
    double elevation = traversability_map_fun.at(Elevation_layer_.c_str(), *iterator);
    double traversability_score = traversability_map_fun.at(Traversability_layer_.c_str(), *iterator);
    double traversability_supplementary_score =
        traversability_map_fun.at(Traversability_supplementary_layer_.c_str(), *iterator);
    double variance_middle_ = traversability_map_fun.at(Variance_layer_.c_str(), *iterator);
    // traversability_score = std::max(traversability_score, traversability_supplementary_score);
    // remove the point whose value is Nan;
    if (elevation != elevation)
    {
      continue;
    }

    if (traversability_score != traversability_score)
    {
      continue;
    }

    if (traversability_supplementary_score != traversability_supplementary_score)
    {
      continue;
    }

    traversability_score = std::max(traversability_score, traversability_supplementary_score);
    // traversability_score = traversability_score - 1.0;
    //  if (traversability_score > traversability_score_threshold_local_graph)
    //  {
    int row;
    int col;
    if (position[0] < -0.5 * pplaner_map_resolution)
    {
      row = (position[0] + pplaner_map_resolution / 2) / pplaner_map_resolution - 1;
    }
    else
    {
      row = (position[0] + pplaner_map_resolution / 2) / pplaner_map_resolution;
    }

    if (position[1] < -0.5 * pplaner_map_resolution)
    {
      col = (position[1] + pplaner_map_resolution / 2) / pplaner_map_resolution - 1;
    }
    else
    {
      col = (position[1] + pplaner_map_resolution / 2) / pplaner_map_resolution;
    }

    int RoughRow;
    int RoughCol;
    if (position[0] < -0.5 * PplannerRoughGlobalMapResolution_)
    {
      RoughRow = (position[0] + PplannerRoughGlobalMapResolution_ / 2) / PplannerRoughGlobalMapResolution_ - 1;
    }
    else
    {
      RoughRow = (position[0] + PplannerRoughGlobalMapResolution_ / 2) / PplannerRoughGlobalMapResolution_;
    }

    if (position[1] < -0.5 * PplannerRoughGlobalMapResolution_)
    {
      RoughCol = (position[1] + PplannerRoughGlobalMapResolution_ / 2) / PplannerRoughGlobalMapResolution_ - 1;
    }
    else
    {
      RoughCol = (position[1] + PplannerRoughGlobalMapResolution_ / 2) / PplannerRoughGlobalMapResolution_;
    }

    StateVec state_new(row * pplaner_map_resolution, col * pplaner_map_resolution, elevation, 0);
    StateVec RoughState_new(RoughRow * PplannerRoughGlobalMapResolution_, RoughCol * PplannerRoughGlobalMapResolution_, elevation, 0);
    Eigen::VectorXd point_new_(7);
    // X,Y,Z,Traversability,if_is_frontier,if_is_bound, variance.
    point_new_ << state_new[0], state_new[1], state_new[2], traversability_score, 0, 0, variance_middle_;

    Eigen::VectorXd RoughPointNew_(5);
    RoughPointNew_ << RoughState_new[0], RoughState_new[1], RoughState_new[2], traversability_score, 0;

    bool vertex_need_to_be_frontier_ = false;
    bool if_point_valid = true;
    bool if_point_bound = false;
    try
    {
      // ros::Time t_if_point_internal_start = ros::Time::now();
      vertex_need_to_be_frontier_ =
          !(if_point_internal(position[0], position[1],
                              traversability_map_fun, &if_point_valid, &if_point_bound));
      // ros::Time t_if_point_internal_end = ros::Time::now();
      // double t_used_if_point_internal = t_if_point_internal_end.toSec() - t_if_point_internal_start.toSec();
      // ROS_INFO("Time used for if_point_internal is %f", t_used_if_point_internal);
      // ROS_ERROR("if_point_valid is %d", if_point_valid);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("There is something wrong when check if point is internal.");
      vertex_need_to_be_frontier_ = true;
    }

    if (!if_point_valid)
    {
      // ROS_ERROR("if_point_valid");
      // point's neighborhood is nan, not a number.
      continue;
    }

    if (if_point_bound)
    {
      point_new_[5] = 1;
    }
    else
    {
      point_new_[5] = 0;
    }

    if (vertex_need_to_be_frontier_)
    {
      point_new_[4] = 1;
      RoughPointNew_[4] = 1;
    }
    bool frontier_point_valid = false;
    if ((vertex_need_to_be_frontier_) &&
        ((traversability_score > traversability_score_threshold_local_graph_for_frontier) ||
         (traversability_supplementary_score > traversability_score_threshold_local_graph_for_frontier)))
    {
      frontier_point_valid = true;
    }
    // if ((traversability_score > traversability_score_threshold_local_graph) || ((vertex_need_to_be_frontier_) && (traversability_score > traversability_score_threshold_local_graph_for_frontier)))
    if ((traversability_score > traversability_score_threshold_local_graph) || (frontier_point_valid))
    {
      std::vector<Vertex *> nearest_vertices;
      nearest_vertices.clear();

      VOXEL_LOC LocalPositionTem_(state_new[0], state_new[1], 0);
      pplanner_vector_map_[LocalPositionTem_].push_back(point_new_);
      // if (vertex_exist(state_new, &nearest_vertices))

      VOXEL_LOC LocalRoughPositionTem_(RoughState_new[0], RoughState_new[1], 0);
      RoughPplanner_vector_map_[LocalRoughPositionTem_].push_back(RoughPointNew_);
    }
    else
    {
      continue;
    }

    // ros::Time t_iterator_local_once_end = ros::Time::now();
    // double t_used_locl_once = t_iterator_local_once_end.toSec() - t_iterator_local_once_start.toSec();
    // ROS_INFO("Time used for iterator local map once is %f", t_used_locl_once);
  }
  ros::Time t_iterator_local_end = ros::Time::now();
  double t_used_locl = t_iterator_local_end.toSec() - t_iterator_local_start.toSec();
  if (t_used_locl > 0.5)
  {
    ROS_INFO("Time used for iterator local map is %f", t_used_locl);
  }

  if ((pplanner_vector_map_.size() > 0) && true)
  {

    // for (unordered_map<int, std::vector<Eigen::Vector4d>>::iterator iter_planner_map_ = pplanner_vector_map_.begin(); iter_planner_map_ != pplanner_vector_map_.end(); ++iter_planner_map_)
    for (auto &iter_pplanner_map_ : pplanner_vector_map_)
    {
      // int id = iter_pplanner_map_.first;
      double elevation_post_ = 0.0;
      double traversability_scoal_post_ = 0.0;
      double variance_post_ = 0.0;
      double traversability_with_covariance_for_post_ = 0.0;
      double elevation_with_covariance_for_post_ = 0.0;
      bool vertex_is_frontier = false;
      bool vertex_is_bound = false;
      int bound_count = 0;

      bool if_point_need_to_be_removed_ = false;
      bool if_point_obstacle = false;
      // bool if_point_need_to_be_deleted_ = false;
      int bad_point_count_ = 0;
      int worst_point_count_ = 0;
      bool LoopFirst_ = true;
      for (std::vector<Eigen::VectorXd>::iterator iterator_pplanner_vector_ =
               iter_pplanner_map_.second.begin();
           iterator_pplanner_vector_ != iter_pplanner_map_.second.end(); ++iterator_pplanner_vector_)
      {
        Eigen::VectorXd vector_middle_ = *iterator_pplanner_vector_;
        elevation_post_ += vector_middle_[2];
        traversability_scoal_post_ += vector_middle_[3];
        if (LoopFirst_)
        {
          elevation_with_covariance_for_post_ = vector_middle_[2];
          traversability_with_covariance_for_post_ = vector_middle_[3];
          variance_post_ = vector_middle_[6];
          LoopFirst_ = false;
        }
        else
        {
          elevation_with_covariance_for_post_ = (vector_middle_[2] * variance_post_ + elevation_with_covariance_for_post_ * vector_middle_[6]) / (variance_post_ + vector_middle_[6]);
          traversability_with_covariance_for_post_ = (vector_middle_[3] * variance_post_ + traversability_with_covariance_for_post_ * vector_middle_[6]) / (variance_post_ + vector_middle_[6]);
          variance_post_ = (variance_post_ * vector_middle_[6]) / (variance_post_ + vector_middle_[6]);
        }

        if (vector_middle_[4] == 1)
        {
          vertex_is_frontier = true;
        }
        if (vector_middle_[5] == 1)
        {
          bound_count++;
          // vertex_is_bound = true;
        }
        if (vector_middle_[3] < traversability_score_threshold_global_graph)
        {
          bad_point_count_++;
        }
        if (vector_middle_[3] < traversability_score_threshold_low_bound)
        {
          // ROS_ERROR("traversability_score_threshold_low_bound is %f", traversability_score_threshold_low_bound);

          // if_point_need_to_be_removed_ = true;
          // if_point_obstacle = true;
          worst_point_count_++;
          // if_point_need_to_be_deleted_ = true;
        }
      }

      if (worst_point_count_ > worst_point_threshold_)
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
      }

      if (bound_count > 0)
      {
        vertex_is_bound = true;
      }

      if ((bad_point_count_ > bad_point_threshold) && (!vertex_is_frontier))
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
        // continue;
      }

      elevation_post_ = elevation_post_ / iter_pplanner_map_.second.size();
      traversability_scoal_post_ = traversability_scoal_post_ / iter_pplanner_map_.second.size();
      // ROS_INFO("Traversability_scoal_post_ and traversability_with_covariance_for_post_ are %f and %f.", traversability_scoal_post_, traversability_with_covariance_for_post_);
      // ROS_INFO("Elevation_post_ and elevation_with_covariance_for_post_ are %f and %f.", elevation_post_, elevation_with_covariance_for_post_);
      if (traversability_with_covariance_for_post_ < traversability_score_threshold_global_graph) // traversability_scoal_post_
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
      }
      if ((iter_pplanner_map_.second.size() < point_threshold) && (!vertex_is_frontier))
      {
        // ROS_ERROR("The Point might need to be removed!");
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
        continue;
      }
      if (iter_pplanner_map_.second.size() < point_threshold_frontier)
      {
        if_point_need_to_be_removed_ = true;
        continue;
      }
      if (elevation_with_covariance_for_post_ != elevation_with_covariance_for_post_)
      {
        continue;
      }

      // if ((if_point_need_to_be_removed_) && (!if_point_need_to_be_deleted_))
      if (if_point_need_to_be_removed_)
      {
        // continue;
      }
      bool IfOrdinaryPoint_ = true;
      if (vertex_is_frontier || vertex_is_bound || if_point_obstacle)
      {
        IfOrdinaryPoint_ = false;
      }

      Eigen::Vector4d MembershipDegreeInput_(vertex_is_frontier, if_point_obstacle, vertex_is_bound, IfOrdinaryPoint_);
      PointAttributeMembershipDegreeVec MembershipDegreeOutput_;

      StateVec state_to_global_(iter_pplanner_map_.first.x, iter_pplanner_map_.first.y, elevation_with_covariance_for_post_, 0); // elevation_post_
      // std::vector<Vertex *> nearest_vertices_in_global_graph_;

      bool if_vertex_exist_in_big_whole_graph; // vertex_exist_in_Graph_Manager
      std::vector<Vertex *> nearest_vertices_in_big_whole_graph;
      nearest_vertices_in_big_whole_graph.clear();
      if_vertex_exist_in_big_whole_graph = vertex_exist_in_Graph_Manager(state_to_global_, &nearest_vertices_in_big_whole_graph, Big_global_graph_with_obstacles_);

      if (if_vertex_exist_in_big_whole_graph)
      {
        if (nearest_vertices_in_big_whole_graph.size() > 1)
        {
          ROS_ERROR("THERE IS SOMETHING WRONG WHEN CHECK IF VERTEX EXISTED IN Big Whole GLOBAL GRAPH");
          if (nearest_vertices_in_big_whole_graph.size() < 4)
          {
            continue;
          }
          else
          {
            ROS_ERROR("WHAT THE HELL! WHERE ARE THE POINTS CONMING IN Big Whole GRAPH?");
            std::cout << "size of nearest_vertices_in_big_whole_graph is " << nearest_vertices_in_big_whole_graph.size() << endl;
            for (size_t hell = 0; hell < nearest_vertices_in_big_whole_graph.size(); ++hell)
            {
              std::cout << "hell is " << hell << "hell id is" << nearest_vertices_in_big_whole_graph[hell]->id << "and state is " << std::endl
                        << nearest_vertices_in_big_whole_graph[hell]->state << std::endl;
            }
            ros::shutdown();
            break;
          }
        }
        else
        {

          PointAttributeMembershipDegreeVec MembershipDegreeLast_ = nearest_vertices_in_big_whole_graph[0]->membership_degree;
          MembershipDegreeOutput_ = MembershipDegreeM_ * MembershipDegreeInput_ + MembershipDegreeN_ * MembershipDegreeLast_;
          MembershipDegreeOutput_.normalize();

          int max_id_by_membership_degree_ = 0;
          double value_middle_ = 0;
          for (int select_id_i = 0; select_id_i < MembershipDegreeOutput_.size(); select_id_i++)
          {
            if (MembershipDegreeOutput_[select_id_i] > value_middle_)
            {
              max_id_by_membership_degree_ = select_id_i;
              value_middle_ = MembershipDegreeOutput_[select_id_i];
            }
          }

          switch (max_id_by_membership_degree_)
          {
          case 0:
            vertex_is_frontier = true;
            if_point_obstacle = false;
            vertex_is_bound = false;
            IfOrdinaryPoint_ = false;
            break;
          case 1:
            vertex_is_frontier = false;
            if_point_obstacle = true;
            vertex_is_bound = false;
            IfOrdinaryPoint_ = false;
            break;
          case 2:
            vertex_is_frontier = false;
            if_point_obstacle = false;
            vertex_is_bound = true;
            IfOrdinaryPoint_ = false;
            break;
          case 3:
            vertex_is_frontier = false;
            if_point_obstacle = false;
            vertex_is_bound = false;
            IfOrdinaryPoint_ = true;
            break;
          default:
            ROS_ERROR("There is something wrong when calculate the membership degree.");
            break;
          }

          if (!nearest_vertices_in_big_whole_graph[0]->bound_could_not_be_changed)
          {
            nearest_vertices_in_big_whole_graph[0]->is_bound = vertex_is_bound;
            if (vertex_is_bound == false)
            {
              nearest_vertices_in_big_whole_graph[0]->bound_could_not_be_changed = true;
            }
          }

          if (!nearest_vertices_in_big_whole_graph[0]->frontier_could_not_be_changed)
          {
            nearest_vertices_in_big_whole_graph[0]->is_frontier = vertex_is_frontier;
            if (vertex_is_frontier == false)
            {
              nearest_vertices_in_big_whole_graph[0]->frontier_could_not_be_changed = true;
            }
          }
          nearest_vertices_in_big_whole_graph[0]->state = state_to_global_;

          if (if_point_obstacle)
          {
            nearest_vertices_in_big_whole_graph[0]->is_obstacle = true;
          }
          else
          {
            nearest_vertices_in_big_whole_graph[0]->is_obstacle = false;
          }

          if (if_point_obstacle)
          {
            if (global_graph_->vertices_map_.find(nearest_vertices_in_big_whole_graph[0]->id) == global_graph_->vertices_map_.end())
            {
              // continue;
            }
            else
            {
              int id_of_vertex_to_be_changed_in_global_graph_ = nearest_vertices_in_big_whole_graph[0]->id;
              // Vertex *vertex_to_be_changed_in_global_graph_ = global_graph_->getVertex(id);
              Vertex *vertex_to_be_changed_in_global_graph_ = nearest_vertices_in_big_whole_graph[0];
              vertex_to_be_changed_in_global_graph_->state = state_to_global_;
              vertex_to_be_changed_in_global_graph_->is_obstacle = true;
            }
          }
          else
          {
            if (global_graph_->vertices_map_.find(nearest_vertices_in_big_whole_graph[0]->id) == global_graph_->vertices_map_.end())
            {
              //  Vertex *vertex_new = new Vertex(global_graph_->generateVertexID(), state_to_global_);
              Vertex *vertex_new = nearest_vertices_in_big_whole_graph[0];
              // vertex_new->is_frontier = vertex_is_frontier;
              // vertex_new->is_bound = vertex_is_bound;
              // vertex_new->is_obstacle = false;

              // if (vertex_is_frontier == false)
              // {
              //   vertex_new->frontier_could_not_be_changed = true;
              // }
              // if (vertex_new->is_bound == false)
              // {
              //   vertex_new->bound_could_not_be_changed = true;
              // }
              global_graph_->addVertex(vertex_new);

              UF_->addpoint(vertex_new->id);

              std::vector<Vertex *> nearest_vertices_in_big_box;
              global_graph_->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                     limit_box_z_for_edge, &nearest_vertices_in_big_box);
              if (nearest_vertices_in_big_box.size() > 9)
              {
                ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
              }
              for (size_t i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
              {
                if (vertex_new->id == nearest_vertices_in_big_box[i]->id)
                {
                  continue;
                }
                // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
                Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                          state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                          state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
                double direction_norm = direction.norm();

                // std::cout << "nearest direction is" << direction_norm << std::endl;
                global_graph_->removeEdge(vertex_new, nearest_vertices_in_big_box[i]);
                global_graph_->addEdge(vertex_new, nearest_vertices_in_big_box[i], direction_norm);

                UF_->merge(vertex_new->id, nearest_vertices_in_big_box[i]->id);
              }
            }
            else
            {
              int id_of_vertex_to_be_changed_in_global_graph_ = nearest_vertices_in_big_whole_graph[0]->id;
              // Vertex *vertex_to_be_changed_in_global_graph_ = global_graph_->getVertex(id);
              Vertex *vertex_to_be_changed_in_global_graph_ = nearest_vertices_in_big_whole_graph[0];
              // Vertex *vertex_to_be_changed_in_global_graph_ = global_graph_->vertices_map_[id_of_vertex_to_be_changed_in_global_graph_];

              // std::cout << "id of test point is " << vertex_to_be_changed_in_global_graph_->id << std::endl;
              // std::cout << "state of test point is " << vertex_to_be_changed_in_global_graph_->state << std::endl;
              // ROS_INFO("vertex_to_be_changed_in_global_graph_->bound_could_not_be_changed is %d ", vertex_to_be_changed_in_global_graph_->bound_could_not_be_changed);
              if (!vertex_to_be_changed_in_global_graph_->bound_could_not_be_changed)
              {
                vertex_to_be_changed_in_global_graph_->is_bound = vertex_is_bound;
                if (vertex_is_bound == false)
                {
                  vertex_to_be_changed_in_global_graph_->bound_could_not_be_changed = true;
                }
                else
                {
                }
              }
              else
              {
              }

              if (!vertex_to_be_changed_in_global_graph_->frontier_could_not_be_changed)
              {
                vertex_to_be_changed_in_global_graph_->is_frontier = vertex_is_frontier;
                if (vertex_is_frontier == false)
                {
                  vertex_to_be_changed_in_global_graph_->frontier_could_not_be_changed = true;
                }
              }

              vertex_to_be_changed_in_global_graph_->state = state_to_global_;
              vertex_to_be_changed_in_global_graph_->is_obstacle = false;

              std::vector<Vertex *> nearest_vertices_in_big_box;
              global_graph_->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                     limit_box_z_for_edge, &nearest_vertices_in_big_box);
              if (nearest_vertices_in_big_box.size() > 9)
              {
                ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
              }
              for (size_t i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
              {
                if (vertex_to_be_changed_in_global_graph_->id == nearest_vertices_in_big_box[i]->id)
                {
                  continue;
                }
                // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
                Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                          state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                          state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
                double direction_norm = direction.norm();

                // std::cout << "nearest direction is" << direction_norm << std::endl;
                global_graph_->removeEdge(vertex_to_be_changed_in_global_graph_, nearest_vertices_in_big_box[i]);
                global_graph_->addEdge(vertex_to_be_changed_in_global_graph_, nearest_vertices_in_big_box[i], direction_norm);

                UF_->merge(vertex_to_be_changed_in_global_graph_->id, nearest_vertices_in_big_box[i]->id);
              }
            }
          }
        }
      }
      else
      {

        PointAttributeMembershipDegreeVec MembershipDegreeLast_(0, 0, 0, 0);
        MembershipDegreeOutput_ = MembershipDegreeM_ * MembershipDegreeInput_ + MembershipDegreeN_ * MembershipDegreeLast_;
        MembershipDegreeOutput_.normalize();

        int max_id_by_membership_degree_ = 0;
        double value_middle_ = 0;
        for (int select_id_i = 0; select_id_i < MembershipDegreeOutput_.size(); select_id_i++)
        {
          if (MembershipDegreeOutput_[select_id_i] > value_middle_)
          {
            max_id_by_membership_degree_ = select_id_i;
            value_middle_ = MembershipDegreeOutput_[select_id_i];
          }
        }

        switch (max_id_by_membership_degree_)
        {
        case 0:
          vertex_is_frontier = true;
          if_point_obstacle = false;
          vertex_is_bound = false;
          IfOrdinaryPoint_ = false;
          break;
        case 1:
          vertex_is_frontier = false;
          if_point_obstacle = true;
          vertex_is_bound = false;
          IfOrdinaryPoint_ = false;
          break;
        case 2:
          vertex_is_frontier = false;
          if_point_obstacle = false;
          vertex_is_bound = true;
          IfOrdinaryPoint_ = false;
          break;
        case 3:
          vertex_is_frontier = false;
          if_point_obstacle = false;
          vertex_is_bound = false;
          IfOrdinaryPoint_ = true;
          break;
        default:
          ROS_ERROR("There is something wrong when calculate the membership degree.");
          break;
        }

        Vertex *vertex_new = new Vertex(Big_global_graph_with_obstacles_->generateVertexID(), state_to_global_);
        vertex_new->is_frontier = vertex_is_frontier;
        vertex_new->is_bound = vertex_is_bound;

        if (vertex_is_frontier == false)
        {
          vertex_new->frontier_could_not_be_changed = true;
        }
        if (vertex_new->is_bound == false)
        {
          vertex_new->bound_could_not_be_changed = true;
        }

        if (if_point_obstacle)
        {
          vertex_new->is_obstacle = true;
        }
        else
        {
          vertex_new->is_obstacle = false;
        }
        Big_global_graph_with_obstacles_->addVertex(vertex_new);

        if (if_point_obstacle)
        {
          continue;
        }
        else
        {
          // Add new point
          Vertex *vertex_new_global = vertex_new;
          vertex_new_global->is_frontier = vertex_is_frontier;
          vertex_new_global->is_bound = vertex_is_bound;
          vertex_new_global->is_obstacle = false;

          if (vertex_is_frontier == false)
          {
            vertex_new_global->frontier_could_not_be_changed = true;
          }
          if (vertex_new_global->is_bound == false)
          {
            vertex_new_global->bound_could_not_be_changed = true;
          }
          global_graph_->addVertex(vertex_new_global);

          UF_->addpoint(vertex_new_global->id);

          std::vector<Vertex *> nearest_vertices_in_big_box;
          global_graph_->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                 limit_box_z_for_edge, &nearest_vertices_in_big_box);
          if (nearest_vertices_in_big_box.size() > 9)
          {
            ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
          }
          for (size_t i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
          {
            if (vertex_new_global->id == nearest_vertices_in_big_box[i]->id)
            {
              continue;
            }
            // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
            Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                      state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                      state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
            double direction_norm = direction.norm();

            // std::cout << "nearest direction is" << direction_norm << std::endl;
            global_graph_->removeEdge(vertex_new_global, nearest_vertices_in_big_box[i]);
            global_graph_->addEdge(vertex_new_global, nearest_vertices_in_big_box[i], direction_norm);

            UF_->merge(vertex_new_global->id, nearest_vertices_in_big_box[i]->id);
          }
        }
      }

      /*
            if (vertex_exist(state_to_global_, &nearest_vertices_in_global_graph_))
            {
              if (nearest_vertices_in_global_graph_.size() > 1)
              {
                ROS_ERROR("THERE IS SOMETHING WRONG WHEN CHECK IF VERTEX EXISTED IN GLOBAL GRAPH");

                if (nearest_vertices_in_global_graph_.size() < 4)
                {
                  continue;
                }
                else
                {
                  ROS_ERROR("WHAT THE HELL! WHERE ARE THE POINTS CONMING IN GLOBAL GRAPH?");
                  std::cout << "size of nearest_vertices_in_global_graph_ is " << nearest_vertices_in_global_graph_.size() << endl;
                  for (int hell = 0; hell < nearest_vertices_in_global_graph_.size(); ++hell)
                  {
                    std::cout << "hell is " << hell << "hell id is" << nearest_vertices_in_global_graph_[hell]->id << "and state is " << std::endl
                              << nearest_vertices_in_global_graph_[hell]->state << std::endl;
                  }
                  ros::shutdown();
                  break;
                }
              }
              else
              {
                // if (if_point_need_to_be_deleted_)
                // {
                //   global_graph_->removeVertex(nearest_vertices_in_global_graph_[0]);
                //   continue;
                // }

                // nearest_vertices_in_global_graph_[0]->is_bound = vertex_is_bound;
                if (!nearest_vertices_in_global_graph_[0]->bound_could_not_be_changed)
                {
                  nearest_vertices_in_global_graph_[0]->is_bound = vertex_is_bound;
                  if (vertex_is_bound == false)
                  {
                    nearest_vertices_in_global_graph_[0]->bound_could_not_be_changed = true;
                  }
                }

                if (!nearest_vertices_in_global_graph_[0]->frontier_could_not_be_changed)
                {
                  // ROS_INFO("Frontier_could_not_be_changed is");
                  // std::cout << nearest_vertices_in_global_graph_[0]->frontier_could_not_be_changed << std::endl;
                  nearest_vertices_in_global_graph_[0]->is_frontier = vertex_is_frontier;
                  if (vertex_is_frontier == false)
                  {
                    nearest_vertices_in_global_graph_[0]->frontier_could_not_be_changed = true;
                  }
                }
                // nearest_vertices_in_global_graph_[0]->is_frontier = vertex_is_frontier;
                nearest_vertices_in_global_graph_[0]->state = state_to_global_;
                // pplanner_vector_map_[nearest_vertices[0]->id].push_back(point_new_);
                std::vector<Vertex *> nearest_vertices_in_big_box;
                global_graph_->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                       limit_box_z_for_edge, &nearest_vertices_in_big_box);
                if (nearest_vertices_in_big_box.size() > 9)
                {
                  ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
                }
                for (int i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
                {
                  // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
                  Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                            state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                            state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
                  double direction_norm = direction.norm();

                  // std::cout << "nearest direction is" << direction_norm << std::endl;
                  global_graph_->removeEdge(nearest_vertices_in_global_graph_[0], nearest_vertices_in_big_box[i]);
                  global_graph_->addEdge(nearest_vertices_in_global_graph_[0], nearest_vertices_in_big_box[i], direction_norm);

                  UF_->merge(nearest_vertices_in_global_graph_[0]->id, nearest_vertices_in_big_box[i]->id);
                }
              }
              continue;
            }
            else
            {
              // if (if_point_need_to_be_deleted_)
              // {
              //   continue;
              // }
              Vertex *vertex_new = new Vertex(global_graph_->generateVertexID(), state_to_global_);
              vertex_new->is_frontier = vertex_is_frontier;
              vertex_new->is_bound = vertex_is_bound;

              if (vertex_is_frontier == false)
              {
                vertex_new->frontier_could_not_be_changed = true;
              }
              if (vertex_new->is_bound == false)
              {
                vertex_new->bound_could_not_be_changed = true;
              }
              global_graph_->addVertex(vertex_new);

              UF_->addpoint(vertex_new->id);
              // pplanner_vector_map_[vertex_new->id].push_back(point_new_);
              std::vector<Vertex *> nearest_vertices_in_big_box;
              global_graph_->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                     limit_box_z_for_edge, &nearest_vertices_in_big_box);
              if (nearest_vertices_in_big_box.size() > 9)
              {
                ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
              }
              for (int i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
              {
                // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
                Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                          state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                          state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
                double direction_norm = direction.norm();

                // std::cout << "nearest direction is" << direction_norm << std::endl;
                global_graph_->removeEdge(vertex_new, nearest_vertices_in_big_box[i]);
                global_graph_->addEdge(vertex_new, nearest_vertices_in_big_box[i], direction_norm);

                UF_->merge(vertex_new->id, nearest_vertices_in_big_box[i]->id);
              }
            }
            */
    }
  }
  else
  {
    ROS_ERROR("THERE IS SOMETHING WRONG WITH pplanner_vector_map_");
  }

  if ((RoughPplanner_vector_map_.size() > 0) && true)
  {
    // ROS_INFO("Size of RoughPplanner_vector_map_ is %d.", RoughPplanner_vector_map_.size());
    for (auto &iter_RoughPplanner_map_ : RoughPplanner_vector_map_)
    {
      // int id = iter_RoughPplanner_map_.first;
      double elevation_post_ = 0.0;
      double traversability_scoal_post_ = 0.0;
      bool vertex_is_frontier = false;
      bool if_point_need_to_be_removed_ = false;
      bool if_point_obstacle = false;
      // bool if_point_need_to_be_deleted_ = false;
      int bad_point_count_ = 0;
      int worst_point_count_ = 0;

      // ROS_INFO("Id is %d and Size of RoughPplanner_vector_map_ is %d.", id, iter_RoughPplanner_map_.second.size());

      for (std::vector<Eigen::VectorXd>::iterator iterator_RoughPplanner_vector_ =
               iter_RoughPplanner_map_.second.begin();
           iterator_RoughPplanner_vector_ != iter_RoughPplanner_map_.second.end(); ++iterator_RoughPplanner_vector_)
      {
        Eigen::VectorXd vector_middle_ = *iterator_RoughPplanner_vector_;
        elevation_post_ += vector_middle_[2];
        traversability_scoal_post_ += vector_middle_[3];

        if (vector_middle_[4] == 1)
        {
          vertex_is_frontier = true;
        }

        if (vector_middle_[3] < traversability_score_threshold_global_graph)
        {
          bad_point_count_++;
        }
        if (vector_middle_[3] < traversability_score_threshold_low_bound)
        {
          worst_point_count_++;
        }
      }

      if (worst_point_count_ > worst_point_threshold_)
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
      }

      if ((bad_point_count_ > bad_point_threshold) && (!vertex_is_frontier))
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
        // continue;
      }

      elevation_post_ = elevation_post_ / iter_RoughPplanner_map_.second.size();
      traversability_scoal_post_ = traversability_scoal_post_ / iter_RoughPplanner_map_.second.size();
      if (traversability_scoal_post_ < traversability_score_threshold_global_graph) // traversability_scoal_post_
      {
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
      }
      // if ((iter_RoughPplanner_map_.second.size() < point_threshold) && (!vertex_is_frontier))
      if ((iter_RoughPplanner_map_.second.size() <
           (0.8 * (int)(PplannerRoughGlobalMapResolution_ / tra_map_resolution) *
            (int)(PplannerRoughGlobalMapResolution_ / tra_map_resolution))) &&
          (!vertex_is_frontier))
      {
        // ROS_ERROR("The Point might need to be removed!");
        if_point_need_to_be_removed_ = true;
        if_point_obstacle = true;
        continue;
      }
      if (iter_RoughPplanner_map_.second.size() < point_threshold_frontier)
      {
        if_point_need_to_be_removed_ = true;
        continue;
      }

      // if ((if_point_need_to_be_removed_) && (!if_point_need_to_be_deleted_))
      if (if_point_need_to_be_removed_)
      {
        // continue;
      }

      StateVec state_to_RoughGlobal_(iter_RoughPplanner_map_.first.x, iter_RoughPplanner_map_.first.y, elevation_post_, 0); // elevation_post_

      bool if_vertex_exist_in_rough_global_graph; // vertex_exist_in_Graph_Manager
      std::vector<Vertex *> nearest_vertices_in_rough_global_graph;
      nearest_vertices_in_rough_global_graph.clear();
      if_vertex_exist_in_rough_global_graph = vertex_exist_in_Graph_Manager(state_to_RoughGlobal_, &nearest_vertices_in_rough_global_graph, RoughGlobalGraph_, PplannerRoughGlobalMapResolution_);

      if (if_vertex_exist_in_rough_global_graph)
      {
        if (nearest_vertices_in_rough_global_graph.size() > 1)
        {
          ROS_ERROR("THERE IS SOMETHING WRONG WHEN CHECK IF VERTEX EXISTED IN ROUGH GLOBAL GRAPH");
          if (nearest_vertices_in_rough_global_graph.size() < 4)
          {
            continue;
          }
          else
          {
            ROS_ERROR("WHAT THE HELL! WHERE ARE THE POINTS CONMING IN ROUGH GLOBAL GRAPH?");
            std::cout << "size of nearest_vertices_in_rough_global_graph is " << nearest_vertices_in_rough_global_graph.size() << endl;
            for (size_t hell = 0; hell < nearest_vertices_in_rough_global_graph.size(); ++hell)
            {
              std::cout << "hell is " << hell << "hell id is" << nearest_vertices_in_rough_global_graph[hell]->id << "and state is " << std::endl
                        << nearest_vertices_in_rough_global_graph[hell]->state << std::endl;
            }
            ros::shutdown();
            break;
          }
        }
        else
        {
          nearest_vertices_in_rough_global_graph[0]->state = state_to_RoughGlobal_;
          if (if_point_obstacle)
          {
            nearest_vertices_in_rough_global_graph[0]->is_obstacle = true;
          }
          else
          {
            nearest_vertices_in_rough_global_graph[0]->is_obstacle = false;
          }

          std::vector<Vertex *> nearest_vertices_in_big_box_for_rough_global_graph_;
          RoughGlobalGraph_->getNearestVerticesInBox(&state_to_RoughGlobal_, 1.2 * PplannerRoughGlobalMapResolution_, 1.2 * PplannerRoughGlobalMapResolution_,
                                                     limit_box_z_for_edge, &nearest_vertices_in_big_box_for_rough_global_graph_);
          if (nearest_vertices_in_big_box_for_rough_global_graph_.size() > 9)
          {
            ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box_for_rough_global_graph_");
          }
          for (size_t i = 0; (i < nearest_vertices_in_big_box_for_rough_global_graph_.size()) && (i < edges_of_one_point_max); ++i)
          {
            if (nearest_vertices_in_rough_global_graph[0]->id == nearest_vertices_in_big_box_for_rough_global_graph_[i]->id)
            {
              continue;
            }
            Eigen::Vector3d direction(nearest_vertices_in_rough_global_graph[0]->state[0] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[0],
                                      nearest_vertices_in_rough_global_graph[0]->state[1] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[1],
                                      nearest_vertices_in_rough_global_graph[0]->state[2] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[2]);
            double direction_norm = direction.norm();
            RoughGlobalGraph_->removeEdge(nearest_vertices_in_rough_global_graph[0], nearest_vertices_in_big_box_for_rough_global_graph_[i]);
            if (if_point_obstacle)
            {
              RoughGlobalGraph_->addEdge(nearest_vertices_in_rough_global_graph[0], nearest_vertices_in_big_box_for_rough_global_graph_[i], 1000);
            }
            else
            {
              RoughGlobalGraph_->addEdge(nearest_vertices_in_rough_global_graph[0], nearest_vertices_in_big_box_for_rough_global_graph_[i], direction_norm);
            }
          }
        }
      }
      else
      {
        if (if_point_obstacle)
        {
          continue;
        }

        Vertex *vertex_new = new Vertex(RoughGlobalGraph_->generateVertexID(), state_to_RoughGlobal_);
        RoughGlobalGraph_->addVertex(vertex_new);
        std::vector<Vertex *> nearest_vertices_in_big_box_for_rough_global_graph_;
        RoughGlobalGraph_->getNearestVerticesInBox(&state_to_RoughGlobal_, 1.2 * PplannerRoughGlobalMapResolution_, 1.2 * PplannerRoughGlobalMapResolution_,
                                                   limit_box_z_for_edge, &nearest_vertices_in_big_box_for_rough_global_graph_);
        if (nearest_vertices_in_big_box_for_rough_global_graph_.size() > 9)
        {
          ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box_for_rough_global_graph_");
        }
        for (size_t i = 0; (i < nearest_vertices_in_big_box_for_rough_global_graph_.size()) && (i < edges_of_one_point_max); ++i)
        {
          if (vertex_new->id == nearest_vertices_in_big_box_for_rough_global_graph_[i]->id)
          {
            continue;
          }
          Eigen::Vector3d direction(vertex_new->state[0] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[0],
                                    vertex_new->state[1] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[1],
                                    vertex_new->state[2] - nearest_vertices_in_big_box_for_rough_global_graph_[i]->state[2]);
          double direction_norm = direction.norm();
          RoughGlobalGraph_->removeEdge(vertex_new, nearest_vertices_in_big_box_for_rough_global_graph_[i]);
          RoughGlobalGraph_->addEdge(vertex_new, nearest_vertices_in_big_box_for_rough_global_graph_[i], direction_norm);
        }
      }
    }
  }
  else
  {
    ROS_ERROR("THERE IS SOMETHING WRONG WITH RoughPplanner_vector_map_");
  }
}

bool PPlanner::vertex_exist(StateVec state_new, std::vector<Vertex *> *nearest_vertices)
{

  // if (global_graph_->getNearestVertex(&state_new, &nearest_vertex))
  if (global_graph_->getNearestVerticesInBox(&state_new, pplaner_map_resolution / 2, pplaner_map_resolution / 2,
                                             limit_box_z, nearest_vertices))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool PPlanner::vertex_exist_in_Graph_Manager(StateVec state_new, std::vector<Vertex *> *nearest_vertices, std::shared_ptr<GraphManager> local_graph)
{

  if (local_graph->getNearestVerticesInBox(&state_new, pplaner_map_resolution / 2, pplaner_map_resolution / 2,
                                           limit_box_z, nearest_vertices))
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool PPlanner::vertex_exist_in_Graph_Manager(StateVec state_new, std::vector<Vertex *> *nearest_vertices, std::shared_ptr<GraphManager> GraphManagerFun_, double MapResolution_)
{
  if (GraphManagerFun_->getNearestVerticesInBox(&state_new, MapResolution_ / 2, MapResolution_ / 2,
                                                limit_box_z, nearest_vertices))
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool PPlanner::if_point_internal(double x, double y,
                                 grid_map::GridMap traversability_map_fun,
                                 bool *if_point_valid, bool *if_point_bound)
{
  // grid_map::Position position_temp0(x, y);
  grid_map::Position position_temp1(x - tra_map_resolution, y - tra_map_resolution);
  grid_map::Position position_temp2(x - tra_map_resolution, y);
  grid_map::Position position_temp3(x - tra_map_resolution, y + tra_map_resolution);

  grid_map::Position position_temp4(x, y - tra_map_resolution);
  grid_map::Position position_temp5(x, y);
  grid_map::Position position_temp6(x, y + tra_map_resolution);

  grid_map::Position position_temp7(x + tra_map_resolution, y - tra_map_resolution);
  grid_map::Position position_temp8(x + tra_map_resolution, y);
  grid_map::Position position_temp9(x + tra_map_resolution, y + tra_map_resolution);

  std::vector<grid_map::Position> position_vector;
  position_vector.clear();
  position_vector.push_back(position_temp1);
  position_vector.push_back(position_temp2);
  position_vector.push_back(position_temp3);
  position_vector.push_back(position_temp4);
  position_vector.push_back(position_temp5);
  position_vector.push_back(position_temp6);
  position_vector.push_back(position_temp7);
  position_vector.push_back(position_temp8);
  position_vector.push_back(position_temp9);
  int inside_count = 0;
  int valid_count = 0;
  int tra_bad_count = 0;
  std::vector<Eigen::Array2i> index_vector;

  for (std::vector<grid_map::Position>::iterator it = position_vector.begin(); it != position_vector.end(); it++)
  {
    if (traversability_map_fun.isInside(*it))
    {
      inside_count++;
      Eigen::Array2i Index;
      traversability_map_fun.getIndex(*it, Index);
      if ((traversability_map_fun.isValid(Index, Elevation_layer_.c_str())) &&
          (traversability_map_fun.isValid(Index, Traversability_layer_.c_str())) &&
          (traversability_map_fun.isValid(Index, Traversability_supplementary_layer_.c_str())))
      {
        valid_count++;
      }
      if ((traversability_map_fun.at(Traversability_layer_.c_str(), Index) < 0.1) &&
          (traversability_map_fun.at(Traversability_supplementary_layer_.c_str(), Index) < 0.1))
      {
        tra_bad_count++;
      }
    }
  }
  if (inside_count > 8)
  {
    if (valid_count > 8)
    {
      if (tra_bad_count > 0)
      {
        *if_point_bound = true;
      }
      return true;
    }
    else
    {
      if (valid_count < 3)
      {
        // TODO The logic of checking if point is valid might need to be corrected.
        // ROS_ERROR("valid_count is %d", valid_count);
        *if_point_valid = false;
      }
      return false;
    }
  }
  else
  {
    return false;
  }
  return false;
}

bool PPlanner::if_point_in_the_range(double x, double y)
{

  if ((tra_map_bound_x_min < x) && (x < tra_map_bound_x_max) && (tra_map_bound_y_min < y) && (y < tra_map_bound_y_max))
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool PPlanner::if_point_isNan(grid_map::GridMap traversability_map_fun, const std::string &layer, grid_map::Position position)
{
  double value_ = traversability_map_fun.atPosition(layer, position);
  //  if (isnan(traversability_map_fun.atPosition(layer, position)))
  if (value_ != value_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PPlanner::GraphManager_check(const GraphManager *graph_manager)
{

  if (graph_manager->Graph_->getNumVertices() == graph_manager->vertices_map_.size())
  {
    // ROS_INFO("pass the point size check");
  }
  else
  {
    ROS_ERROR("There is something wrong when checking the point size");
  }
}

void PPlanner::GraphManager_check(const std::shared_ptr<GraphManager> GraphManager_)
{

  if (GraphManager_->Graph_->getNumVertices() == GraphManager_->vertices_map_.size())
  {
    // ROS_INFO("pass the point size check");
  }
  else
  {
    ROS_ERROR("There is something wrong when checking the point size, GraphManager_.");
    ROS_ERROR("GraphManager_->Graph_->getNumVertices() is %d, ", GraphManager_->Graph_->getNumVertices());
    ROS_ERROR("GraphManager_->vertices_map_.size() is %ld, ", GraphManager_->vertices_map_.size());
  }
}

void PPlanner::visualizeGraph(GraphManager *graph_manager)
{
  if (graph_manager->getNumVertices() < 2)
  {
    return;
  }
  else
  {
  }

  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  // std::cout << "robot_state_ is " << robot_state_ << std::endl;

  Vertex *robot_nearest_vertex;
  int robot_global_id = 0;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertex))
  {
    robot_global_id = robot_nearest_vertex->id;
  }
  else
  {
    ROS_ERROR("Could not find robot_nearest_vertex in visualizeGraph function.");
  }
  int root_of_robot_global_id_ = UF_->find(robot_global_id);

  frontier_id_vector_.clear();

  GraphManager_check(graph_manager);
  gridcells_msg_.cells.clear();
  gridcells_frontier_msg_.cells.clear();
  gridcells_bound_msg_.cells.clear();

  std::shared_ptr<Graph> graph_for_visualize = graph_manager->Graph_;
  std::unordered_map<int, Vertex *> &v_map_visualize = graph_manager->vertices_map_;

  // if (graph_manager->getNumVertices() == 0)
  // {
  //   return;
  // }
  // if (pplanner_graph_pub_.getNumSubscribers() < 1)
  // {
  //   return;
  // }
  visualization_msgs::Marker vertex_marker;
  visualization_msgs::Marker vertex_frontier_marker;
  visualization_msgs::Marker vertex_bound_marker;
  if (if_vertex_marker_also_pub)
  {
    // visualization_msgs::Marker vertex_marker;
    vertex_marker.header.stamp = ros::Time::now();
    vertex_marker.header.seq = 0;
    vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
    vertex_marker.id = 0;
    vertex_marker.ns = "vertices";
    vertex_marker.action = visualization_msgs::Marker::ADD;
    vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_marker.scale.x = pplaner_map_resolution;
    vertex_marker.scale.y = pplaner_map_resolution;
    vertex_marker.scale.z = 0.01;
    vertex_marker.color.r = 0.0 / 255.0;
    vertex_marker.color.g = 255.0 / 255.0;
    vertex_marker.color.b = 0.0 / 255.0;
    vertex_marker.color.a = 1.0;
    vertex_marker.lifetime = ros::Duration(0.0);
    vertex_marker.frame_locked = false;

    // visualization_msgs::Marker vertex_frontier_marker;
    vertex_frontier_marker.header.stamp = ros::Time::now();
    vertex_frontier_marker.header.seq = 0;
    vertex_frontier_marker.header.frame_id = Grid_cell_frame_.c_str();
    vertex_frontier_marker.id = 0;
    vertex_frontier_marker.ns = "vertices_frontier";
    vertex_frontier_marker.action = visualization_msgs::Marker::ADD;
    vertex_frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_frontier_marker.scale.x = pplaner_map_resolution;
    vertex_frontier_marker.scale.y = pplaner_map_resolution;
    vertex_frontier_marker.scale.z = 0.01;
    vertex_frontier_marker.color.r = 255.0 / 255.0;
    vertex_frontier_marker.color.g = 0.0 / 255.0;
    vertex_frontier_marker.color.b = 0.0 / 255.0;
    vertex_frontier_marker.color.a = 1.0;
    vertex_frontier_marker.lifetime = ros::Duration(0.0);
    vertex_frontier_marker.frame_locked = false;

    // visualization_msgs::Marker vertex_bound_marker;
    vertex_bound_marker.header.stamp = ros::Time::now();
    vertex_bound_marker.header.seq = 0;
    vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
    vertex_bound_marker.id = 0;
    vertex_bound_marker.ns = "vertices_bound";
    vertex_bound_marker.action = visualization_msgs::Marker::ADD;
    vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertex_bound_marker.scale.x = pplaner_map_resolution;
    vertex_bound_marker.scale.y = pplaner_map_resolution;
    vertex_bound_marker.scale.z = 0.01;
    vertex_bound_marker.color.r = 0.0 / 255.0;
    vertex_bound_marker.color.g = 0.0 / 255.0;
    vertex_bound_marker.color.b = 255.0 / 255.0;
    vertex_bound_marker.color.a = 1.0;
    vertex_bound_marker.lifetime = ros::Duration(0.0);
    vertex_bound_marker.frame_locked = false;
  }

  for (auto &iter_vertices_map_ : graph_manager->vertices_map_)
  {
    int id = iter_vertices_map_.first;
    if (UF_->find(id) != root_of_robot_global_id_)
    {
      continue;
    }
    else
    {
    }
    if (v_map_visualize[id]->is_obstacle == true)
    {
      continue;
    }
    else
    {
    }
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];

    if ((p1.x != p1.x) || (p1.y != p1.y) || (p1.z != p1.z))
    {
      continue;
    }

    if (v_map_visualize[id]->is_frontier)
    {
      gridcells_frontier_msg_.cells.push_back(p1);
      frontier_id_vector_.push_back(id);
      if (if_vertex_marker_also_pub)
      {
        vertex_frontier_marker.points.push_back(p1);
      }
    }
    else
    {
      if (v_map_visualize[id]->is_bound)
      {
        // std::cout << "ceshi1" << std::endl;
        gridcells_bound_msg_.cells.push_back(p1);
      }
      else
      {
        gridcells_msg_.cells.push_back(p1);
      }

      if (if_vertex_marker_also_pub)
      {
        if (v_map_visualize[id]->is_bound)
        {
          vertex_bound_marker.points.push_back(p1);
        }
        else
        {
          vertex_marker.points.push_back(p1);
        }
        // vertex_marker.points.push_back(p1);
      }
    }
  }

  // std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  // graph_for_visualize->getVertexIterator(vi);
  // for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  // {
  //   int id = graph_for_visualize->getVertexProperty(it);

  //   if (UF_->find(id) != root_of_robot_global_id_)
  //   {
  //     continue;
  //   }
  //   else
  //   {
  //   }

  //   geometry_msgs::Point p1;
  //   p1.x = v_map_visualize[id]->state[0];
  //   p1.y = v_map_visualize[id]->state[1];
  //   p1.z = v_map_visualize[id]->state[2];

  //   if (v_map_visualize[id]->is_frontier)
  //   {
  //     gridcells_frontier_msg_.cells.push_back(p1);
  //     frontier_id_vector_.push_back(id);
  //     if (if_vertex_marker_also_pub)
  //     {
  //       vertex_frontier_marker.points.push_back(p1);
  //     }
  //   }
  //   else
  //   {
  //     if (v_map_visualize[id]->is_bound)
  //     {
  //       // std::cout << "ceshi1" << std::endl;
  //       gridcells_bound_msg_.cells.push_back(p1);
  //     }
  //     else
  //     {
  //       gridcells_msg_.cells.push_back(p1);
  //     }

  //     if (if_vertex_marker_also_pub)
  //     {
  //       if (v_map_visualize[id]->is_bound)
  //       {
  //         vertex_bound_marker.points.push_back(p1);
  //       }
  //       else
  //       {
  //         vertex_marker.points.push_back(p1);
  //       }
  //       // vertex_marker.points.push_back(p1);
  //     }
  //   }

  //   // gridcells_msg_.cells.push_back(p1);
  // }
  gridcells_msg_.header.seq++;
  gridcells_msg_.header.stamp = ros::Time::now();
  gridcells_frontier_msg_.header.seq++;
  gridcells_frontier_msg_.header.stamp = ros::Time::now();
  gridcells_bound_msg_.header.seq++;
  gridcells_bound_msg_.header.stamp = ros::Time::now();

  if (if_vertex_marker_also_pub)
  {
    pplanner_graph_pub_.publish(vertex_marker);
    pplanner_graph_pub_.publish(vertex_frontier_marker);
    pplanner_graph_pub_.publish(vertex_bound_marker);
  }

  // std::cout << "if_edge_pub is " << if_edge_pub << std::endl;
  if (if_edge_pub)
  {
    // Plot all edges
    visualization_msgs::Marker edge_marker;
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.header.seq = 0;
    edge_marker.header.frame_id = Grid_cell_frame_.c_str();
    edge_marker.id = 0;
    edge_marker.ns = "edges";
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = 0.02;
    edge_marker.color.r = 200.0 / 255.0;
    edge_marker.color.g = 100.0 / 255.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;
    edge_marker.lifetime = ros::Duration(0.0);
    edge_marker.frame_locked = false;

    std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
        ei;
    graph_for_visualize->getEdgeIterator(ei);
    for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
    {
      int src_id, tgt_id;
      double weight;
      std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);

      if (UF_->find(src_id) != root_of_robot_global_id_)
      {
        continue;
      }
      else
      {
      }

      if (UF_->find(tgt_id) != root_of_robot_global_id_)
      {
        continue;
      }
      else
      {
      }

      if ((v_map_visualize[src_id]->is_obstacle) || (v_map_visualize[tgt_id]->is_obstacle))
      {
        continue;
      }
      else
      {
      }

      geometry_msgs::Point p1;
      p1.x = v_map_visualize[src_id]->state[0];
      p1.y = v_map_visualize[src_id]->state[1];
      p1.z = v_map_visualize[src_id]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map_visualize[tgt_id]->state[0];
      p2.y = v_map_visualize[tgt_id]->state[1];
      p2.z = v_map_visualize[tgt_id]->state[2];
      if ((p1.x != p1.x) || (p1.y != p1.y) || (p1.z != p1.z))
      {
        continue;
      }
      if ((p2.x != p2.x) || (p2.y != p2.y) || (p2.z != p2.z))
      {
        continue;
      }

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
    pplanner_graph_pub_.publish(edge_marker);
  }
}

void PPlanner::traverability_map_Callback(const grid_map_msgs::GridMap &msg)
{
  std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
  msg_traversability_map = msg;
  grid_map::GridMapRosConverter::fromMessage(msg_traversability_map, traversability_map);
  if (tra_map_sizex != msg_traversability_map.info.length_x)
  {
    ROS_ERROR("Traversability parameters tra_map_sizex might be wrong and the lucky is that would be fixed");
    std::cout << "tra_map_sizex is " << tra_map_sizex << " and the msg_traversability_map.info.length_x is " << msg_traversability_map.info.length_x << std::endl;
    tra_map_sizex = msg_traversability_map.info.length_x;
  }
  if (tra_map_sizey != msg_traversability_map.info.length_y)
  {
    ROS_ERROR("Traversability parameters tra_map_sizey might be wrong and the lucky is that would be fixed");
    std::cout << "tra_map_sizey is " << tra_map_sizey << " and the msg_traversability_map.info.length_y is " << msg_traversability_map.info.length_y << std::endl;
    tra_map_sizey = msg_traversability_map.info.length_y;
  }

  if (tra_map_resolution != msg_traversability_map.info.resolution)
  {
    ROS_ERROR("Traversability parameters tra_map_resolution might be wrong and the lucky is that would be fixed");
    std::cout << "tra_map_resolution is " << tra_map_resolution << " and the msg_traversability_map.info.resolution is " << msg_traversability_map.info.resolution << std::endl;
    tra_map_resolution = msg_traversability_map.info.resolution;
    ROS_ERROR("However, if the bad point threshold improve automatically, it might be a wrong value and for reduce the load of calcualte I do not change the bad point threshold");
  }

  tra_map_bound_x_max = tra_map_sizex / 2 + msg_traversability_map.info.pose.position.x;
  tra_map_bound_x_min = -tra_map_sizex / 2 + msg_traversability_map.info.pose.position.x;
  tra_map_bound_y_max = tra_map_sizex / 2 + msg_traversability_map.info.pose.position.y;
  tra_map_bound_y_min = -tra_map_sizex / 2 + msg_traversability_map.info.pose.position.y;
  if (!if_map_initialled_)
  {
    if_map_initialled_ = true;
  }

  // ROS_INFO("HELLO,CALL BACK!");

  // try
  // {
  //   world_base_listener.waitForTransform(Track_frame_.c_str(), World_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
  //   world_base_listener.lookupTransform(Track_frame_.c_str(), World_frame_.c_str(),
  //                                       ros::Time(0), world_base_transform);
  // }
  // catch (tf::TransformException &ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  //   // ros::Duration(1.0).sleep();
  // }

  if (if_iterator_map_with_map_Callback)
  {
    std::lock_guard<std::mutex> lock(updateMapMutex_);
    // std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
    std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
    std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
    std::lock_guard<std::mutex> lock14(TheRoughGlobalGraph_Mutex_);
    ros::Time t_qian = ros::Time::now();
    iterator_map(traversability_map);
    if (IfNeedUpdateGlobalGraphForUpdatingTarget_)
    {
      IfNeedUpdateGlobalGraphForUpdatingTarget_ = false;
    }
    ros::Time t_hou = ros::Time::now();
    double t_used = t_hou.toSec() - t_qian.toSec();
    ROS_INFO("Iterator map with map callback and time used for iterator map is %f", t_used);
  }
}

void PPlanner::visualizeGlobalGraphTimerCallback(const ros::TimerEvent &event)
{
  // ROS_INFO("[visualizeGlobalGraphTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock14(TheRoughGlobalGraph_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);
  if (if_map_initialled_)
  {
  }
  else
  {
    return;
  }
  ros::Time t_qian = ros::Time::now();
  visualizeGraph(global_graph_);
  grid_cells_pub_.publish(gridcells_msg_);
  grid_cells_of_frontier_pub_.publish(gridcells_frontier_msg_);
  // std::cout << "size of gridcells_bound_msg_ cells is " << gridcells_bound_msg_.cells.size() << std::endl;
  grid_cells_of_bound_pub_.publish(gridcells_bound_msg_);

  visualizeGraphManager(Big_global_graph_with_obstacles_, WholeGloabalGraphWithObstacles_pub);
  string RoughGlobalGrpahNameSpace_ = "RoughGlobalGrpah_ns_";
  visualizeGraphManager(RoughGlobalGraph_, RoughGlobalGraph_pub, RoughGlobalGrpahNameSpace_);
  // ROS_INFO("Size of RoughGlobalGraph_ is %d.", RoughGlobalGraph_->vertices_map_.size());

  if (IfNeedUpdateFrontireGraphForUpdatingTarget_)
  {
    if (IfNeedUpdateGlobalGraphForUpdatingTarget_)
    {
    }
    else
    {
      IfNeedUpdateFrontireGraphForUpdatingTarget_ = false;
    }
  }

  if (if_update_local_planning_graph)
  {
    std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
    visualizeLocalPlanningGraphManager(local_graph_for_safety_with_planning_);
  }

  ros::Time t_hou = ros::Time::now();
  double t_used = t_hou.toSec() - t_qian.toSec();
  if (t_used > 0.5)
  {
    ROS_INFO("Time used for visualize map is %f", t_used);
  }
  else
  {
  }
  // ROS_INFO("[visualizeGlobalGraphTimer_INFO]:END.");
}

void PPlanner::iteratorMapTimerCallback(const ros::TimerEvent &event)
{
  // ROS_INFO("[iteratorMapTimer_INFO]:START.");
  if (if_map_initialled_)
  {
    std::lock_guard<std::mutex> lock(updateMapMutex_);
    std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
    std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
    std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
    std::lock_guard<std::mutex> lock14(TheRoughGlobalGraph_Mutex_);
    // ROS_INFO("ITERATOR CALLBACK");
    ros::Time t_qian = ros::Time::now();
    iterator_map(traversability_map);

    if (IfNeedUpdateGlobalGraphForUpdatingTarget_)
    {
      IfNeedUpdateGlobalGraphForUpdatingTarget_ = false;
    }
    else
    {
    }

    ros::Time t_hou = ros::Time::now();
    double t_used = t_hou.toSec() - t_qian.toSec();
    if (t_used > 0.5)
    {
      ROS_INFO("Iterator map without map callback and time used for iterator map is %f", t_used);
    }
    // ROS_INFO("Iterator map without map callback and time used for iterator map is %f", t_used);

    bool code_ceshi = false;
    if (code_ceshi)
    {
      double a = std::asin(-0.5);
      double angle = a / 3.141592654 * 180;
      std::cout << "asin(-0.5) is " << a << std::endl;
      std::cout << "angle is " << angle << std::endl;
      // ROS_INFO("point_threshold is %d.", point_threshold);
      //  try
      //  {
      //    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
      //    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
      //                                        ros::Time(0), world_base_transform);
      //  }
      //  catch (tf::TransformException &ex)
      //  {
      //    ROS_ERROR("There is something wrong when trying get robot pose to get target orientation.");
      //    ROS_ERROR("%s", ex.what());
      //    // ros::Duration(1.0).sleep();
      //  }
      //  tf::Quaternion q_robot_target = world_base_transform.getRotation();
      //  double yaw_robot = tf::getYaw(q_robot_target); // - M_PI;
      //  std::cout << "yaw_robot is " << yaw_robot << std::endl;
      //  std::cout << "sin M_PI_2 is " << sin(M_PI_2) << std::endl;
      //  test_fun();
    }
  }
  // ROS_INFO("[iteratorMapTimer_INFO]:END.");
}
void PPlanner::update_frontier_map(const ros::TimerEvent &event)
{
  // ROS_INFO("[update_frontier_mapTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);

  // ros::Time t_qian = ros::Time::now();

  if (global_graph_->getNumVertices() > 1)
  {
    std::shared_ptr<Graph> graph_for_update = global_graph_->Graph_;
    std::unordered_map<int, Vertex *> &v_map_update = global_graph_->vertices_map_;

    for (auto &iter_vertices_map_ : v_map_update)
    {
      int id = iter_vertices_map_.first;

      Vertex *vertex_to_update_ = global_graph_->getVertex(id);
      if ((v_map_update[id]->is_frontier) && (!v_map_update[id]->frontier_could_not_be_changed))
      {
        std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
        nearest_vertices_in_global_graph_for_update_.clear();
        StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
        if (global_graph_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   limit_box_z, &nearest_vertices_in_global_graph_for_update_))
        {
          // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
          if (nearest_vertices_in_global_graph_for_update_.size() > 8)
          {
            // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
            // Vertex *vertex_to_update_ = global_graph_->getVertex(id);
            // ROS_ERROR("Frontier of aertex to update befor is %d, and frontier_could_not_be_changed is %d.", vertex_to_update_->is_frontier, vertex_to_update_->frontier_could_not_be_changed);
            vertex_to_update_->is_frontier = false;
            vertex_to_update_->frontier_could_not_be_changed = true;
          }
          int neighbor_valid_count_ = 0;
          for (size_t i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
          {
            if (nearest_vertices_in_global_graph_for_update_[i]->is_frontier == true)
            {
              ++neighbor_valid_count_;
            }
          }
          if (neighbor_valid_count_ < 2)
          {
            vertex_to_update_->is_frontier = false;
            // vertex_to_update_->frontier_could_not_be_changed = true;
          }
        }
        else
        {
          vertex_to_update_->is_frontier = false;
        }

        std::vector<Vertex *> nearest_vertices_in_BigWholeGlobal_graph_for_update_;
        nearest_vertices_in_BigWholeGlobal_graph_for_update_.clear();
        if (Big_global_graph_with_obstacles_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      limit_box_z, &nearest_vertices_in_BigWholeGlobal_graph_for_update_))
        {
          if (nearest_vertices_in_BigWholeGlobal_graph_for_update_.size() > 8)
          {
            vertex_to_update_->is_frontier = false;
            vertex_to_update_->frontier_could_not_be_changed = true;
          }
          int neighbor_valid_count2_ = 0;
          int nobstacles_count_ = 0;
          for (size_t i = 0; i < nearest_vertices_in_BigWholeGlobal_graph_for_update_.size(); ++i)
          {
            if (nearest_vertices_in_BigWholeGlobal_graph_for_update_[i]->is_frontier == true)
            {
              ++neighbor_valid_count2_;
            }
            if (nearest_vertices_in_BigWholeGlobal_graph_for_update_[i]->is_obstacle == true)
            {
              ++nobstacles_count_;
            }
          }
          if (neighbor_valid_count2_ < 2)
          {
            vertex_to_update_->is_frontier = false;
          }
          if (nobstacles_count_ > 4)
          {
            vertex_to_update_->is_frontier = false;
          }
        }
        else
        {
          vertex_to_update_->is_frontier = false;
        }
      }
    }

    // std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
    // graph_for_update->getVertexIterator(vi);
    // for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
    // {
    //   //  Need to be changed when maintain a big whole graph
    //   int id = graph_for_update->getVertexProperty(it);
    //   Vertex *vertex_to_update_ = global_graph_->getVertex(id);
    //   if ((v_map_update[id]->is_frontier) && (!v_map_update[id]->frontier_could_not_be_changed))
    //   {
    //     std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
    //     nearest_vertices_in_global_graph_for_update_.clear();
    //     StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
    //     if (global_graph_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
    //                                                pplaner_map_resolution + 0.25 * pplaner_map_resolution,
    //                                                limit_box_z, &nearest_vertices_in_global_graph_for_update_))
    //     {
    //       // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
    //       if (nearest_vertices_in_global_graph_for_update_.size() > 8)
    //       {
    //         // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
    //         // Vertex *vertex_to_update_ = global_graph_->getVertex(id);
    //         // ROS_ERROR("Frontier of aertex to update befor is %d, and frontier_could_not_be_changed is %d.", vertex_to_update_->is_frontier, vertex_to_update_->frontier_could_not_be_changed);
    //         vertex_to_update_->is_frontier = false;
    //         vertex_to_update_->frontier_could_not_be_changed = true;
    //       }
    //       int neighbor_valid_count_ = 0;
    //       for (int i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
    //       {
    //         if (nearest_vertices_in_global_graph_for_update_[i]->is_frontier == true)
    //         {
    //           ++neighbor_valid_count_;
    //         }
    //       }
    //       if (neighbor_valid_count_ < 2)
    //       {
    //         vertex_to_update_->is_frontier = false;
    //         // vertex_to_update_->frontier_could_not_be_changed = true;
    //       }
    //     }
    //     else
    //     {
    //       vertex_to_update_->is_frontier = false;
    //     }
    //   }
    // }
  }

  // if (frontier_id_vector_.size() > 0)
  // {
  //   for (std::vector<int>::iterator it = frontier_id_vector_.begin(); it != frontier_id_vector_.end(); it++)
  //   {
  //     int id = *it;
  //     Vertex *vertex_to_update_ = global_graph_->getVertex(id);

  //     if ((vertex_to_update_->is_frontier) && (!vertex_to_update_->frontier_could_not_be_changed))
  //     {
  //       std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
  //       nearest_vertices_in_global_graph_for_update_.clear();
  //       StateVec state_(vertex_to_update_->state[0], vertex_to_update_->state[1], vertex_to_update_->state[2], 0);
  //       if (global_graph_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
  //                                                  pplaner_map_resolution + 0.25 * pplaner_map_resolution,
  //                                                  limit_box_z, &nearest_vertices_in_global_graph_for_update_))
  //       {
  //         // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
  //         if (nearest_vertices_in_global_graph_for_update_.size() > 8)
  //         {
  //           // ROS_ERROR("Size of nearest vertices in_global_graph_for_update_ is %ld", nearest_vertices_in_global_graph_for_update_.size());
  //           // Vertex *vertex_to_update_ = global_graph_->getVertex(id);
  //           // ROS_ERROR("Frontier of aertex to update befor is %d, and frontier_could_not_be_changed is %d.", vertex_to_update_->is_frontier, vertex_to_update_->frontier_could_not_be_changed);
  //           vertex_to_update_->is_frontier = false;
  //           vertex_to_update_->frontier_could_not_be_changed = true;
  //         }
  //         int neighbor_valid_count_ = 0;
  //         for (int i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
  //         {
  //           if (nearest_vertices_in_global_graph_for_update_[i]->is_frontier == true)
  //           {
  //             ++neighbor_valid_count_;
  //           }
  //         }
  //         if (neighbor_valid_count_ < 2)
  //         {
  //           // ROS_ERROR("THe neighbor valid count is %d", neighbor_valid_count_);
  //           vertex_to_update_->is_frontier = false;
  //           // vertex_to_update_->frontier_could_not_be_changed = true;
  //         }
  //       }
  //       else
  //       {
  //         vertex_to_update_->is_frontier = false;
  //       }
  //     }
  //   }
  // }

  if (if_update_local_planning_graph)
  {
    std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
    if (local_graph_for_safety_with_planning_->getNumVertices() > 1)
    {
      std::shared_ptr<Graph> graph_for_update = local_graph_for_safety_with_planning_->Graph_;
      std::unordered_map<int, Vertex *> &v_map_update = local_graph_for_safety_with_planning_->vertices_map_;
      std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
      graph_for_update->getVertexIterator(vi);
      for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
      {
        int id = graph_for_update->getVertexProperty(it);
        Vertex *vertex_to_update_ = local_graph_for_safety_with_planning_->getVertex(id);
        if ((v_map_update[id]->is_frontier) && (!v_map_update[id]->frontier_could_not_be_changed))
        {
          std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
          nearest_vertices_in_global_graph_for_update_.clear();
          StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
          if (local_graph_for_safety_with_planning_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                             pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                             limit_box_z, &nearest_vertices_in_global_graph_for_update_))
          {
            if (nearest_vertices_in_global_graph_for_update_.size() > 8)
            {
              vertex_to_update_->is_frontier = false;
              vertex_to_update_->frontier_could_not_be_changed = true;
            }
            int neighbor_valid_count_ = 0;
            for (size_t i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
            {
              if (nearest_vertices_in_global_graph_for_update_[i]->is_frontier == true)
              {
                ++neighbor_valid_count_;
              }
            }
            if (neighbor_valid_count_ < 2)
            {
              vertex_to_update_->is_frontier = false;
            }
          }
          else
          {
            vertex_to_update_->is_frontier = false;
          }
        }
      }
    }
  }

  // ros::Time t_hou = ros::Time::now();
  // double t_used = t_hou.toSec() - t_qian.toSec();
  // ROS_INFO("Time used for update frontier map is %f", t_used);
  // ROS_INFO("[update_frontier_mapTimer_INFO]:END.");
}

void PPlanner::update_frontier_submap(const ros::TimerEvent &event)
{
  // ROS_INFO("[update_frontier_submapTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock5(Sub_Graph_Of_Frontier_Mutex_);
  std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);

  if (frontier_graph_ != NULL)
  {
    frontier_graph_->reset();
  }

  if (frontier_boost_graph_ != NULL)
  {
    // frontier_boost_graph_.reset();
    frontier_boost_graph_->clear();
  }
  if (frontier_id_vector_.size() > 0)
  {
    for (std::vector<int>::iterator it = frontier_id_vector_.begin(); it != frontier_id_vector_.end(); it++)
    {
      int id = *it;
      Vertex *vertex_to_update_ = global_graph_->getVertex(id);
      // frontier_graph
      StateVec state_frontier_new(vertex_to_update_->state[0], vertex_to_update_->state[1], vertex_to_update_->state[2], 0);
      Vertex *vertex_frontier_new = new Vertex(frontier_graph_->generateVertexID(), state_frontier_new);
      vertex_frontier_new->id = vertex_to_update_->id;
      frontier_graph_->addVertex(vertex_frontier_new);

      if (it == frontier_id_vector_.begin())
      {

        frontier_boost_graph_->addSourceVertex(vertex_frontier_new->id);
      }
      else
      {
        frontier_boost_graph_->addVertex(vertex_frontier_new->id);
        // std::cout << "id yuan is " << vertex_frontier_new->id << std::endl;
      }

      std::vector<Vertex *> nearest_vertices_in_big_box;
      frontier_graph_->getNearestVerticesInBox(&state_frontier_new, limit_box_x, limit_box_y,
                                               limit_box_z_for_edge, &nearest_vertices_in_big_box);
      if (nearest_vertices_in_big_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
      }
      for (size_t i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
      {
        if (vertex_frontier_new->id == nearest_vertices_in_big_box[i]->id)
        {
          continue;
        }
        // std::cout << "nearest id is" << nearest_vertices_in_big_box[i]->id << std::endl;
        Eigen::Vector3d direction(state_frontier_new[0] - nearest_vertices_in_big_box[i]->state[0],
                                  state_frontier_new[1] - nearest_vertices_in_big_box[i]->state[1],
                                  state_frontier_new[2] - nearest_vertices_in_big_box[i]->state[2]);
        double direction_norm = direction.norm();

        // std::cout << "nearest direction is" << direction_norm << std::endl;
        frontier_graph_->removeEdge(vertex_frontier_new, nearest_vertices_in_big_box[i]);
        frontier_graph_->addEdge(vertex_frontier_new, nearest_vertices_in_big_box[i], direction_norm);

        frontier_boost_graph_->removeEdge(vertex_frontier_new->id, nearest_vertices_in_big_box[i]->id);
        frontier_boost_graph_->addEdge(vertex_frontier_new->id, nearest_vertices_in_big_box[i]->id, direction_norm);
      }
    }

    // std::cout << "size of frontier_boost_graph_ is " << frontier_boost_graph_->getNumVertices() << std::endl;
    std::vector<int> component;
    int num_components;
    frontier_boost_graph_->getcomponent_and_num(&component, &num_components);

    // std::cout << "num_components is " << num_components << std::endl;
    // std::cout << "num vertices is " << frontier_boost_graph_->getNumVertices() << std::endl;
    // std::cout << "num edges is " << frontier_boost_graph_->getNumEdges() << std::endl;

    frontier_sub_graphs.clear();
    centers_of_frontier_sub_graphs_.clear();

    for (int i = 0; i < frontier_boost_graph_->getNumVertices(); i++)
    {
      frontier_sub_graphs[component[i]].push_back(frontier_boost_graph_->get_id_global(i));
    }

    // wait for point
    // std::cout << "frontier_sub_graphs.size is " << frontier_sub_graphs.size() << std::endl;
    visualization_msgs::MarkerArray marker_array_submap;
    marker_array_submap.markers.clear();

    visualization_msgs::MarkerArray marker_array_point_pre_exploration;
    marker_array_point_pre_exploration.markers.clear();

    int marker_id = 0;

    for (auto &iter_frontier_submap_ : frontier_sub_graphs)
    {
      int frontiercalss = iter_frontier_submap_.first;
      double rgb = (double)frontiercalss / (double)frontier_sub_graphs.size();
      string ns_middle = std::to_string(rgb);
      visualization_msgs::Marker vertex_submap_marker;
      vertex_submap_marker.header.stamp = ros::Time::now();
      vertex_submap_marker.header.seq = 0;
      vertex_submap_marker.header.frame_id = Grid_cell_frame_.c_str();
      vertex_submap_marker.id = marker_id;
      vertex_submap_marker.ns = ns_middle.c_str();
      vertex_submap_marker.action = visualization_msgs::Marker::ADD;
      vertex_submap_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      vertex_submap_marker.scale.x = pplaner_map_resolution;
      vertex_submap_marker.scale.y = pplaner_map_resolution;
      vertex_submap_marker.scale.z = 0.01;
      vertex_submap_marker.color.r = 1 - rgb;
      vertex_submap_marker.color.g = 1 - rgb;
      vertex_submap_marker.color.b = rgb;
      vertex_submap_marker.color.a = 1.0;
      vertex_submap_marker.lifetime = ros::Duration(0.0);
      vertex_submap_marker.frame_locked = false;

      for (std::vector<int>::iterator iterator_submap_vector_ = iter_frontier_submap_.second.begin(); iterator_submap_vector_ != iter_frontier_submap_.second.end(); ++iterator_submap_vector_)
      {
        Vertex *vertex_submap_used_ = frontier_graph_->getVertex(*iterator_submap_vector_);
        geometry_msgs::Point p_submap_;
        // std::cout << "state is " << vertex_submap_used_->state << std::endl;
        p_submap_.x = vertex_submap_used_->state[0];
        p_submap_.y = vertex_submap_used_->state[1];
        p_submap_.z = vertex_submap_used_->state[2];

        vertex_submap_marker.points.push_back(p_submap_);
      }
      if (vertex_submap_marker.points.size() < 2)
      {
        continue;
      }
      marker_array_submap.markers.push_back(vertex_submap_marker);

      bool id_center_valid = false;
      int id_central_point_ = get_central_point(iter_frontier_submap_.second, frontier_graph_, &id_center_valid);
      if (id_central_point_ == 0)
      {
        ROS_ERROR("I can not understand why the central id is %d.", id_central_point_);
      }

      if (id_center_valid)
      {
        centers_of_frontier_sub_graphs_[frontiercalss] = id_central_point_;
        Vertex *center = frontier_graph_->getVertex(id_central_point_);

        // point_pre_exploration
        visualization_msgs::Marker pre_exploration_marker;
        pre_exploration_marker.header.stamp = ros::Time::now();
        pre_exploration_marker.header.seq = 0;
        pre_exploration_marker.header.frame_id = Grid_cell_frame_.c_str();
        pre_exploration_marker.ns = "point_pre_exploration";
        pre_exploration_marker.action = visualization_msgs::Marker::ADD;
        pre_exploration_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        pre_exploration_marker.scale.z = 0.5; // text height
        pre_exploration_marker.color.r = 18.0 / 255.0;
        pre_exploration_marker.color.g = 15.0 / 255.0;
        pre_exploration_marker.color.b = 196.0 / 255.0;
        pre_exploration_marker.color.a = 1.0;
        pre_exploration_marker.lifetime = ros::Duration(0.0);
        pre_exploration_marker.frame_locked = false;
        pre_exploration_marker.pose.position.x = center->state[0];
        pre_exploration_marker.pose.position.y = center->state[1];
        pre_exploration_marker.pose.position.z = center->state[2] + 0.1;
        std::string text_display = std::to_string(id_central_point_);

        pre_exploration_marker.text = text_display;
        pre_exploration_marker.id = marker_id;
        marker_array_point_pre_exploration.markers.push_back(pre_exploration_marker);
        marker_id++;
      }
    }
    frontier_submap_pub.publish(marker_array_submap);
    points_for_pre_exploration_pub.publish(marker_array_point_pre_exploration);
  }
  // ROS_INFO("[update_frontier_submapTimer_INFO]:END.");
  if (IfNeedUpdateFrontierSubmap_)
  {
    if (IfNeedUpdateGlobalGraphForUpdatingTarget_)
    {
    }
    else
    {
      if (IfNeedUpdateFrontireGraphForUpdatingTarget_)
      {
      }
      else
      {
        IfNeedUpdateFrontierSubmap_ = false;
      }
      // IfNeedUpdateFrontierSubmap_ = false;
    }
    // IfNeedUpdateFrontierSubmap_ = false;
  }
}

int PPlanner::get_central_point(std::vector<int> ids_global_, std::shared_ptr<GraphManager> frontier_graph, bool *if_success)
{
  *if_success = false;
  Eigen::Vector3d point_sum_(0, 0, 0);
  for (std::vector<int>::iterator iterator_ids_vector_ = ids_global_.begin(); iterator_ids_vector_ != ids_global_.end(); ++iterator_ids_vector_)
  {
    Vertex *middle = frontier_graph->getVertex(*iterator_ids_vector_);
    Eigen::Vector3d point_middle_(middle->state[0], middle->state[1], middle->state[2]);
    point_sum_ = point_sum_ + point_middle_;
  }
  point_sum_ = (double)1 / (double)ids_global_.size() * point_sum_;

  StateVec state_nearest(point_sum_[0], point_sum_[1], point_sum_[2], 0);
  Vertex *nearest_vertices;
  if (frontier_graph->getNearestVertex(&state_nearest, &nearest_vertices))
  {
    *if_success = true;
    if (nearest_vertices->id == 0)
    {
      std::vector<Vertex *> nearest_vertices_middle;
      frontier_graph->getNearestVertices(&state_nearest, 10, &nearest_vertices_middle);
      if (nearest_vertices_middle.size() > 0)
      {
        for (size_t index_temp = 0; index_temp < nearest_vertices_middle.size(); index_temp++)
        {
          if (nearest_vertices_middle[index_temp]->id == 0)
          {
            continue;
          }
          else
          {
            return nearest_vertices_middle[index_temp]->id;
          }
        }
      }
      else
      {
        ROS_ERROR("Why the central point of frontier sub graph could not be find in range of 10.");
      }
    }
    else
    {
      return nearest_vertices->id;
    }
    // return nearest_vertices->id;
  }
  else
  {
    ROS_INFO("Could not find the nearest vertex and extend to search in range 10m.");
    std::vector<Vertex *> nearest_vertices_middle2;
    frontier_graph->getNearestVertices(&state_nearest, 10, &nearest_vertices_middle2);
    if (nearest_vertices_middle2.size() > 0)
    {
      for (size_t index_temp = 0; index_temp < nearest_vertices_middle2.size(); index_temp++)
      {
        if (nearest_vertices_middle2[index_temp]->id == 0)
        {
          continue;
        }
        else
        {
          return nearest_vertices_middle2[index_temp]->id;
        }
      }
    }
    else
    {
      ROS_ERROR("WHAT THE HELL! Why the central point of frontier sub graph could not be find in range of 10.");
    }
  }
  ROS_ERROR("Can not find the central point and the point id is set to 0.");
  return 0;
}

void PPlanner::update_bound_map(const ros::TimerEvent &event)
{
  // ROS_INFO("[update_bound_mapTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);

  if (global_graph_->getNumVertices() > 1)
  {
    std::shared_ptr<Graph> graph_for_update = global_graph_->Graph_;
    std::unordered_map<int, Vertex *> &v_map_update = global_graph_->vertices_map_;

    for (auto &iter_vertices_map_ : v_map_update)
    {
      int id = iter_vertices_map_.first;

      Vertex *vertex_to_update_ = global_graph_->getVertex(id);
      if (v_map_update[id]->is_frontier)
      {
        continue;
        StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
        // if near points in BigWholeGraph is obstacles
        std::vector<Vertex *> nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_;
        nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.clear();
        if (Big_global_graph_with_obstacles_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      1.0, &nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_))
        {
          for (size_t index = 0; index < nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.size(); index++)
          {
            if (nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_[index]->is_obstacle)
            {
              // ROS_INFO("Vertex is set to be bound.");
              vertex_to_update_->is_bound = true;
              vertex_to_update_->is_frontier = false;
              vertex_to_update_->frontier_could_not_be_changed = true;
              break;
            }
            else
            {
            }
          }
        }
        else
        {
        }
        continue;
      }
      else
      {
        std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
        nearest_vertices_in_global_graph_for_update_.clear();
        StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
        if (global_graph_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   limit_box_z, &nearest_vertices_in_global_graph_for_update_))
        {
          if (nearest_vertices_in_global_graph_for_update_.size() > 8)
          {
            if ((v_map_update[id]->is_bound)) //&& (!v_map_update[id]->bound_could_not_be_changed))
            {
              vertex_to_update_->is_bound = false;
              vertex_to_update_->bound_could_not_be_changed = true;
            }
          }
          else if (nearest_vertices_in_global_graph_for_update_.size() < 7)
          {
            // vertex_to_update_->is_bound = true;
          }
          else
          {
          }
        }
        else
        {
          // might be wrong
          // vertex_to_update_->is_bound = true;
        }
        // if near points in BigWholeGraph is obstacles
        std::vector<Vertex *> nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_;
        nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.clear();
        if (Big_global_graph_with_obstacles_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                      1.0, &nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_))
        {
          for (size_t index = 0; index < nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.size(); index++)
          {
            if (nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_[index]->is_obstacle)
            {
              // ROS_INFO("Vertex is set to be bound.");
              vertex_to_update_->is_bound = true;
              vertex_to_update_->is_frontier = false;
              vertex_to_update_->frontier_could_not_be_changed = true;
              break;
            }
            else
            {
            }
          }
        }
        else
        {
        }

        // nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.clear();
        // if (Big_global_graph_with_obstacles_->getNearestVerticesInBox(&state_, 0.25 * pplaner_map_resolution,
        //                                                               pplaner_map_resolution + 0.25 * pplaner_map_resolution,
        //                                                               limit_box_z, &nearest_vertices_in_global_graph_for_update_))
        // {
        //   for (size_t index = 0; index < nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_.size(); index++)
        //   {
        //     if (nearest_vertices_in_big_whole_global_graph_with_obstacles_for_update_[index]->is_obstacle)
        //     {
        //       vertex_to_update_->is_bound = true;
        //       break;
        //     }
        //     else
        //     {
        //     }
        //   }
        // }
        // else
        // {
        // }
      }
    }

    // std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
    // graph_for_update->getVertexIterator(vi);
    // for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
    // {
    //   //  Need to be changed when maintain a big whole graph
    //   int id = graph_for_update->getVertexProperty(it);
    //   Vertex *vertex_to_update_ = global_graph_->getVertex(id);
    //   if (v_map_update[id]->is_frontier)
    //   {
    //     continue;
    //   }
    //   else
    //   {
    //     std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
    //     nearest_vertices_in_global_graph_for_update_.clear();
    //     StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
    //     if (global_graph_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
    //                                                pplaner_map_resolution + 0.25 * pplaner_map_resolution,
    //                                                limit_box_z, &nearest_vertices_in_global_graph_for_update_))
    //     {
    //       if (nearest_vertices_in_global_graph_for_update_.size() > 8)
    //       {
    //         if ((v_map_update[id]->is_bound)) //&& (!v_map_update[id]->bound_could_not_be_changed))
    //         {
    //           vertex_to_update_->is_bound = false;
    //           vertex_to_update_->bound_could_not_be_changed = true;
    //         }
    //       }
    //       else if (nearest_vertices_in_global_graph_for_update_.size() < 7)
    //       {
    //         vertex_to_update_->is_bound = true;
    //       }
    //       else
    //       {
    //       }
    //       // int neighbor_valid_count_ = 0;
    //       // for (int i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
    //       // {
    //       //   if (nearest_vertices_in_global_graph_for_update_[i]->is_bound == true)
    //       //   {
    //       //     ++neighbor_valid_count_;
    //       //   }
    //       // }
    //       // if (neighbor_valid_count_ < 2)
    //       // {
    //       //   // might be wrong
    //       //   vertex_to_update_->is_bound = false;
    //       // }
    //     }
    //     else
    //     {
    //       // might be wrong
    //       vertex_to_update_->is_bound = true;
    //     }
    //   }
    // }
  }

  if (if_update_local_planning_graph)
  {
    std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
    if (local_graph_for_safety_with_planning_->getNumVertices() > 1)
    {
      std::shared_ptr<Graph> graph_for_update = local_graph_for_safety_with_planning_->Graph_;
      std::unordered_map<int, Vertex *> &v_map_update = local_graph_for_safety_with_planning_->vertices_map_;
      std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
      graph_for_update->getVertexIterator(vi);
      for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
      {
        int id = graph_for_update->getVertexProperty(it);
        Vertex *vertex_to_update_ = local_graph_for_safety_with_planning_->getVertex(id);
        if (v_map_update[id]->is_frontier)
        {
          continue;
        }
        else
        {
          std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_;
          nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.clear();
          StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
          if (local_graph_for_safety_with_planning_->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                             pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                                             limit_box_z, &nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_))
          {
            if (nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.size() > 8)
            {
              if ((v_map_update[id]->is_bound)) //&& (!v_map_update[id]->bound_could_not_be_changed))
              {
                vertex_to_update_->is_bound = false;
                vertex_to_update_->bound_could_not_be_changed = true;
              }
            }
            else if (nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.size() < 7)
            {
              vertex_to_update_->is_bound = true;
            }
            else
            {
            }
          }
          else
          {
            // might be wrong
            vertex_to_update_->is_bound = true;
          }
        }
      }
    }
    else
    {
    }
  }
  else
  {
  }
  // ROS_INFO("[update_bound_mapTimer_INFO]:END.");
}

bool PPlanner::globalPlannerCallback(
    pplanner_msgs::pplanner_pub_target::Request &req,
    pplanner_msgs::pplanner_pub_target::Response &res)
{
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);
  // If planning_boost_graph_ should be mutex?
  ros::Time t_qian = ros::Time::now();
  int id_target = req.id;
  global_target_id = id_target;
  int id_robot;
  path_tracked_vector_.clear();

  if (global_graph_->vertices_map_.find(global_target_id) == global_graph_->vertices_map_.end())
  {
    ROS_ERROR("The target point is not reachable.");
    res.result = false;
    return false;
  }
  else
  {
  }

  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  // std::cout << "robot_state_ is " << robot_state_ << std::endl;

  Vertex *robot_nearest_vertex;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertex))
  {
    if (robot_nearest_vertex->is_obstacle)
    {
      ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle when in global planner call back.");
      robot_nearest_vertex->is_obstacle = false;
      robot_nearest_vertex->membership_degree << 0, 0, 0, 1;
    }
    id_robot = robot_nearest_vertex->id;
    double x_expand_ = 2.0;
    double y_expand_ = 2.0;
    double z_expand_ = 0.3;

    double x_puls_used_ = 0.0;
    double y_puls_used_ = 0.0;
    double z_puls_used_ = 0.0;

    std::vector<Vertex *> path;

    for (int plus_count = 0; plus_count < Max_plus_count; plus_count++)
    {
      double x_expand_used_ = x_expand_ + x_puls_used_;
      double y_expand_used_ = y_expand_ + y_puls_used_;
      double z_expand_used_ = z_expand_ + z_puls_used_;
      // get_planning_graph(id_robot, id_target, graph_for_planning_, planning_boost_graph_,
      //                    x_expand_used_, y_expand_used_, z_expand_used_);
      get_planning_graph(id_robot, id_target, planning_boost_graph_,
                         x_expand_used_, y_expand_used_, z_expand_used_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);

      ROS_INFO("[globalPlannerCallback]: The %d th time of searching the path.", plus_count);

      // ROS_INFO("graph_for_planning_->getNumVertices is %d", graph_for_planning_->getNumVertices());
      // ROS_INFO("graph_for_planning_->getNumEdges is %d", graph_for_planning_->getNumEdges());
      // ROS_INFO("planning_boost_graph_->getNumVertices is %d", planning_boost_graph_->getNumVertices());
      // ROS_INFO("planning_boost_graph_->getNumEdges is %d", planning_boost_graph_->getNumEdges());

      ShortestPathsReportForPlanning report_for_planning;
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        // ROS_INFO("[globalPlannerCallback]: report_for_planning.parent_id_map'size is %ld", report_for_planning.parent_id_map.size());
        getShortestPath(id_target, report_for_planning, true, path);
        if (path.size() > 0)
        {
          // ROS_INFO("[globalPlannerCallback]: path'size is %ld", path.size());
          visualizePath(path);
          break;
        }
        else
        {
          x_puls_used_ += x_plus_;
          y_puls_used_ += y_plus_;
          z_puls_used_ += z_plus_;
        }
      }
      else
      {
        ROS_ERROR("[globalPlannerCallback]: Could not find the shortest path by dijkstra mehtod.");
        res.result = false;
        return false;
      }
    }

    if (path.size() == 0)
    {
      ROS_INFO("[globalPlannerCallback]: After %d times expand, the algorithm still could not find the path. Now try to find path in global graph.", Max_plus_count);

      // get_planning_graph(id_robot, id_target, graph_for_planning_, planning_boost_graph_);
      get_planning_graph(id_robot, id_target, planning_boost_graph_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);
      ShortestPathsReportForPlanning report_for_planning;
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        // ROS_INFO("[globalPlannerCallback]: report_for_planning.parent_id_map'size is %ld", report_for_planning.parent_id_map.size());
        getShortestPath(id_target, report_for_planning, true, path);
        if (path.size() > 0)
        {
          // ROS_INFO("[globalPlannerCallback]: path'size is %ld", path.size());
          visualizePath(path);
        }
        else
        {
          ROS_ERROR("[globalPlannerCallback]: Can not plan the path in global graph.");
        }
      }
      else
      {
        ROS_ERROR("[globalPlannerCallback]: Could not find the shortest path by dijkstra mehtod.");
        res.result = false;
        return false;
      }
    }

    if (path.size() == 0)
    {
      // ROS_ERROR("[globalPlannerCallback]: After %d times expand, the algorithm still could not find the path", Max_plus_count);
      res.result = false;
      return false;
    }
    else
    {
      if (path_tracked_vector_.size() == 0)
      {
        ROS_ERROR("[globalPlannerCallback]: There is something wrong with path_tracked_vector_");
      }
      else
      {

        tracker_wait_switch = false;
        local_path_tracker_is_paused_by_user = false;
        global_path_tracker_is_paused_by_user = false;
        // ppath_tracker_->stopTracker();
        // Local_ppath_tracker_->stopTracker();

        Local_ppath_tracker_->clearPathTracked();
        ppath_tracker_->clearPathTracked();
        ppath_tracker_->setPathTracked(path_tracked_vector_);
      }
    }
  }
  else
  {
    res.result = false;
    ROS_ERROR("[globalPlannerCallback]: Could not find the point which is nearest to robot state.");
    return false;
  }

  ros::Time t_hou = ros::Time::now();
  double t_used = t_hou.toSec() - t_qian.toSec();
  ROS_INFO("[globalPlannerCallback]: GlobalPlannerCallback succeed and time used is %f", t_used);

  res.result = true;
  return true;
}

void PPlanner::get_planning_graph(int source_id, int target_id,
                                  std::shared_ptr<GraphManager> graph_for_planning,
                                  std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                                  double x_expand, double y_expand,
                                  double z_expand)
{
  if (graph_for_planning != NULL)
  {
    graph_for_planning->reset();
  }
  bound_obstacle_vector_.clear();

  if (planning_boost_graph != NULL)
  {
    planning_boost_graph->clear();
  }
  // int count_internal = 0;

  const Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  const Vertex *vertex_target_ = global_graph_->getVertex(target_id);

  // Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  // Vertex *vertex_target_ = global_graph_->getVertex(target_id);

  double center_x = (vertex_source_->state[0] + vertex_target_->state[0]) / 2;
  double center_y = (vertex_source_->state[1] + vertex_target_->state[1]) / 2;
  double center_z = (vertex_source_->state[2] + vertex_target_->state[2]) / 2;
  StateVec center_(center_x, center_y, center_z, 0);
  double limit_x_ = abs(vertex_source_->state[0] - center_x) + x_expand; // + 2.0;
  double limit_y_ = abs(vertex_source_->state[1] - center_y) + y_expand; // + 2.0;
  double limit_z_ = abs(vertex_source_->state[2] - center_z) + z_expand; // + 0.3;

  std::vector<Vertex *> nearest_vertices_in_global_graph_for_planning_;
  nearest_vertices_in_global_graph_for_planning_.clear();

  if (global_graph_->getNearestVerticesInBox(&center_, limit_x_,
                                             limit_y_,
                                             limit_z_, &nearest_vertices_in_global_graph_for_planning_))
  {
    for (size_t i = 0; i < nearest_vertices_in_global_graph_for_planning_.size(); ++i)
    {
      if (nearest_vertices_in_global_graph_for_planning_[i]->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(nearest_vertices_in_global_graph_for_planning_[i]->state[0], nearest_vertices_in_global_graph_for_planning_[i]->state[1], nearest_vertices_in_global_graph_for_planning_[i]->state[2], 0);
      Vertex *vertex_planning_new;
      if (nearest_vertices_in_global_graph_for_planning_[i]->id == 0)
      {
        vertex_planning_new = graph_for_planning->getVertex(0);
        vertex_planning_new->is_frontier = nearest_vertices_in_global_graph_for_planning_[i]->is_frontier;
        vertex_planning_new->is_bound = nearest_vertices_in_global_graph_for_planning_[i]->is_bound;
      }
      else
      {
        vertex_planning_new = new Vertex(graph_for_planning->generateVertexID(), state_planning_new);
        vertex_planning_new->id = nearest_vertices_in_global_graph_for_planning_[i]->id;
        vertex_planning_new->is_frontier = nearest_vertices_in_global_graph_for_planning_[i]->is_frontier;
        vertex_planning_new->is_bound = nearest_vertices_in_global_graph_for_planning_[i]->is_bound;
        graph_for_planning->addVertex(vertex_planning_new);
      }
      // Vertex *vertex_planning_new = new Vertex(graph_for_planning->generateVertexID(), state_planning_new);
      // vertex_planning_new->id = nearest_vertices_in_global_graph_for_planning_[i]->id;
      // vertex_planning_new->is_frontier = nearest_vertices_in_global_graph_for_planning_[i]->is_frontier;
      // vertex_planning_new->is_bound = nearest_vertices_in_global_graph_for_planning_[i]->is_bound;
      // graph_for_planning->addVertex(vertex_planning_new);

      if (i == 0)
      {
        planning_boost_graph->addSourceVertex(vertex_planning_new->id);
      }
      else
      {
        planning_boost_graph->addVertex(vertex_planning_new->id);
      }

      if (vertex_planning_new->is_bound)
      {
        Eigen::Vector3d obstacle(vertex_planning_new->state[0], vertex_planning_new->state[1], vertex_planning_new->state[2]);
        bound_obstacle_vector_.push_back(obstacle);
      }
      else
      {
      }

      std::vector<Vertex *> nearest_vertices_in_small_box;
      graph_for_planning->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                  limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
      {

        if (vertex_planning_new->id == nearest_vertices_in_small_box[j]->id)
        {
          // std::cout << "same point continue;" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
        double direction_norm = direction.norm();

        // std::cout << "nearest direction is" << direction_norm << std::endl;
        graph_for_planning->removeEdge(vertex_planning_new, nearest_vertices_in_small_box[j]);
        graph_for_planning->addEdge(vertex_planning_new, nearest_vertices_in_small_box[j], direction_norm);

        planning_boost_graph->removeEdge(vertex_planning_new->id, nearest_vertices_in_small_box[j]->id);
        planning_boost_graph->addEdge(vertex_planning_new->id, nearest_vertices_in_small_box[j]->id, direction_norm);
      }
      // usleep(100);
    }
    // generate cost map
    std::vector<double> cost_map;
    std::unordered_map<int, double> cost_unorder_map;
    int robot_global_id_ = source_id;
    int target_global_id_ = target_id;
    Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
    Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
    double attractive_gain = Attractive_gain_;
    double repulsive_gain = Repulsive_gain_;
    double bound_radiu_ = Bound_radiu_;

    // The parameter should be given by yaml.
    //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
    cost_map.clear();
    cost_unorder_map.clear();

    get_cost_map(planning_boost_graph, &cost_map, &cost_unorder_map,
                 robot_global_id_, target_global_id_,
                 robot_pos_, target_pos_,
                 attractive_gain, repulsive_gain, bound_radiu_,
                 bound_obstacle_vector_);

    // update boost edge
    for (size_t ii = 0; ii < nearest_vertices_in_global_graph_for_planning_.size(); ++ii)
    {
      if (nearest_vertices_in_global_graph_for_planning_[ii]->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(nearest_vertices_in_global_graph_for_planning_[ii]->state[0], nearest_vertices_in_global_graph_for_planning_[ii]->state[1], nearest_vertices_in_global_graph_for_planning_[ii]->state[2], 0);
      std::vector<Vertex *> nearest_vertices_in_small_box2;
      graph_for_planning->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                  limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
      {

        if (nearest_vertices_in_global_graph_for_planning_[ii]->id == nearest_vertices_in_small_box2[jj]->id)
        {
          // std::cout << "same point in small box 2, continue" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
        double direction_norm = direction.norm();
        double force_middle_ = (cost_unorder_map[nearest_vertices_in_global_graph_for_planning_[ii]->id] + cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) / 2;
        double weight_middle = (direction_norm + force_middle_) / 2;
        // std::cout << "direction_norm is " << direction_norm << std::endl;
        // std::cout << "force_middle_ is " << force_middle_ << std::endl;

        planning_boost_graph->removeEdge(nearest_vertices_in_global_graph_for_planning_[ii]->id, nearest_vertices_in_small_box2[jj]->id);
        planning_boost_graph->addEdge(nearest_vertices_in_global_graph_for_planning_[ii]->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
      }
    }
  }
  else
  {
    ROS_ERROR("[get_planning_graph_INFO]: There is something wrong when running it.");
  }
}

void PPlanner::get_planning_graph(int source_id, int target_id,
                                  std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                                  double x_expand, double y_expand,
                                  double z_expand)
{
  bound_obstacle_vector_.clear();

  if (planning_boost_graph != NULL)
  {
    planning_boost_graph->clear();
  }
  // int count_internal = 0;

  const Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  const Vertex *vertex_target_ = global_graph_->getVertex(target_id);

  // Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  // Vertex *vertex_target_ = global_graph_->getVertex(target_id);

  double center_x = (vertex_source_->state[0] + vertex_target_->state[0]) / 2;
  double center_y = (vertex_source_->state[1] + vertex_target_->state[1]) / 2;
  double center_z = (vertex_source_->state[2] + vertex_target_->state[2]) / 2;
  StateVec center_(center_x, center_y, center_z, 0);
  double limit_x_ = abs(vertex_source_->state[0] - center_x) + x_expand; // + 2.0;
  double limit_y_ = abs(vertex_source_->state[1] - center_y) + y_expand; // + 2.0;
  double limit_z_ = abs(vertex_source_->state[2] - center_z) + z_expand; // + 0.3;

  std::vector<Vertex *> nearest_vertices_in_global_graph_for_planning_;
  nearest_vertices_in_global_graph_for_planning_.clear();

  if (global_graph_->getNearestVerticesInBox(&center_, limit_x_,
                                             limit_y_,
                                             limit_z_, &nearest_vertices_in_global_graph_for_planning_))
  {
    for (size_t i = 0; i < nearest_vertices_in_global_graph_for_planning_.size(); ++i)
    {
      if (nearest_vertices_in_global_graph_for_planning_[i]->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(nearest_vertices_in_global_graph_for_planning_[i]->state[0], nearest_vertices_in_global_graph_for_planning_[i]->state[1], nearest_vertices_in_global_graph_for_planning_[i]->state[2], 0);

      if (i == 0)
      {
        planning_boost_graph->addSourceVertex(nearest_vertices_in_global_graph_for_planning_[i]->id);
      }
      else
      {
        planning_boost_graph->addVertex(nearest_vertices_in_global_graph_for_planning_[i]->id);
      }

      if (nearest_vertices_in_global_graph_for_planning_[i]->is_bound)
      {
        Eigen::Vector3d obstacle(nearest_vertices_in_global_graph_for_planning_[i]->state[0],
                                 nearest_vertices_in_global_graph_for_planning_[i]->state[1],
                                 nearest_vertices_in_global_graph_for_planning_[i]->state[2]);
        bound_obstacle_vector_.push_back(obstacle);
      }
      else
      {
      }

      std::vector<Vertex *> nearest_vertices_in_small_box;
      global_graph_->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                             limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
      {

        if (nearest_vertices_in_global_graph_for_planning_[i]->id == nearest_vertices_in_small_box[j]->id)
        {
          // std::cout << "same point continue;" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
        double direction_norm = direction.norm();

        // std::cout << "nearest direction is" << direction_norm << std::endl;

        planning_boost_graph->removeEdge(nearest_vertices_in_global_graph_for_planning_[i]->id, nearest_vertices_in_small_box[j]->id);
        planning_boost_graph->addEdge(nearest_vertices_in_global_graph_for_planning_[i]->id, nearest_vertices_in_small_box[j]->id, direction_norm);
      }

      // usleep(100);
    }
    // generate cost map
    std::vector<double> cost_map;
    std::unordered_map<int, double> cost_unorder_map;
    int robot_global_id_ = source_id;
    int target_global_id_ = target_id;
    Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
    Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
    double attractive_gain = Attractive_gain_;
    double repulsive_gain = Repulsive_gain_;
    double bound_radiu_ = Bound_radiu_;

    // The parameter should be given by yaml.
    //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
    cost_map.clear();
    cost_unorder_map.clear();

    get_cost_map(planning_boost_graph, &cost_map, &cost_unorder_map,
                 robot_global_id_, target_global_id_,
                 robot_pos_, target_pos_,
                 attractive_gain, repulsive_gain, bound_radiu_,
                 bound_obstacle_vector_);

    // update boost edge
    for (size_t ii = 0; ii < nearest_vertices_in_global_graph_for_planning_.size(); ++ii)
    {
      if (nearest_vertices_in_global_graph_for_planning_[ii]->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(nearest_vertices_in_global_graph_for_planning_[ii]->state[0], nearest_vertices_in_global_graph_for_planning_[ii]->state[1], nearest_vertices_in_global_graph_for_planning_[ii]->state[2], 0);
      std::vector<Vertex *> nearest_vertices_in_small_box2;
      global_graph_->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                             limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
      {

        if (nearest_vertices_in_global_graph_for_planning_[ii]->id == nearest_vertices_in_small_box2[jj]->id)
        {
          // std::cout << "same point in small box 2, continue" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
        double direction_norm = direction.norm();
        double force_middle_ = (cost_unorder_map[nearest_vertices_in_global_graph_for_planning_[ii]->id] + cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) / 2;
        double weight_middle = (direction_norm + force_middle_) / 2;
        // std::cout << "direction_norm is " << direction_norm << std::endl;
        // std::cout << "force_middle_ is " << force_middle_ << std::endl;
        planning_boost_graph->removeEdge(nearest_vertices_in_global_graph_for_planning_[ii]->id, nearest_vertices_in_small_box2[jj]->id);
        planning_boost_graph->addEdge(nearest_vertices_in_global_graph_for_planning_[ii]->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
      }
    }
  }
  else
  {
    ROS_ERROR("[get_planning_graph_INFO]: There is something wrong when running it.");
  }
}

void PPlanner::get_planning_graph(int source_id, int target_id,
                                  std::shared_ptr<GraphManager> graph_for_planning,
                                  std::shared_ptr<Graph_For_Planning> planning_boost_graph)
{
  if (graph_for_planning != NULL)
  {
    graph_for_planning->reset();
  }
  bound_obstacle_vector_.clear();

  if (planning_boost_graph != NULL)
  {
    planning_boost_graph->clear();
  }
  // int count_internal = 0;

  const Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  const Vertex *vertex_target_ = global_graph_->getVertex(target_id);

  if (global_graph_->vertices_map_.size() > 0)
  {
    for (auto &iter_vertices_map_ : global_graph_->vertices_map_)
    {
      int i = iter_vertices_map_.first;
      if (iter_vertices_map_.second->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2], 0);
      Vertex *vertex_planning_new;
      if (iter_vertices_map_.second->id == 0)
      {
        vertex_planning_new = graph_for_planning->getVertex(0);
        vertex_planning_new->is_frontier = iter_vertices_map_.second->is_frontier;
        vertex_planning_new->is_bound = iter_vertices_map_.second->is_bound;
      }
      else
      {
        vertex_planning_new = new Vertex(graph_for_planning->generateVertexID(), state_planning_new);
        vertex_planning_new->id = iter_vertices_map_.second->id;
        vertex_planning_new->is_frontier = iter_vertices_map_.second->is_frontier;
        vertex_planning_new->is_bound = iter_vertices_map_.second->is_bound;
        graph_for_planning->addVertex(vertex_planning_new);
      }

      if (i == 0)
      {
        planning_boost_graph->addSourceVertex(vertex_planning_new->id);
      }
      else
      {
        planning_boost_graph->addVertex(vertex_planning_new->id);
      }

      if (vertex_planning_new->is_bound)
      {
        Eigen::Vector3d obstacle(vertex_planning_new->state[0], vertex_planning_new->state[1], vertex_planning_new->state[2]);
        bound_obstacle_vector_.push_back(obstacle);
      }
      else
      {
      }

      std::vector<Vertex *> nearest_vertices_in_small_box;
      graph_for_planning->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                  limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
      {

        if (vertex_planning_new->id == nearest_vertices_in_small_box[j]->id)
        {
          // std::cout << "same point continue;" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
        double direction_norm = direction.norm();

        // std::cout << "nearest direction is" << direction_norm << std::endl;
        graph_for_planning->removeEdge(vertex_planning_new, nearest_vertices_in_small_box[j]);
        graph_for_planning->addEdge(vertex_planning_new, nearest_vertices_in_small_box[j], direction_norm);

        planning_boost_graph->removeEdge(vertex_planning_new->id, nearest_vertices_in_small_box[j]->id);
        planning_boost_graph->addEdge(vertex_planning_new->id, nearest_vertices_in_small_box[j]->id, direction_norm);
      }
    }
    // generate cost map
    std::vector<double> cost_map;
    std::unordered_map<int, double> cost_unorder_map;
    int robot_global_id_ = source_id;
    int target_global_id_ = target_id;
    Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
    Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
    double attractive_gain = Attractive_gain_;
    double repulsive_gain = Repulsive_gain_;
    double bound_radiu_ = Bound_radiu_;

    // The parameter should be given by yaml.
    //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
    cost_map.clear();
    cost_unorder_map.clear();

    get_cost_map(planning_boost_graph, &cost_map, &cost_unorder_map,
                 robot_global_id_, target_global_id_,
                 robot_pos_, target_pos_,
                 attractive_gain, repulsive_gain, bound_radiu_,
                 bound_obstacle_vector_);

    // update boost edge
    for (auto &iter_vertices_map2_ : global_graph_->vertices_map_)
    {

      if (iter_vertices_map2_.second->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }

      StateVec state_planning_new(iter_vertices_map2_.second->state[0], iter_vertices_map2_.second->state[1], iter_vertices_map2_.second->state[2], 0);
      std::vector<Vertex *> nearest_vertices_in_small_box2;
      graph_for_planning->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                  limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
      {

        if (iter_vertices_map2_.second->id == nearest_vertices_in_small_box2[jj]->id)
        {
          // std::cout << "same point in small box 2, continue" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
        double direction_norm = direction.norm();
        double force_middle_ = (cost_unorder_map[iter_vertices_map2_.second->id] + cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) / 2;
        double weight_middle = (direction_norm + force_middle_) / 2;
        // std::cout << "direction_norm is " << direction_norm << std::endl;
        // std::cout << "force_middle_ is " << force_middle_ << std::endl;

        planning_boost_graph->removeEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id);
        planning_boost_graph->addEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
      }
    }
  }
  else
  {
    ROS_ERROR("[get_planning_graph_INFO]: There is something wrong when running it.");
  }
}

void PPlanner::get_planning_graph(int source_id, int target_id,
                                  std::shared_ptr<Graph_For_Planning> planning_boost_graph)
{
  bound_obstacle_vector_.clear();

  if (planning_boost_graph != NULL)
  {
    planning_boost_graph->clear();
  }
  // int count_internal = 0;

  const Vertex *vertex_source_ = global_graph_->getVertex(source_id);
  const Vertex *vertex_target_ = global_graph_->getVertex(target_id);
  bool IfFirstVertexForBoostGraph_ = true;

  if (global_graph_->vertices_map_.size() > 0)
  {
    for (auto &iter_vertices_map_ : global_graph_->vertices_map_)
    {

      if (iter_vertices_map_.second->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }
      StateVec state_planning_new(iter_vertices_map_.second->state[0], iter_vertices_map_.second->state[1], iter_vertices_map_.second->state[2], 0);

      if (IfFirstVertexForBoostGraph_)
      {
        planning_boost_graph->addSourceVertex(iter_vertices_map_.second->id);
        IfFirstVertexForBoostGraph_ = false;
      }
      else
      {
        planning_boost_graph->addVertex(iter_vertices_map_.second->id);
      }

      if (iter_vertices_map_.second->is_bound)
      {
        Eigen::Vector3d obstacle(iter_vertices_map_.second->state[0],
                                 iter_vertices_map_.second->state[1],
                                 iter_vertices_map_.second->state[2]);
        bound_obstacle_vector_.push_back(obstacle);
      }
      else
      {
      }

      std::vector<Vertex *> nearest_vertices_in_small_box;
      global_graph_->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                             limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
      {

        if (iter_vertices_map_.second->id == nearest_vertices_in_small_box[j]->id)
        {
          // std::cout << "same point continue;" << std::endl;
          continue;
        }

        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box[j]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box[j]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box[j]->state[2]);
        double direction_norm = direction.norm();

        // std::cout << "nearest direction is" << direction_norm << std::endl;

        planning_boost_graph->removeEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id);
        planning_boost_graph->addEdge(iter_vertices_map_.second->id, nearest_vertices_in_small_box[j]->id, direction_norm);
      }
    }
    // generate cost map
    std::vector<double> cost_map;
    std::unordered_map<int, double> cost_unorder_map;
    int robot_global_id_ = source_id;
    int target_global_id_ = target_id;
    Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
    Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
    double attractive_gain = Attractive_gain_;
    double repulsive_gain = Repulsive_gain_;
    double bound_radiu_ = Bound_radiu_;

    // The parameter should be given by yaml.
    //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
    cost_map.clear();
    cost_unorder_map.clear();

    get_cost_map(planning_boost_graph, &cost_map, &cost_unorder_map,
                 robot_global_id_, target_global_id_,
                 robot_pos_, target_pos_,
                 attractive_gain, repulsive_gain, bound_radiu_,
                 bound_obstacle_vector_);

    // update boost edge
    for (auto &iter_vertices_map2_ : global_graph_->vertices_map_)
    {
      if (iter_vertices_map2_.second->is_obstacle == true)
      {
        continue;
      }
      else
      {
      }

      StateVec state_planning_new(iter_vertices_map2_.second->state[0], iter_vertices_map2_.second->state[1], iter_vertices_map2_.second->state[2], 0);
      std::vector<Vertex *> nearest_vertices_in_small_box2;
      global_graph_->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                             limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
      {

        if (iter_vertices_map2_.second->id == nearest_vertices_in_small_box2[jj]->id)
        {
          // std::cout << "same point in small box 2, continue" << std::endl;
          continue;
        }
        Eigen::Vector3d direction(state_planning_new[0] - nearest_vertices_in_small_box2[jj]->state[0],
                                  state_planning_new[1] - nearest_vertices_in_small_box2[jj]->state[1],
                                  state_planning_new[2] - nearest_vertices_in_small_box2[jj]->state[2]);
        double direction_norm = direction.norm();
        double force_middle_ = (cost_unorder_map[iter_vertices_map2_.second->id] + cost_unorder_map[nearest_vertices_in_small_box2[jj]->id]) / 2;
        double weight_middle = (direction_norm + force_middle_) / 2;
        // std::cout << "direction_norm is " << direction_norm << std::endl;
        // std::cout << "force_middle_ is " << force_middle_ << std::endl;
        planning_boost_graph->removeEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id);
        planning_boost_graph->addEdge(iter_vertices_map2_.second->id, nearest_vertices_in_small_box2[jj]->id, weight_middle);
      }
    }
  }
  else
  {
    ROS_ERROR("[get_planning_graph_INFO]: There is something wrong when running it.");
  }
}

void PPlanner::get_cost_map(const std::shared_ptr<Graph_For_Planning> planning_boost_graph,
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
  // ROS_INFO("get_cost_map_neibu DI YI GUAN");
  std::vector<double> cost_map_;
  std::unordered_map<int, double> cost_unorder_map_;
  cost_unorder_map_.clear();
  cost_unorder_map->clear();
  cost_map_.clear();
  cost_map_.resize(planning_boost_graph->getNumVertices());
  cost_map->resize(planning_boost_graph->getNumVertices());
  int robot_internal_id_ = planning_boost_graph->get_id_internal(robot_global_id_);
  int target_internal_id_ = planning_boost_graph->get_id_internal(target_global_id_);
  // ROS_INFO("get_cost_map_neibu DI ER GUAN");
  //  Calculate the shortest path distance from each vertex to the goal
  std::vector<int> parent(planning_boost_graph->getNumVertices());
  std::vector<double> d(planning_boost_graph->getNumVertices());
  // boost::dijkstra_shortest_paths(graph_, vertex_descriptors_[target_global_id_], boost::predecessor_map(&parent[0]).distance_map(&d[0]));
  planning_boost_graph->dijkstra_process(target_global_id_, &parent, &d);

  // ROS_INFO("get_cost_map_neibu DI SAN GUAN");

  const Vertex *vertex_robot_ = global_graph_->getVertex(robot_global_id_);
  const Vertex *vertex_target_ = global_graph_->getVertex(target_global_id_);

  // ROS_INFO("get_cost_map_neibu DI SI GUAN");

  // Eigen::Vector3d robot_pos_(vertex_robot_->state[0], vertex_robot_->state[1], vertex_robot_->state[2]);
  // Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);

  std::pair<Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator, Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator> vi;
  planning_boost_graph->getVertexIterator(vi);
  for (Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    // ROS_INFO("get_cost_map_neibu DI SI.1 GUAN");
    int id = planning_boost_graph->get_id_global(planning_boost_graph->getVertexProperty(it));
    // int id = planning_boost_graph->getVertexProperty(it);
    //    What is the id? internal or global?. The following code is in the premise that it is global.
    const Vertex *vertex_temp_ = global_graph_->getVertex(id);
    // const Vertex *vertex_temp_ = Big_global_graph_with_obstacles_->getVertex(id);
    // ROS_INFO("get_cost_map_neibu DI SI.2 GUAN");
    // std::cout << "vertex_temp_ is " << vertex_temp_->state;
    Eigen::Vector3d point_pos_(vertex_temp_->state[0], vertex_temp_->state[1], vertex_temp_->state[2]);
    // ROS_INFO("get_cost_map_neibu DI SI.3 GUAN");
    Eigen::Vector3d total_force_;
    planning_boost_graph->get_total_force(point_pos_,
                                          target_pos_,
                                          bound_obstacle_vector,
                                          &total_force_,
                                          attractive_gain,
                                          repulsive_gain,
                                          bound_radiu_);
    // ROS_INFO("get_cost_map_neibu DI SI.4 GUAN");
    double total_force_norm = total_force_.norm();
    int id_internal_temp = planning_boost_graph->get_id_internal(id);
    double distance = d[id_internal_temp];
    cost_unorder_map_[id] = total_force_norm;
    cost_map_[id_internal_temp] = total_force_norm;
    // ROS_INFO("get_cost_map_neibu DI SI.5 GUAN");
    //  std::cout << "id is " << id << " and the total_force_norm is " << total_force_norm << std::endl;

    // cost_unorder_map_[id] = total_force_norm * distance;
    // cost_map_[id_internal_temp] = total_force_norm * distance;
  }
  *cost_unorder_map = cost_unorder_map_;
  *cost_map = cost_map_;
  // ROS_INFO("get_cost_map_neibu DI WU GUAN");
}

void PPlanner::get_cost_map(const std::shared_ptr<GraphManager> source_graph_manager,
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

void PPlanner::get_cost_map(GraphManager *source_graph_manager,
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

void PPlanner::visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_)
{
  GraphManager_check(GraphManager_);

  std::shared_ptr<Graph> graph_for_visualize = GraphManager_->Graph_;
  std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager_->vertices_map_;

  if (GraphManager_->getNumVertices() == 0)
  {
    return;
  }
  // if (GraphManager_pub_.getNumSubscribers() < 1)
  // {
  //   return;
  // }
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_graphmanager";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker vertex_bound_marker;
  // visualization_msgs::Marker vertex_bound_marker;
  vertex_bound_marker.header.stamp = ros::Time::now();
  vertex_bound_marker.header.seq = 0;
  vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_bound_marker.id = 0;
  vertex_bound_marker.ns = "vertices_bound_graphmanager";
  vertex_bound_marker.action = visualization_msgs::Marker::ADD;
  vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_bound_marker.scale.x = pplaner_map_resolution;
  vertex_bound_marker.scale.y = pplaner_map_resolution;
  vertex_bound_marker.scale.z = 0.01;
  vertex_bound_marker.color.r = 0.0 / 255.0;
  vertex_bound_marker.color.g = 0.0 / 255.0;
  vertex_bound_marker.color.b = 255.0 / 255.0;
  vertex_bound_marker.color.a = 1.0;
  vertex_bound_marker.lifetime = ros::Duration(0.0);
  vertex_bound_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    // TODO This id might be id_global in 99 percent certainty by std::cout. Why in other site it might be ind internal;
    int id = graph_for_visualize->getVertexProperty(it);
    int id_global = graph_for_visualize->get_id_global(id);
    id = id_global;
    // std::cout << "graph_for_visualize id is " << id << std::endl;
    //  int id = graph_for_visualize->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];
    if (v_map_visualize[id]->is_bound)
    {
      vertex_bound_marker.points.push_back(p1);
    }
    else
    {
      vertex_marker.points.push_back(p1);
    }
  }

  GraphManager_pub_.publish(vertex_marker);
  GraphManager_pub_.publish(vertex_bound_marker);

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = "edges_GrahManager";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[src_id]->state[0];
    p1.y = v_map_visualize[src_id]->state[1];
    p1.z = v_map_visualize[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize[tgt_id]->state[0];
    p2.y = v_map_visualize[tgt_id]->state[1];
    p2.z = v_map_visualize[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  GraphManager_pub_.publish(edge_marker);
}

void PPlanner::visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_, std::shared_ptr<ros::Publisher> TopicPublisher)
{
  GraphManager_check(GraphManager_);
  std::shared_ptr<Graph> graph_for_visualize = GraphManager_->Graph_;
  std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager_->vertices_map_;

  if (GraphManager_->getNumVertices() == 0)
  {
    return;
  }
  if (TopicPublisher->getNumSubscribers() < 1)
  {
    return;
  }
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_graphmanager";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker vertex_bound_marker;
  // visualization_msgs::Marker vertex_bound_marker;
  vertex_bound_marker.header.stamp = ros::Time::now();
  vertex_bound_marker.header.seq = 0;
  vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_bound_marker.id = 0;
  vertex_bound_marker.ns = "vertices_bound_graphmanager";
  vertex_bound_marker.action = visualization_msgs::Marker::ADD;
  vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_bound_marker.scale.x = pplaner_map_resolution;
  vertex_bound_marker.scale.y = pplaner_map_resolution;
  vertex_bound_marker.scale.z = 0.01;
  vertex_bound_marker.color.r = 0.0 / 255.0;
  vertex_bound_marker.color.g = 0.0 / 255.0;
  vertex_bound_marker.color.b = 255.0 / 255.0;
  vertex_bound_marker.color.a = 1.0;
  vertex_bound_marker.lifetime = ros::Duration(0.0);
  vertex_bound_marker.frame_locked = false;

  visualization_msgs::Marker vertex_frontier_marker;
  vertex_frontier_marker.header.stamp = ros::Time::now();
  vertex_frontier_marker.header.seq = 0;
  vertex_frontier_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_frontier_marker.id = 0;
  vertex_frontier_marker.ns = "vertices_bound_graphmanager";
  vertex_frontier_marker.action = visualization_msgs::Marker::ADD;
  vertex_frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_frontier_marker.scale.x = pplaner_map_resolution;
  vertex_frontier_marker.scale.y = pplaner_map_resolution;
  vertex_frontier_marker.scale.z = 0.01;
  vertex_frontier_marker.color.r = 255.0 / 255.0;
  vertex_frontier_marker.color.g = 0.0 / 255.0;
  vertex_frontier_marker.color.b = 0.0 / 255.0;
  vertex_frontier_marker.color.a = 1.0;
  vertex_frontier_marker.lifetime = ros::Duration(0.0);
  vertex_frontier_marker.frame_locked = false;

  visualization_msgs::Marker vertex_obstacles_marker;
  vertex_obstacles_marker.header.stamp = ros::Time::now();
  vertex_obstacles_marker.header.seq = 0;
  vertex_obstacles_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_obstacles_marker.id = 0;
  vertex_obstacles_marker.ns = "vertices_bound_graphmanager";
  vertex_obstacles_marker.action = visualization_msgs::Marker::ADD;
  vertex_obstacles_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_obstacles_marker.scale.x = pplaner_map_resolution;
  vertex_obstacles_marker.scale.y = pplaner_map_resolution;
  vertex_obstacles_marker.scale.z = 0.01;
  vertex_obstacles_marker.color.r = 255.0 / 255.0;
  vertex_obstacles_marker.color.g = 0.0 / 255.0;
  vertex_obstacles_marker.color.b = 255.0 / 255.0;
  vertex_obstacles_marker.color.a = 1.0;
  vertex_obstacles_marker.lifetime = ros::Duration(0.0);
  vertex_obstacles_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = graph_for_visualize->getVertexProperty(it);
    int id_global = graph_for_visualize->get_id_global(id);
    id = id_global;

    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];
    if (v_map_visualize[id]->is_obstacle)
    {
      vertex_obstacles_marker.points.push_back(p1);
    }
    else
    {

      if (v_map_visualize[id]->is_frontier)
      {
        vertex_frontier_marker.points.push_back(p1);
      }
      else
      {
        if (v_map_visualize[id]->is_bound)
        {
          vertex_bound_marker.points.push_back(p1);
        }
        else
        {
          vertex_marker.points.push_back(p1);
        }
      }
    }
  }

  TopicPublisher->publish(vertex_marker);
  TopicPublisher->publish(vertex_bound_marker);
  TopicPublisher->publish(vertex_frontier_marker);
  TopicPublisher->publish(vertex_obstacles_marker);

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = "edges_GrahManager";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[src_id]->state[0];
    p1.y = v_map_visualize[src_id]->state[1];
    p1.z = v_map_visualize[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize[tgt_id]->state[0];
    p2.y = v_map_visualize[tgt_id]->state[1];
    p2.z = v_map_visualize[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  if (edge_marker.points.size() < 1)
  {
  }
  else
  {
    TopicPublisher->publish(edge_marker);
  }
  // TopicPublisher->publish(edge_marker);
}

void PPlanner::visualizeGraphManager(std::shared_ptr<GraphManager> GraphManager_, std::shared_ptr<ros::Publisher> TopicPublisher, string ns_)
{
  GraphManager_check(GraphManager_);
  std::shared_ptr<Graph> graph_for_visualize = GraphManager_->Graph_;
  std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager_->vertices_map_;

  if (GraphManager_->getNumVertices() == 0)
  {
    return;
  }
  if (TopicPublisher->getNumSubscribers() < 1)
  {
    return;
  }
  std::string vertex_str_ = "vertex";
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = (ns_ + vertex_str_).c_str();
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = PplannerRoughGlobalMapResolution_;
  vertex_marker.scale.y = PplannerRoughGlobalMapResolution_;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  std::string vertex_bound_str_ = "vertex_bound";
  visualization_msgs::Marker vertex_bound_marker;
  // visualization_msgs::Marker vertex_bound_marker;
  vertex_bound_marker.header.stamp = ros::Time::now();
  vertex_bound_marker.header.seq = 0;
  vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_bound_marker.id = 0;
  vertex_bound_marker.ns = (ns_ + vertex_bound_str_).c_str();
  vertex_bound_marker.action = visualization_msgs::Marker::ADD;
  vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_bound_marker.scale.x = PplannerRoughGlobalMapResolution_;
  vertex_bound_marker.scale.y = PplannerRoughGlobalMapResolution_;
  vertex_bound_marker.scale.z = 0.01;
  vertex_bound_marker.color.r = 0.0 / 255.0;
  vertex_bound_marker.color.g = 0.0 / 255.0;
  vertex_bound_marker.color.b = 255.0 / 255.0;
  vertex_bound_marker.color.a = 1.0;
  vertex_bound_marker.lifetime = ros::Duration(0.0);
  vertex_bound_marker.frame_locked = false;

  std::string vertex_frontier_str_ = "vertex_frontier";
  visualization_msgs::Marker vertex_frontier_marker;
  vertex_frontier_marker.header.stamp = ros::Time::now();
  vertex_frontier_marker.header.seq = 0;
  vertex_frontier_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_frontier_marker.id = 0;
  vertex_frontier_marker.ns = (ns_ + vertex_frontier_str_).c_str();
  vertex_frontier_marker.action = visualization_msgs::Marker::ADD;
  vertex_frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_frontier_marker.scale.x = PplannerRoughGlobalMapResolution_;
  vertex_frontier_marker.scale.y = PplannerRoughGlobalMapResolution_;
  vertex_frontier_marker.scale.z = 0.01;
  vertex_frontier_marker.color.r = 255.0 / 255.0;
  vertex_frontier_marker.color.g = 0.0 / 255.0;
  vertex_frontier_marker.color.b = 0.0 / 255.0;
  vertex_frontier_marker.color.a = 1.0;
  vertex_frontier_marker.lifetime = ros::Duration(0.0);
  vertex_frontier_marker.frame_locked = false;

  std::string vertex_obstacle_str_ = "vertex_obstacle";
  visualization_msgs::Marker vertex_obstacles_marker;
  vertex_obstacles_marker.header.stamp = ros::Time::now();
  vertex_obstacles_marker.header.seq = 0;
  vertex_obstacles_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_obstacles_marker.id = 0;
  vertex_obstacles_marker.ns = (ns_ + vertex_obstacle_str_).c_str();
  vertex_obstacles_marker.action = visualization_msgs::Marker::ADD;
  vertex_obstacles_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_obstacles_marker.scale.x = PplannerRoughGlobalMapResolution_;
  vertex_obstacles_marker.scale.y = PplannerRoughGlobalMapResolution_;
  vertex_obstacles_marker.scale.z = 0.01;
  vertex_obstacles_marker.color.r = 255.0 / 255.0;
  vertex_obstacles_marker.color.g = 0.0 / 255.0;
  vertex_obstacles_marker.color.b = 255.0 / 255.0;
  vertex_obstacles_marker.color.a = 1.0;
  vertex_obstacles_marker.lifetime = ros::Duration(0.0);
  vertex_obstacles_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = graph_for_visualize->getVertexProperty(it);
    int id_global = graph_for_visualize->get_id_global(id);
    id = id_global;

    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];
    if (v_map_visualize[id]->is_obstacle)
    {
      vertex_obstacles_marker.points.push_back(p1);
    }
    else
    {
      if (v_map_visualize[id]->is_frontier)
      {
        vertex_frontier_marker.points.push_back(p1);
      }
      else
      {
        if (v_map_visualize[id]->is_bound)
        {
          vertex_bound_marker.points.push_back(p1);
        }
        else
        {
          vertex_marker.points.push_back(p1);
        }
      }
    }
  }

  TopicPublisher->publish(vertex_marker);
  // TopicPublisher->publish(vertex_bound_marker);
  // TopicPublisher->publish(vertex_frontier_marker);
  TopicPublisher->publish(vertex_obstacles_marker);

  // Plot all edges
  std::string edge_str_ = "edge";
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = (ns_ + edge_str_).c_str();
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[src_id]->state[0];
    p1.y = v_map_visualize[src_id]->state[1];
    p1.z = v_map_visualize[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize[tgt_id]->state[0];
    p2.y = v_map_visualize[tgt_id]->state[1];
    p2.z = v_map_visualize[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  if (edge_marker.points.size() < 1)
  {
  }
  else
  {
    TopicPublisher->publish(edge_marker);
  }
  // TopicPublisher->publish(edge_marker);
}

bool PPlanner::Find_shortest_path_by_dijkstra(int start_id,
                                              ShortestPathsReportForPlanning &rep,
                                              std::shared_ptr<Graph_For_Planning> Planning_boost_graph)
{
  return Planning_boost_graph->findDijkstraShortestPaths(start_id, rep);
}

void PPlanner::getShortestPath(int target_id,
                               const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order,
                               std::vector<Vertex *> &path)
{
  path.clear();
  std::vector<int> path_id;
  path_id.clear();
  getShortestPath(target_id, rep, source_to_target_order, &path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p)
  {
    path.push_back(global_graph_->vertices_map_[*p]);
  }
}

void PPlanner::getShortestPath(int target_id, const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order, std::vector<Vertex *> &path,
                               std::shared_ptr<GraphManager> Graph_Manager)
{
  path.clear();
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p)
  {
    path.push_back(Graph_Manager->vertices_map_[*p]);
  }
}

void PPlanner::getShortestPath(int target_id,
                               const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order,
                               std::vector<Eigen::Vector3d> &path)
{
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p)
  {
    path.emplace_back(Eigen::Vector3d(global_graph_->vertices_map_[*p]->state.x(),
                                      global_graph_->vertices_map_[*p]->state.y(),
                                      global_graph_->vertices_map_[*p]->state.z()));
  }
}

void PPlanner::getShortestPath(int target_id,
                               const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order,
                               std::vector<StateVec> &path)
{
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p)
  {
    path.emplace_back(global_graph_->vertices_map_[*p]->state);
  }
}

void PPlanner::getShortestPath(int target_id,
                               const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order,
                               std::vector<int> &path)
{
  // ros::Time t_qian = ros::Time::now();
  path.clear();
  if (!rep.status)
  {
    // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
    ROS_ERROR("Shortest paths report is not valid");

    return;
  }

  // if (rep.parent_id_map.size() <= target_id)
  // {
  //   ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertext with ID [%d] doesn't exist in the graph", target_id);
  //   return;
  // }

  if (target_id == rep.source_id)
  {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id)
  {
    // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] is isolated from the graph", target_id);
    ROS_ERROR("Vertex with ID [%d] is isolated from the graph", target_id);
    // visualizePlanningBoostGraph(planning_boost_graph_);

    return;
  }

  // ros::Time t1 = ros::Time::now();
  // double t1_used = t1.toSec() - t_qian.toSec();
  // ROS_INFO("t1_used is %f", t1_used);

  path.push_back(target_id); // current vertex id first
  path.push_back(parent_id); // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id)
  {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // ros::Time t2 = ros::Time::now();
  // double t2_used = t2.toSec() - t_qian.toSec();
  // ROS_INFO("t2_used is %f", t2_used);

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order)
  {
    std::reverse(path.begin(), path.end());
  }

  // ros::Time t_hou = ros::Time::now();
  // double t_used = t_hou.toSec() - t_qian.toSec();
  // ROS_INFO("Get shortest path succeed and time used is %f", t_used);
}

void PPlanner::getShortestPath(int target_id,
                               const ShortestPathsReportForPlanning &rep,
                               bool source_to_target_order,
                               std::vector<int> *path)
{

  path->clear();
  if (!rep.status)
  {
    ROS_ERROR("Shortest paths report is not valid");
    return;
  }

  if (target_id == rep.source_id)
  {
    path->push_back(target_id);
    return;
  }
  int parent_id = -1;
  try
  {
    parent_id = rep.parent_id_map.at(target_id);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    std::cerr << e.what() << '\n';
    return;
  }

  if (parent_id == target_id)
  {
    // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] is isolated from the graph", target_id);
    ROS_ERROR("Vertex with ID [%d] is isolated from the graph", target_id);
    // visualizePlanningBoostGraph(planning_boost_graph_);

    return;
  }

  path->push_back(target_id); // current vertex id first
  path->push_back(parent_id); // its first parent, the rest is recursively looked up
  int id_size_max_ = 0;
  while (parent_id != rep.source_id)
  {
    id_size_max_++;
    if ((parent_id == rep.parent_id_map.at(parent_id)) || (id_size_max_ > rep.parent_id_map.size()))
    {
      ROS_ERROR("The parent of ID [%d] is itself or the loop time is bigger.", parent_id);
      path->clear();
      return;
    }
    parent_id = rep.parent_id_map.at(parent_id);
    path->push_back(parent_id);
  }

  if (source_to_target_order)
  {
    std::reverse(path->begin(), path->end());
  }
}

double PPlanner::getShortestDistance(int target_id,
                                     const ShortestPathsReportForPlanning &rep)
{
  double dist = std::numeric_limits<double>::max();

  if (!rep.status)
  {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id)
  {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] doesn't exist in the graph", target_id);
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

void PPlanner::visualizePath(std::vector<Vertex *> Path)
{
  if (Path.size() == 0)
  {
    return;
  }
  path_tracked_vector_.clear();
  // if (Global_Path_pub_.getNumSubscribers() < 1)
  // {
  //   return;
  // }

  visualization_msgs::MarkerArray Path_marker_array;

  visualization_msgs::Marker Path_edge_marker;
  Path_edge_marker.header.stamp = ros::Time::now();
  Path_edge_marker.header.seq = 0;
  Path_edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  Path_edge_marker.id = 0;
  Path_edge_marker.ns = "global_path_edges";
  Path_edge_marker.action = visualization_msgs::Marker::ADD;
  Path_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  Path_edge_marker.scale.x = 0.25;
  Path_edge_marker.color.r = 0.957;
  Path_edge_marker.color.g = 66.0 / 255.0;
  Path_edge_marker.color.b = 226.0 / 255.0;
  Path_edge_marker.color.a = 1.0;
  Path_edge_marker.lifetime = ros::Duration(0.0);
  Path_edge_marker.frame_locked = false;

  visualization_msgs::Marker Path_vertex_marker;
  Path_vertex_marker.header.stamp = ros::Time::now();
  Path_vertex_marker.header.seq = 0;
  Path_vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  Path_vertex_marker.id = 0;
  Path_vertex_marker.ns = "global_path_vertices";
  Path_vertex_marker.action = visualization_msgs::Marker::ADD;
  Path_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  Path_vertex_marker.scale.x = 0.25;
  Path_vertex_marker.scale.y = 0.25;
  Path_vertex_marker.scale.z = 0.25;
  Path_vertex_marker.color.r = 200.0 / 255.0;
  Path_vertex_marker.color.g = 100.0 / 255.0;
  Path_vertex_marker.color.b = 0.0;
  Path_vertex_marker.color.a = 1.0;
  Path_vertex_marker.lifetime = ros::Duration(0.0);
  Path_vertex_marker.frame_locked = false;

  for (size_t i = 0; i < (Path.size() - 1); ++i)
  {
    geometry_msgs::Point p1;
    p1.x = Path[i]->state[0];
    p1.y = Path[i]->state[1];
    p1.z = Path[i]->state[2] + 0.5;
    geometry_msgs::Point p2;
    p2.x = Path[i + 1]->state[0];
    p2.y = Path[i + 1]->state[1];
    p2.z = Path[i + 1]->state[2] + 0.5;
    Path_edge_marker.points.push_back(p1);
    Path_edge_marker.points.push_back(p2);
    Path_vertex_marker.points.push_back(p1);
    Eigen::Vector3d path_point_(Path[i]->state[0],
                                Path[i]->state[1],
                                Path[i]->state[2]);
    path_tracked_vector_.push_back(path_point_);
    if (i == (Path.size() - 2))
    {
      Path_vertex_marker.points.push_back(p2);

      Eigen::Vector3d path_point2_(Path[i + 1]->state[0],
                                   Path[i + 1]->state[1],
                                   Path[i + 1]->state[2]);
      path_tracked_vector_.push_back(path_point2_);
    }
    else
    {
    }
  }
  Path_marker_array.markers.push_back(Path_edge_marker);
  Path_marker_array.markers.push_back(Path_vertex_marker);
  Global_Path_pub_.publish(Path_marker_array);
}

void PPlanner::a_star_serach(std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                             int start_global_id,
                             std::vector<double> costmap)
{

  // std::vector<Graph_For_Planning::VertexDescriptor> p(boost::num_vertices(planning_boost_graph->graph_)); // Predecessor map
  // std::vector<double> d(boost::num_vertices(planning_boost_graph->graph_));                               // Distance map

  // boost::astar_search(planning_boost_graph->graph_,
  //                     planning_boost_graph->vertex_descriptors_[start_global_id],
  //                     heuristic_for_planning,
  //                     boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(boost::null_visitor()),
  //                     boost::make_transform_value_property_map([&costmap](double c)
  //                                                              { return c; },
  //                                                              boost::get(boost::vertex_index, planning_boost_graph->graph_)),
  //                     boost::default_astar_visitor());
}

double PPlanner::heuristic_for_planning(Graph_For_Planning::VertexDescriptor u, Graph_For_Planning::VertexDescriptor v)
{
  return 0.0;
}

void PPlanner::visualizePlanningBoostGraph(std::shared_ptr<Graph_For_Planning> planning_boost_graph)
{
  // std::cout << "print boost graph." << std::endl;
  //  std::ofstream dotfile;
  //  dotfile.open("/home/december/pplanner_middle/planning_boost_graph.dot");
  //  boost::write_graphviz(dotfile, planning_boost_graph->graph_);

  std::shared_ptr<Graph_For_Planning> graph_for_visualize = planning_boost_graph;
  const std::unordered_map<int, Vertex *> &v_map_visualize = global_graph_->vertices_map_;

  if (planning_boost_graph->getNumVertices() == 0)
  {
    return;
  }
  // // if (Planning_Boost_pub_.getNumSubscribers() < 1)
  // // {
  // //   return;
  // // }
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_planning_boost";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  // visualization_msgs::Marker vertex_bound_marker;
  // // visualization_msgs::Marker vertex_bound_marker;
  // vertex_bound_marker.header.stamp = ros::Time::now();
  // vertex_bound_marker.header.seq = 0;
  // vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
  // vertex_bound_marker.id = 0;
  // vertex_bound_marker.ns = "vertices_bound_graphmanager";
  // vertex_bound_marker.action = visualization_msgs::Marker::ADD;
  // vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // vertex_bound_marker.scale.x = pplaner_map_resolution;
  // vertex_bound_marker.scale.y = pplaner_map_resolution;
  // vertex_bound_marker.scale.z = 0.01;
  // vertex_bound_marker.color.r = 0.0 / 255.0;
  // vertex_bound_marker.color.g = 0.0 / 255.0;
  // vertex_bound_marker.color.b = 255.0 / 255.0;
  // vertex_bound_marker.color.a = 1.0;
  // vertex_bound_marker.lifetime = ros::Duration(0.0);
  // vertex_bound_marker.frame_locked = false;

  std::pair<Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator, Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    // TODO This id might be id_internal in 99 percent certainty by std::cout. Why in other site it might be ind internal;Because it is boost::vertex_index.
    int id = graph_for_visualize->get_id_global(graph_for_visualize->getVertexProperty(it));
    // std::cout << "graph_for_visualize id is " << id << std::endl;
    //  int id = graph_for_visualize->getVertexProperty(it);
    geometry_msgs::Point p1;

    p1.x = v_map_visualize.at(id)->state[0];
    p1.y = v_map_visualize.at(id)->state[1];
    p1.z = v_map_visualize.at(id)->state[2];
    vertex_marker.points.push_back(p1);
    // if (v_map_visualize[id]->is_bound)
    // {
    //   vertex_bound_marker.points.push_back(p1);
    // }
    // else
    // {
    //   vertex_marker.points.push_back(p1);
    // }
  }

  Planning_Boost_pub_.publish(vertex_marker);
  // GraphManager_pub_.publish(vertex_bound_marker);

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = "edges_PlanningBoost";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph_For_Planning::GraphType_For_PLANNING::edge_iterator, Graph_For_Planning::GraphType_For_PLANNING::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph_For_Planning::GraphType_For_PLANNING::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    int src_id_global = graph_for_visualize->get_id_global(src_id);
    int tgt_id_global = graph_for_visualize->get_id_global(tgt_id);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize.at(src_id_global)->state[0];
    p1.y = v_map_visualize.at(src_id_global)->state[1];
    p1.z = v_map_visualize.at(src_id_global)->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize.at(tgt_id_global)->state[0];
    p2.y = v_map_visualize.at(tgt_id_global)->state[1];
    p2.z = v_map_visualize.at(tgt_id_global)->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  Planning_Boost_pub_.publish(edge_marker);
}

void PPlanner::update_tracker_orientation(const ros::TimerEvent &event)
{
  // std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);
  // ROS_INFO("[update_tracker_orientationTimer_INFO]:START.");
  checkPpathTrackerIfCouldTurn(ppath_tracker_);
  checkPpathTrackerIfCouldTurn(Local_ppath_tracker_);
  // ROS_INFO("[update_tracker_orientationTimer_INFO]:END.");

  /*
    if ((ppath_tracker_->getStatus()) || (ppath_tracker_->getIfPause()))
    {
      if (ppath_tracker_->get_x_of_cmd_vel() < 0)
      {
        std::cout << "ppath_tracker_->get_x_of_cmd_vel() is " << ppath_tracker_->get_x_of_cmd_vel() << std::endl;

        bool if_could_turn = if_robot_could_around();
        // std::cout << "if_could_turn is " << if_could_turn << std::endl;
        if (if_could_turn)
        {
          is_turning = true;
          ppath_tracker_->pauseTracker();

          try
          {
            world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
            world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                                ros::Time(0), world_base_transform);
          }
          catch (tf::TransformException &ex)
          {
            ROS_ERROR("There is something wrong when trying get robot pose to get target orientation.");
            ROS_ERROR("%s", ex.what());
            // ros::Duration(1.0).sleep();
          }
          tf::Quaternion q_robot_target = world_base_transform.getRotation();
          double yaw_target_ = tf::getYaw(q_robot_target); // - M_PI;
          if (yaw_target_ < 0)
          {
            yaw_target_ = yaw_target_ + M_PI;
          }
          else
          {
            yaw_target_ = yaw_target_ - M_PI;
          }
          // std::cout << "yaw_target_ is " << yaw_target_ << std::endl;

          let_robot_turn_around_z_axe(yaw_target_);
          ppath_tracker_->continueTracker();
          usleep(1000000);
          is_turning = false;
        }
        else
        {
        }
      }
      else
      {
      }
    }
    else
    {
    }
  */
}

bool PPlanner::if_robot_could_around()
{
  std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("There is something wrong when trying get robot pose in the check if_robot_could_around function.");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  Vertex *robot_nearest_vertices;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    if (robot_nearest_vertices->is_obstacle)
    {
      ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle when check if_robot_could_around.");
      robot_nearest_vertices->is_obstacle = false;
      robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
    }

    int id_robot = robot_nearest_vertices->id;

    int vertex_count = 0;

    std::unordered_map<int, Graph_For_Planning::VertexDescriptor> vertex_descriptors_middle_;
    vertex_descriptors_middle_.clear();
    vertex_descriptors_middle_[id_robot] = planning_boost_graph_->vertex_descriptors_[id_robot];

    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it1_, end1_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it2_, end2_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it3_, end3_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it4_, end4_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it5_, end5_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it6_, end6_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it7_, end7_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it8_, end8_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it9_, end9_;
    Graph_For_Planning::GraphType_For_PLANNING::adjacency_iterator it10_, end10_;
    for (boost::tie(it1_, end1_) = boost::adjacent_vertices(planning_boost_graph_->vertex_descriptors_[id_robot], planning_boost_graph_->graph_); it1_ != end1_; ++it1_)
    {
      int id_global = planning_boost_graph_->get_id_global(planning_boost_graph_->getVertexID(*it1_));
      vertex_descriptors_middle_[id_global] = *it1_;

      for (boost::tie(it2_, end2_) = boost::adjacent_vertices(*it1_, planning_boost_graph_->graph_); it2_ != end2_; ++it2_)
      {
        int id_global2 = planning_boost_graph_->get_id_global(planning_boost_graph_->getVertexID(*it2_));
        vertex_descriptors_middle_[id_global2] = *it2_;

        for (boost::tie(it3_, end3_) = boost::adjacent_vertices(*it2_, planning_boost_graph_->graph_); it3_ != end3_; ++it3_)
        {
          int id_global3 = planning_boost_graph_->get_id_global(planning_boost_graph_->getVertexID(*it3_));
          vertex_descriptors_middle_[id_global3] = *it3_;

          for (boost::tie(it4_, end4_) = boost::adjacent_vertices(*it3_, planning_boost_graph_->graph_); it4_ != end4_; ++it4_)
          {
            int id_global4 = planning_boost_graph_->get_id_global(planning_boost_graph_->getVertexID(*it4_));
            vertex_descriptors_middle_[id_global4] = *it4_;

            for (boost::tie(it5_, end5_) = boost::adjacent_vertices(*it4_, planning_boost_graph_->graph_); it5_ != end5_; ++it5_)
            {
              int id_global5 = planning_boost_graph_->get_id_global(planning_boost_graph_->getVertexID(*it5_));
              vertex_descriptors_middle_[id_global5] = *it5_;
              // if (*it5_ == *it3_)
              // {
              //   continue;
              // }
              // vertex_count++;
              // for (boost::tie(it6_, end6_) = boost::adjacent_vertices(*it5_, planning_boost_graph_->graph_); it6_ != end6_; ++it6_)
              // {
              //   if (*it6_ == *it4_)
              //   {
              //     continue;
              //   }
              //   vertex_count++;
              //   for (boost::tie(it7_, end7_) = boost::adjacent_vertices(*it6_, planning_boost_graph_->graph_); it7_ != end7_; ++it7_)
              //   {
              //     if (*it7_ == *it5_)
              //     {
              //       continue;
              //     }
              //     vertex_count++;
              //     for (boost::tie(it8_, end8_) = boost::adjacent_vertices(*it7_, planning_boost_graph_->graph_); it8_ != end8_; ++it8_)
              //     {
              //       if (*it8_ == *it6_)
              //       {
              //         continue;
              //       }
              //       vertex_count++;
              //       for (boost::tie(it9_, end9_) = boost::adjacent_vertices(*it8_, planning_boost_graph_->graph_); it9_ != end9_; ++it9_)
              //       {
              //         if (*it9_ == *it7_)
              //         {
              //           continue;
              //         }
              //         vertex_count++;
              //         for (boost::tie(it10_, end10_) = boost::adjacent_vertices(*it9_, planning_boost_graph_->graph_); it10_ != end10_; ++it10_)
              //         {
              //           if (*it10_ == *it8_)
              //           {
              //             continue;
              //           }
              //           vertex_count++;
              //         }
              //       }
              //     }
              //   }
              // }
            }
          }
        }
      }
      // Do something with the neighbor vertex descriptor ()
      // get_id_global(getVertexID(shortest_paths[*it]))
    }
    // ROS_INFO("Size of vertex_descriptors_middle_ is %ld", vertex_descriptors_middle_.size());
    double NumRobotSize = 2 * robot_width / pplaner_map_resolution;
    // if (vertex_descriptors_middle_.size() > 11 * 11 - 3)
    if (vertex_descriptors_middle_.size() > NumRobotSize * NumRobotSize - 3)
    {
      ROS_INFO("Size of vertex_descriptors_middle_ is %ld", vertex_descriptors_middle_.size());
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR("Could not find the nearest vertex when checking if_robot_could_around.");
    return false;
  }
  return false;
}

void PPlanner::let_robot_turn_around_z_axe(double yaw_target)
{
  bool if_stop = false;
  while (!if_stop)
  {
    try
    {
      world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
      world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                          ros::Time(0), world_base_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("There is something wrong when trying get robot pose in the let_robot_turn_around_z_axe function.");
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
    }
    tf::Quaternion q_robot_middle = world_base_transform.getRotation();
    double yaw_middle_ = tf::getYaw(q_robot_middle);
    // std::cout << "yaw_target_ is " << yaw_target << std::endl;
    // std::cout << "yaw_middle_ is " << yaw_middle_ << std::endl;
    if (abs(yaw_middle_ - yaw_target) < 0.5)
    {
      if_stop = true;
      continue;
    }
    else
    {
      geometry_msgs::Twist cmd_vel_;
      cmd_vel_.angular.z = 0.3;
      cmd_vel_pub_.publish(cmd_vel_);
    }
    usleep(100000);
  }
  return;
}

void PPlanner::let_robot_turn_around_z_axe(Eigen::Vector2d TargetAttitude)
{
  bool if_stop = false;
  double angular_z;
  if (TargetAttitude[0] == 1)
  {
    angular_z = 0.5;
  }
  else if (TargetAttitude[0] == -1)
  {
    angular_z = -0.5;
  }
  else
  {
    ROS_ERROR("There is something wrong with TargetAttitude.");
  }
  while (!if_stop)
  {
    try
    {
      world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
      world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                          ros::Time(0), world_base_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("There is something wrong when trying get robot pose in the let_robot_turn_around_z_axe function.");
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
    }

    tf::Quaternion q_robot_middle = world_base_transform.getRotation();
    double yaw_middle_ = tf::getYaw(q_robot_middle);
    // std::cout << "yaw_target_ is " << TargetAttitude[1] << std::endl;
    // std::cout << "yaw_middle_ is " << yaw_middle_ << std::endl;
    if (abs(yaw_middle_ - TargetAttitude[1]) < 0.5)
    {
      if_stop = true;
      break;
    }
    else
    {
      geometry_msgs::Twist cmd_vel_;
      cmd_vel_.angular.z = angular_z;
      cmd_vel_pub_.publish(cmd_vel_);
    }
    usleep(100000);
  }
  return;
}

void PPlanner::update_tracker_path_safety_circumvent(const ros::TimerEvent &event)
{
  // ROS_INFO("[update_tracker_path_safety_circumventTimer_INFO]:START.");
  if ((ppath_tracker_->getStatus()) || (ppath_tracker_->getIfPause()))
  {
    if ((ppath_tracker_->get_x_of_cmd_vel() > 0) && (!is_turning))
    {
      std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
      // double vel_x_middle = ppath_tracker_->get_x_of_cmd_vel();
      double vel_x_middle = 0.5;
      double forward_x = vel_x_middle * lookahead_time_for_safety_circumvent_;

      try
      {
        world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
        world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                            ros::Time(0), world_base_transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("There is something wrong when trying get robot pose in the update_tracker_path_safety_circumvent function.");
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
      }

      tf::StampedTransform world_base_transform_for_safety_circumvent = world_base_transform;
      Eigen::Vector2d robot_location_2d(world_base_transform_for_safety_circumvent.getOrigin().x(), world_base_transform_for_safety_circumvent.getOrigin().y());

      double forward_x_for_check_middle_;
      if (vel_x_middle > 0)
      {
        forward_x_for_check_middle_ = abs(forward_x) + x_forward_offset_for_safety_ckeck_;
      }
      else
      {
        forward_x_for_check_middle_ = abs(forward_x) + x_backward_offset_for_safety_ckeck_;
      }

      bool if_obstacles_exist =
          !check_if_pass_the_area(traversability_map,
                                  robot_location_2d,
                                  forward_x_for_check_middle_,
                                  0,
                                  world_base_transform_for_safety_circumvent);
      if (if_obstacles_exist)
      {

        ppath_tracker_->pauseTracker();
        std::vector<int> rotate_angles_vector;
        int free_angle_index = 0;
        bool if_find_safe_road_succeed = false;
        double rotate_angle_useful = 0.0;
        rotate_angles_vector.push_back(0);
        for (int i = 1; i < 19; ++i)
        {
          rotate_angles_vector.push_back(i);
          rotate_angles_vector.push_back(-i);
        }

        for (size_t i = 0; i < rotate_angles_vector.size(); ++i)
        {
          double rotate_angle_middle = M_PI / 36.0 * rotate_angles_vector[i];

          double forward_x_middle = vel_x_middle * lookahead_time_for_safety_circumvent_;

          bool robot_could_move =
              check_if_pass_the_area(traversability_map,
                                     robot_location_2d,
                                     forward_x_for_check_middle_,
                                     rotate_angle_middle,
                                     world_base_transform_for_safety_circumvent);

          if (robot_could_move)
          {
            if_find_safe_road_succeed = true;
            free_angle_index = i;
            rotate_angle_useful = rotate_angle_middle;
            ROS_INFO("free_angle_index is %d.", free_angle_index);
            ROS_INFO("rotate_angles_vector[free_angle_index]is %d.", rotate_angles_vector[free_angle_index]);
            ROS_INFO("rotate_angle_useful is %f.", rotate_angle_useful);
            break;
          }
          else
          {
            continue;
          }
        }
        if (if_find_safe_road_succeed)
        {
          ROS_INFO("Find the free_angle_index and the value is %d.", free_angle_index);
          ROS_INFO("Find the rotate_angles_vector[free_angle_index] and the value is %d.", rotate_angles_vector[free_angle_index]);
          ROS_INFO("Find the rotate_angle_useful and the value is %f.", rotate_angle_useful);

          if (rotate_angle_useful == 0)
          {
            ROS_INFO("I can not understand the free_angle_index is %d, why the algorithm go there.", free_angle_index);

            ppath_tracker_->continueTracker();
          }
          else
          {
            double cmd_vel_x_corrected_by_detecting_obstacle;
            cmd_vel_x_corrected_by_detecting_obstacle = vel_x_middle * 0.8 * forward_x * (18 - std::abs(rotate_angles_vector[free_angle_index])) / 18.0;
            double cmd_vel_y_corrected_by_detecting_obstacle;
            if (rotate_angles_vector[free_angle_index] > 0)
            {
              cmd_vel_y_corrected_by_detecting_obstacle = 0.1;
            }
            else
            {
              cmd_vel_y_corrected_by_detecting_obstacle = -0.1;
            }
            double cmd_angle_z_corrected_by_detecting_obstacle;
            cmd_angle_z_corrected_by_detecting_obstacle = M_PI / 36.0 * rotate_angles_vector[free_angle_index] * (1 / forward_x);

            if (cmd_angle_z_corrected_by_detecting_obstacle > 0.5)
            {
              cmd_angle_z_corrected_by_detecting_obstacle = 0.5;
            }
            else if (cmd_angle_z_corrected_by_detecting_obstacle < -0.5)
            {
              cmd_angle_z_corrected_by_detecting_obstacle = -0.5;
            }
            else
            {
            }

            if (cmd_vel_x_corrected_by_detecting_obstacle > 0.5)
            {
              cmd_vel_x_corrected_by_detecting_obstacle = 0.5;
            }
            else if (cmd_vel_x_corrected_by_detecting_obstacle < -0.5)
            {
              cmd_vel_x_corrected_by_detecting_obstacle = -0.5;
            }
            else
            {
            }
            geometry_msgs::Twist cmd_vel_;
            cmd_vel_.linear.x = cmd_vel_x_corrected_by_detecting_obstacle;
            cmd_vel_.linear.y = cmd_vel_y_corrected_by_detecting_obstacle;
            cmd_vel_.angular.z = cmd_angle_z_corrected_by_detecting_obstacle;
            ROS_INFO("cmd_vel_.linear.x is %f.", cmd_vel_.linear.x);
            ROS_INFO("cmd_vel_.angular.z is %f.", cmd_vel_.angular.z);
            cmd_vel_pub_.publish(cmd_vel_);
            usleep(100000);
            ppath_tracker_->continueTracker();
          }
        }
        else
        {
          ROS_ERROR("Sorry, there is obstacles in my path and I could not find the free_angle_index.");
          ROS_ERROR("I guess there is a bad guy who is molesting me, how boring he is.");
          ROS_ERROR("And I will stop the tracker. Waiting for your next target id.");

          // ppath_tracker_->stopTracker();
        }
      }
      else
      {
      }
    }
  }
  // ROS_INFO("[update_tracker_path_safety_circumventTimer_INFO]:END.");
}

bool PPlanner::check_if_pass_the_area(grid_map::GridMap traversability_map_fun,
                                      Eigen::Vector2d robot_location,
                                      double x_expand_,
                                      double angle,
                                      tf::StampedTransform world_base_transform_for_safety_circumvent)
{
  // std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
  double target_x_in_robot_frame_ = x_expand_ * cos(angle);
  double target_y_in_robot_frame_ = x_expand_ * sin(angle);

  tf::Quaternion q_robot_middle = world_base_transform_for_safety_circumvent.getRotation();
  double yaw_middle_ = tf::getYaw(q_robot_middle);

  double target_x_in_world_frame_ = robot_location[0] +
                                    target_x_in_robot_frame_ * cos(yaw_middle_) -
                                    target_y_in_robot_frame_ * sin(yaw_middle_);
  double target_y_in_world_frame_ = robot_location[1] +
                                    target_x_in_robot_frame_ * sin(yaw_middle_) +
                                    target_y_in_robot_frame_ * cos(yaw_middle_);

  Eigen::Vector2d start_pos_0(robot_location[0], robot_location[1]);
  Eigen::Vector2d end_pos_0(target_x_in_world_frame_, target_y_in_world_frame_);

  int size_in_y = robot_width / tra_map_resolution;
  int size_y_used = size_in_y / 2;

  int bad_point_count_ = 0;

  for (int i = -size_y_used; i < size_y_used + 1; i++)
  {
    double y_change_in_robot_frame_ = i * tra_map_resolution;
    double x_start_in_body_frame_ = y_change_in_robot_frame_ * (-sin(angle));
    double y_start_in_body_frame = y_change_in_robot_frame_ * (cos(angle));
    double x_start_in_world_frame = robot_location[0] +
                                    x_start_in_body_frame_ * cos(yaw_middle_) -
                                    y_start_in_body_frame * sin(yaw_middle_);
    double y_start_in_world_frame = robot_location[1] +
                                    x_start_in_body_frame_ * sin(yaw_middle_) +
                                    y_start_in_body_frame * cos(yaw_middle_);

    Eigen::Vector2d start_middle(x_start_in_world_frame, y_start_in_world_frame);
    Eigen::Vector2d end_middle = end_pos_0 - start_pos_0 + start_middle;
    try
    {
      for (grid_map::LineIterator iterator(traversability_map_fun, start_middle, end_middle);
           !iterator.isPastEnd(); ++iterator)
      {
        double elevation = traversability_map_fun.at(Elevation_layer_.c_str(), *iterator);
        double traversability_score = traversability_map_fun.at(Traversability_layer_.c_str(), *iterator);
        double traversability_supplementary_score =
            traversability_map_fun.at(Traversability_supplementary_layer_.c_str(), *iterator);

        traversability_score = std::max(traversability_score, traversability_supplementary_score);
        if (elevation != elevation)
        {
          // bad_point_count_++;
          continue;
        }
        else
        {
        }

        if (traversability_score != traversability_score)
        {
          // bad_point_count_++;
          continue;
        }
        else
        {
        }

        if (traversability_score < traversability_score_threshold_low_bound)
        {
          bad_point_count_++;
          continue;
        }
        else
        {
        }
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
  }

  // std::cout << "angle is " << angle << " and bad_point_count_ is " << bad_point_count_ << std::endl;

  if (bad_point_count_ > 1)
  {
    return false;
  }
  else
  {
    return true;
  }

  return false;
}

void PPlanner::test_fun()
{
  double vel_x_middle = 0.5;
  double forward_x = vel_x_middle * lookahead_time_for_safety_circumvent_;

  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("There is something wrong when trying get robot pose in the update_tracker_path_safety_circumvent function.");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }

  tf::StampedTransform world_base_transform_for_safety_circumvent = world_base_transform;
  Eigen::Vector2d robot_location_2d(world_base_transform_for_safety_circumvent.getOrigin().x(), world_base_transform_for_safety_circumvent.getOrigin().y());

  double forward_x_for_check_middle_;
  if (vel_x_middle > 0)
  {
    forward_x_for_check_middle_ = abs(forward_x) + x_forward_offset_for_safety_ckeck_;
  }
  else
  {
    forward_x_for_check_middle_ = abs(forward_x) + x_backward_offset_for_safety_ckeck_;
  }

  bool if_obstacles_exist =
      !check_if_pass_the_area(traversability_map,
                              robot_location_2d,
                              forward_x_for_check_middle_,
                              0,
                              world_base_transform_for_safety_circumvent);
}

void PPlanner::visualizeLocalPlanningGraphManager(std::shared_ptr<GraphManager> GraphManager_)
{
  GraphManager_check(GraphManager_);

  std::shared_ptr<Graph> graph_for_visualize = GraphManager_->Graph_;
  std::unordered_map<int, Vertex *> &v_map_visualize = GraphManager_->vertices_map_;

  if (GraphManager_->getNumVertices() == 0)
  {
    return;
  }

  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_localgraphmanager";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker vertex_bound_marker;
  // visualization_msgs::Marker vertex_bound_marker;
  vertex_bound_marker.header.stamp = ros::Time::now();
  vertex_bound_marker.header.seq = 0;
  vertex_bound_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_bound_marker.id = 0;
  vertex_bound_marker.ns = "vertices_bound_localgraphmanager";
  vertex_bound_marker.action = visualization_msgs::Marker::ADD;
  vertex_bound_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_bound_marker.scale.x = pplaner_map_resolution;
  vertex_bound_marker.scale.y = pplaner_map_resolution;
  vertex_bound_marker.scale.z = 0.01;
  vertex_bound_marker.color.r = 0.0 / 255.0;
  vertex_bound_marker.color.g = 0.0 / 255.0;
  vertex_bound_marker.color.b = 255.0 / 255.0;
  vertex_bound_marker.color.a = 1.0;
  vertex_bound_marker.lifetime = ros::Duration(0.0);
  vertex_bound_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    // TODO This id might be id_global in 99 percent certainty by std::cout. Why in other site it might be ind internal;
    int id = graph_for_visualize->getVertexProperty(it);
    int id_global = graph_for_visualize->get_id_global(id);
    id = id_global;
    // std::cout << "graph_for_visualize id is " << id << std::endl;
    //  int id = graph_for_visualize->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[id]->state[0];
    p1.y = v_map_visualize[id]->state[1];
    p1.z = v_map_visualize[id]->state[2];
    if (v_map_visualize[id]->is_bound)
    {
      vertex_bound_marker.points.push_back(p1);
    }
    else
    {
      vertex_marker.points.push_back(p1);
    }
  }

  LocalPlanningGraphManager_pub_.publish(vertex_marker);
  LocalPlanningGraphManager_pub_.publish(vertex_bound_marker);

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = "edges_localGrahManager";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize[src_id]->state[0];
    p1.y = v_map_visualize[src_id]->state[1];
    p1.z = v_map_visualize[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize[tgt_id]->state[0];
    p2.y = v_map_visualize[tgt_id]->state[1];
    p2.z = v_map_visualize[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  LocalPlanningGraphManager_pub_.publish(edge_marker);
}

void PPlanner::GetPlanningGraphFromLocalGraphManager(int source_id_in_local_planning_graph,
                                                     int target_id_in_local_planning_graph,
                                                     std::shared_ptr<GraphManager> source_graph_manager,
                                                     std::shared_ptr<Graph_For_Planning> planning_boost_graph)
{
  LocalPlanningBoundObstacleVector_.clear();
  // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu di 1 guan");

  if (planning_boost_graph != NULL)
  {
    planning_boost_graph->clear();
  }

  const Vertex *vertex_source_ = source_graph_manager->getVertex(source_id_in_local_planning_graph);
  const Vertex *vertex_target_ = source_graph_manager->getVertex(target_id_in_local_planning_graph);

  const std::unordered_map<int, Vertex *> source_vertices_map_ = source_graph_manager->vertices_map_;

  // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu di 2 guan");

  if (source_vertices_map_.size() > 0)
  {
    // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 1 guan");
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
        LocalPlanningBoundObstacleVector_.push_back(obstacle);
      }
      else
      {
      }

      std::vector<Vertex *> nearest_vertices_in_small_box;
      source_graph_manager->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                    limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box in GetPlanningGraphFromLocalGraphManager function.");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
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
    int robot_global_id_ = source_id_in_local_planning_graph;
    int target_global_id_ = target_id_in_local_planning_graph;
    Eigen::Vector3d robot_pos_(vertex_source_->state[0], vertex_source_->state[1], vertex_source_->state[2]);
    Eigen::Vector3d target_pos_(vertex_target_->state[0], vertex_target_->state[1], vertex_target_->state[2]);
    double attractive_gain = Local_Attractive_gain_;
    double repulsive_gain = Local_Repulsive_gain_;
    double bound_radiu_ = Local_Bound_radiu_;
    // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3 guan");

    //  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacle_vector;
    cost_map.clear();
    cost_unorder_map.clear();
    // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 3.5 guan");

    // get_cost_map(planning_boost_graph, &cost_map, &cost_unorder_map,
    //              robot_global_id_, target_global_id_,
    //              robot_pos_, target_pos_,
    //              attractive_gain, repulsive_gain, bound_radiu_,
    //              LocalPlanningBoundObstacleVector_);

    get_cost_map(source_graph_manager, planning_boost_graph, &cost_map, &cost_unorder_map,
                 robot_global_id_, target_global_id_,
                 robot_pos_, target_pos_,
                 attractive_gain, repulsive_gain, bound_radiu_,
                 LocalPlanningBoundObstacleVector_);

    // ROS_INFO("GetPlanningGraphFromLocalGraphManage_neibu if neibu di 4 guan");

    // update boost edge
    for (auto &iter_vertices_map2_ : source_vertices_map_)
    {
      StateVec state_planning_new(iter_vertices_map2_.second->state[0],
                                  iter_vertices_map2_.second->state[1],
                                  iter_vertices_map2_.second->state[2], 0);
      std::vector<Vertex *> nearest_vertices_in_small_box2;
      source_graph_manager->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                    limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2 in GetPlanningGraphFromLocalGraphManager function.");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      else
      {
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
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
    ROS_ERROR("There is something wrong when running GetPlanningGraphFromLocalGraphManager.");
  }
}

void PPlanner::LocalPlannerTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[LocalPlannerTimer_INFO]:START.");
  if (ppath_tracker_->getStatus())
  {
    if ((ppath_tracker_->get_x_of_cmd_vel() > 0) && (!is_turning))
    {
      std::lock_guard<std::mutex> lock(updateMapMutex_);
      std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
      std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
      std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
      std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
      std::lock_guard<std::mutex> lock12(UpdateBadTargetPointVector_Mutex_);
      std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);
      std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);
      // double vel_x_middle = ppath_tracker_->get_x_of_cmd_vel();
      double vel_x_middle = 0.5;
      double forward_x = vel_x_middle * lookahead_time_for_safety_circumvent_;
      try
      {
        world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
        world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                            ros::Time(0), world_base_transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("There is something wrong when trying get robot pose in the update_tracker_path_safety_circumvent function.");
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
      }

      tf::StampedTransform world_base_transform_for_safety_circumvent = world_base_transform;
      Eigen::Vector2d robot_location_2d(world_base_transform_for_safety_circumvent.getOrigin().x(), world_base_transform_for_safety_circumvent.getOrigin().y());

      double forward_x_for_check_middle_;
      if (vel_x_middle > 0)
      {
        forward_x_for_check_middle_ = abs(forward_x) + x_forward_offset_for_safety_ckeck_;
      }
      else
      {
        forward_x_for_check_middle_ = abs(forward_x) + x_backward_offset_for_safety_ckeck_;
      }

      bool if_obstacles_exist =
          !check_if_pass_the_area(traversability_map,
                                  robot_location_2d,
                                  forward_x_for_check_middle_,
                                  0,
                                  world_base_transform_for_safety_circumvent);
      // if_obstacles_exist = true;
      if (if_obstacles_exist)
      {
        ROS_INFO("Find the obstacle.");
        StateVec robot_state_current(world_base_transform_for_safety_circumvent.getOrigin().x(),
                                     world_base_transform_for_safety_circumvent.getOrigin().y(),
                                     world_base_transform_for_safety_circumvent.getOrigin().z() - RobotHeight_,
                                     0);
        // ROS_INFO("DI YI GUAN.");
        ppath_tracker_->pauseTracker();
        // ROS_INFO("DI ER GUAN.");
        int robot_local_id = -1;
        Vertex *robot_nearest_vertices;
        GraphManager graph_manager_temp = *local_graph_for_safety_with_planning_;
        // std::cout << "size of points of graph_manager_temp is " << graph_manager_temp.getNumVertices() << std::endl;
        // ROS_INFO("DI SAN GUAN.");
        if (graph_manager_temp.getNearestVertex(&robot_state_current, &robot_nearest_vertices))
        {
          if (robot_nearest_vertices->is_obstacle)
          {
            ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is vertex in local planner.");
            robot_nearest_vertices->is_obstacle = false;
            robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
          }
          robot_local_id = robot_nearest_vertices->id;
        }
        else
        {
          ROS_ERROR("Can not find the nearest point of robot in local planner graph manager.");
          return;
        }
        // ROS_INFO("DI SI GUAN.");
        //   If ignore the effect of attractive of potential field, constructing the planning boost graph
        //    for local planner only once is fine.
        planning_boost_graph_for_local_planner_->clear();

        // id in local graph is different to in global graph.
        GetPlanningGraphFromLocalGraphManager(robot_local_id,
                                              robot_local_id,
                                              local_graph_for_safety_with_planning_,
                                              planning_boost_graph_for_local_planner_);
        // ROS_INFO("DI SI.5 GUAN.");
        visualizeLocalPlanningBoostGraph(planning_boost_graph_for_local_planner_);

        // ROS_INFO("DI WU GUAN.");

        int CarrotIdFromGlobalPlanner = ppath_tracker_->getCarrotPointIdToOutput();
        Eigen::Vector3d target_current = path_tracked_vector_[CarrotIdFromGlobalPlanner];
        Eigen::Vector3d target_current_global = ppath_tracker_->getCurrentCarrot();
        // ROS_INFO("DI WU.1 GUAN.");
        if (target_current != target_current_global)
        {
          // ROS_ERROR("Why the target_current is not equal to target_current_global?");
          // std::cout << "target_current is " << target_current << std::endl;
          // std::cout << "target_current_global is " << target_current_global << std::endl;
          for (size_t ceshiid = 0; ceshiid < path_tracked_vector_.size(); ceshiid++)
          {
            // std::cout << "ceshiid is " << ceshiid << std::endl;
            // std::cout << "path_tracked_vector_[ceshiid] is " << path_tracked_vector_[ceshiid] << std::endl;
            if (path_tracked_vector_[ceshiid] == target_current_global)
            {
              CarrotIdFromGlobalPlanner = ceshiid;
              // std::cout << "find the CarrotIdFromGlobalPlanner." << std::endl;
            }
          }
        }
        // ROS_INFO("DI WU.2 GUAN.");
        int target_for_local = 0;
        bool find_local_target = false;
        Eigen::Vector3d local_target_pos;

        std::vector<Vertex *> local_path_vertex;

        int i_inside_max = 0;
        // ROS_INFO("DI LIU GUAN.");
        for (size_t i = CarrotIdFromGlobalPlanner; i < path_tracked_vector_.size(); ++i)
        {

          grid_map::Position position_middle(path_tracked_vector_[i].x(), path_tracked_vector_[i].y());
          if (traversability_map.isInside(position_middle))
          {

            i_inside_max = i;

            Eigen::Array2i Index;
            traversability_map.getIndex(position_middle, Index);
            double elevation = traversability_map.at(Elevation_layer_.c_str(), Index);
            double traversability_score = traversability_map.at(Traversability_layer_.c_str(), Index);
            double traversability_supplementary_score =
                traversability_map.at(Traversability_supplementary_layer_.c_str(), Index);

            traversability_score = std::max(traversability_score, traversability_supplementary_score);
            double dis = elevation - path_tracked_vector_[i].z();
            // 0.5 is added to path point .z in viualize
            // std::cout << "dis is " << dis << std::endl;
            // std::cout << "traversability_score is " << dis << std::endl;
            if (abs(dis) > 0.2)
            {
              continue;
            }
            else if (traversability_score < traversability_score_threshold_low_bound)
            {
              continue;
            }
            else
            {
              // std::cout << "hello world!" << std::endl;

              target_for_local = i;
              local_target_pos << position_middle[0], position_middle[1], elevation;
              // find_local_target = true;

              StateVec target_state_current(position_middle[0], position_middle[1], elevation, 0);

              int target_local_id = -1;
              Vertex *target_nearest_vertices;
              if (local_graph_for_safety_with_planning_->getNearestVertex(&target_state_current,
                                                                          &target_nearest_vertices))
              {
                target_local_id = target_nearest_vertices->id;
              }
              else
              {
                ROS_ERROR("Can not find the nearest point of target in local planner graph manager.");
                continue;
              }

              // std::cout << "target_local_id is " << target_local_id << std::endl;
              bool if_the_path_to_target_exist = false;

              if_the_path_to_target_exist = if_find_local_path_(robot_local_id,
                                                                target_local_id,
                                                                planning_boost_graph_for_local_planner_,
                                                                &local_path_vertex,
                                                                local_graph_for_safety_with_planning_);
              if (if_the_path_to_target_exist)
              {
                find_local_target = true;
              }
              else
              {
              }

              // GetPlanningGraphFromLocalGraphManager();
            }
          }
          else
          {
            continue;
          }
        }
        // ROS_INFO("DI QI GUAN.");

        if (find_local_target)
        {
          ROS_INFO("Find the optimum local target and the size of path is %ld", local_path_vertex.size());
          visualizeLocalPath(local_path_vertex);
          Local_ppath_tracker_->clearPathTracked();
          Local_ppath_tracker_->setPathTracked(path_tracked_vector_of_local_);
          tracker_wait_switch = true;
          // while (!Local_ppath_tracker_->getIfTrackSucceed())
          // {
          //   usleep(1000000);
          // }
          // ROS_INFO("Local tracker has finished successfully.");
          // ppath_tracker_->continueTracker();
        }
        else
        {
          ROS_ERROR("Can not find the local target, ");
          ROS_ERROR("and select the local target is usful point in local graph");
          ROS_ERROR("which is nearest to the finally global point in local 4x4.");

          StateVec target_state_current(path_tracked_vector_[i_inside_max].x(),
                                        path_tracked_vector_[i_inside_max].y(),
                                        path_tracked_vector_[i_inside_max].z(), 0);

          int target_local_id = -1;
          std::vector<Vertex *> target_nearest_vertices;
          if (local_graph_for_safety_with_planning_->getNearestVertices(&target_state_current,
                                                                        4,
                                                                        &target_nearest_vertices))
          {
            bool if_the_path_to_target_exist = false;
            for (size_t i = 0; (i < target_nearest_vertices.size()) && (i < 10); ++i)
            {
              target_local_id = target_nearest_vertices[i]->id;

              if_the_path_to_target_exist = if_find_local_path_(robot_local_id,
                                                                target_local_id,
                                                                planning_boost_graph_for_local_planner_,
                                                                &local_path_vertex,
                                                                local_graph_for_safety_with_planning_);
              if (if_the_path_to_target_exist)
              {
                break;
              }
            }

            if (if_the_path_to_target_exist)
            {
              ROS_INFO("Find the suboptimum local target and the size of path is %ld", local_path_vertex.size());
              visualizeLocalPath(local_path_vertex);
              Local_ppath_tracker_->clearPathTracked();
              Local_ppath_tracker_->setPathTracked(path_tracked_vector_of_local_);
              tracker_wait_switch = true;
            }
            else
            {
              ROS_ERROR("There might not be useful points in local graph.");
              // ppath_tracker_->continueTracker();
              ppath_tracker_->clearPathAndSetIfSucceedTrue();
              BadGlobalTargetIdVector_.push_back(global_target_id);
              if_waiting_for_updating_target_ = true;
              if_target_updated_ = false;
              IfNeedUpdateFrontierSubmap_ = true;
              IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
              IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
            }
          }
          else
          {
            ROS_ERROR("Finally, can not find the nearest points of target in local planner graph manager.");
            ppath_tracker_->clearPathAndSetIfSucceedTrue();
            BadGlobalTargetIdVector_.push_back(global_target_id);
            if_waiting_for_updating_target_ = true;
            if_target_updated_ = false;
            IfNeedUpdateFrontierSubmap_ = true;
            IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
            IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
          }
        }
      }
      else
      {
      }
    }
  }
  // ROS_INFO("[LocalPlannerTimer_INFO]:END.");
}

bool PPlanner::if_find_local_path_(int robot_local_id,
                                   int target_local_id,
                                   std::shared_ptr<Graph_For_Planning> planning_boost_graph,
                                   std::vector<Vertex *> *local_path_,
                                   std::shared_ptr<GraphManager> Graph_manager)
{
  local_path_->clear();
  std::vector<Vertex *> path;
  ShortestPathsReportForPlanning report_for_planning;
  if (Find_shortest_path_by_dijkstra(robot_local_id, report_for_planning, planning_boost_graph))
  {
    // ROS_INFO("local report_for_planning.parent_id_map'size is %ld", report_for_planning.parent_id_map.size());
    getShortestPath(target_local_id, report_for_planning, true, path, Graph_manager);
    if (path.size() > 0)
    {
      // ROS_INFO("local path'size is %ld", path.size());
      *local_path_ = path;
      // visualizeLocalPath(path);
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR("Could not find the shortest path by dijkstra mehtod in local.");
    return false;
  }
}

void PPlanner::visualizeLocalPath(std::vector<Vertex *> Path)
{
  if (Path.size() == 0)
  {
    return;
  }
  path_tracked_vector_of_local_.clear();
  // if (Local_Path_pub_.getNumSubscribers() < 1)
  // {
  //   return;
  // }

  visualization_msgs::MarkerArray Path_marker_array;

  visualization_msgs::Marker Path_edge_marker;
  Path_edge_marker.header.stamp = ros::Time::now();
  Path_edge_marker.header.seq = 0;
  Path_edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  Path_edge_marker.id = 0;
  Path_edge_marker.ns = "local_path_edges";
  Path_edge_marker.action = visualization_msgs::Marker::ADD;
  Path_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  Path_edge_marker.scale.x = 0.25;
  Path_edge_marker.color.r = 0.1;
  Path_edge_marker.color.g = 25.0 / 255.0;
  Path_edge_marker.color.b = 226.0 / 255.0;
  Path_edge_marker.color.a = 1.0;
  Path_edge_marker.lifetime = ros::Duration(0.0);
  Path_edge_marker.frame_locked = false;

  visualization_msgs::Marker Path_vertex_marker;
  Path_vertex_marker.header.stamp = ros::Time::now();
  Path_vertex_marker.header.seq = 0;
  Path_vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  Path_vertex_marker.id = 0;
  Path_vertex_marker.ns = "local_path_vertices";
  Path_vertex_marker.action = visualization_msgs::Marker::ADD;
  Path_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  Path_vertex_marker.scale.x = 0.25;
  Path_vertex_marker.scale.y = 0.25;
  Path_vertex_marker.scale.z = 0.25;
  Path_vertex_marker.color.r = 10.0 / 255.0;
  Path_vertex_marker.color.g = 100.0 / 255.0;
  Path_vertex_marker.color.b = 200.0 / 255.0;
  Path_vertex_marker.color.a = 1.0;
  Path_vertex_marker.lifetime = ros::Duration(0.0);
  Path_vertex_marker.frame_locked = false;

  for (size_t i = 0; i < (Path.size() - 1); ++i)
  {
    geometry_msgs::Point p1;
    p1.x = Path[i]->state[0];
    p1.y = Path[i]->state[1];
    p1.z = Path[i]->state[2] + 0.5;
    geometry_msgs::Point p2;
    p2.x = Path[i + 1]->state[0];
    p2.y = Path[i + 1]->state[1];
    p2.z = Path[i + 1]->state[2] + 0.5;
    Path_edge_marker.points.push_back(p1);
    Path_edge_marker.points.push_back(p2);
    Path_vertex_marker.points.push_back(p1);
    Eigen::Vector3d path_point_(Path[i]->state[0],
                                Path[i]->state[1],
                                Path[i]->state[2]);
    path_tracked_vector_of_local_.push_back(path_point_);
    if (i == (Path.size() - 2))
    {
      Path_vertex_marker.points.push_back(p2);

      Eigen::Vector3d path_point2_(Path[i + 1]->state[0],
                                   Path[i + 1]->state[1],
                                   Path[i + 1]->state[2]);
      path_tracked_vector_of_local_.push_back(path_point2_);
    }
    else
    {
    }
  }
  Path_marker_array.markers.push_back(Path_edge_marker);
  Path_marker_array.markers.push_back(Path_vertex_marker);
  Local_Path_pub_.publish(Path_marker_array);
}

void PPlanner::UpdateLocalGraphManagerTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[UpdateLocalGraphManagerTimer_INFO]:START.");
  if (if_map_initialled_)
  {

    std::lock_guard<std::mutex> lock2(traversability_map_Mutex_);
    std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
    ros::Time t_start = ros::Time::now();
    local_graph_for_safety_with_planning_->reset();
    iterator_map_for_graph_manager_(traversability_map, local_graph_for_safety_with_planning_);
    updateFrontierOfGraphManager(local_graph_for_safety_with_planning_);
    updateBoundOfGraphManager(local_graph_for_safety_with_planning_);
    visualizeLocalPlanningGraphManager(local_graph_for_safety_with_planning_);
    ros::Time t_end = ros::Time::now();
    double t_used = t_end.toSec() - t_start.toSec();
    if (t_used > 0.5)
    {
      ROS_INFO("Time used for UpdateLocalGraphManagerTimerCallBack is %f", t_used);
    }
    else
    {
    }
  }
  // ROS_INFO("[UpdateLocalGraphManagerTimer_INFO]:END.");
}

void PPlanner::iterator_map_for_graph_manager_(grid_map::GridMap traversability_map_fun,
                                               std::shared_ptr<GraphManager> Graph_manager)
{
  std::unordered_map<VOXEL_LOC, std::vector<Eigen::VectorXd>, std::hash<VOXEL_LOC>, std::equal_to<VOXEL_LOC>, Eigen::aligned_allocator<std::pair<const VOXEL_LOC, Eigen::VectorXd>>> pplanner_vector_map_for_local_map_;
  pplanner_vector_map_for_local_map_.clear();
  if (local_graph_ != NULL)
  {
    local_graph_->reset();
  }
  ros::Time t_iterator_local_start = ros::Time::now();

  for (grid_map::GridMapIterator iterator(traversability_map_fun); !iterator.isPastEnd(); ++iterator)
  {
    // ros::Time t_iterator_local_once_start = ros::Time::now();
    traversability_map_fun.getPosition(*iterator, position);
    double elevation = traversability_map_fun.at(Elevation_layer_.c_str(), *iterator);
    double traversability_score = traversability_map_fun.at(Traversability_layer_.c_str(), *iterator);
    double traversability_supplementary_score =
        traversability_map_fun.at(Traversability_supplementary_layer_.c_str(), *iterator);

    traversability_score = std::max(traversability_score, traversability_supplementary_score);

    // remove the point whose value is Nan;
    if (elevation != elevation)
    {
      continue;
    }

    if (traversability_score != traversability_score)
    {
      continue;
    }
    // traversability_score = traversability_score - 1.0;
    //  if (traversability_score > traversability_score_threshold_local_graph)
    //  {
    int row;
    int col;
    if (position[0] < -0.5 * pplaner_map_resolution)
    {
      row = (position[0] + pplaner_map_resolution / 2) / pplaner_map_resolution - 1;
    }
    else
    {
      row = (position[0] + pplaner_map_resolution / 2) / pplaner_map_resolution;
    }

    if (position[1] < -0.5 * pplaner_map_resolution)
    {
      col = (position[1] + pplaner_map_resolution / 2) / pplaner_map_resolution - 1;
    }
    {
      col = (position[1] + pplaner_map_resolution / 2) / pplaner_map_resolution;
    }

    StateVec state_new(row * pplaner_map_resolution, col * pplaner_map_resolution, elevation, 0);
    Eigen::VectorXd point_new_(6);
    // X,Y,Z,Traversability,if_is_frontier,if_is_bound
    point_new_ << state_new[0], state_new[1], state_new[2], traversability_score, 0, 0;

    bool vertex_need_to_be_frontier_ = false;
    bool if_point_valid = true;
    bool if_point_bound = false;
    try
    {
      // ros::Time t_if_point_internal_start = ros::Time::now();
      vertex_need_to_be_frontier_ =
          !(if_point_internal(position[0], position[1],
                              traversability_map_fun, &if_point_valid, &if_point_bound));
      // ros::Time t_if_point_internal_end = ros::Time::now();
      // double t_used_if_point_internal = t_if_point_internal_end.toSec() - t_if_point_internal_start.toSec();
      // ROS_INFO("Time used for if_point_internal is %f", t_used_if_point_internal);
      // ROS_ERROR("if_point_valid is %d", if_point_valid);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
      ROS_ERROR("guai weng");
      vertex_need_to_be_frontier_ = true;
    }

    if (!if_point_valid)
    {
      // ROS_ERROR("if_point_valid");
      continue;
    }

    if (if_point_bound)
    {
      // std::cout << "if_point_bound is " << if_point_bound << std::endl;
      point_new_[5] = 1;
    }
    else
    {
      // std::cout << "if_point_bound is " << if_point_bound << std::endl;
      point_new_[5] = 0;
    }

    if (vertex_need_to_be_frontier_)
    {
      point_new_[4] = 1;
    }
    bool frontier_point_valid = false;
    if ((vertex_need_to_be_frontier_) &&
        ((traversability_score > traversability_score_threshold_local_graph_for_frontier) ||
         (traversability_supplementary_score > traversability_score_threshold_local_graph_for_frontier)))
    {
      frontier_point_valid = true;
    }
    // if ((traversability_score > traversability_score_threshold_local_graph) || ((vertex_need_to_be_frontier_) && (traversability_score > traversability_score_threshold_local_graph_for_frontier)))
    if ((traversability_score > traversability_score_threshold_local_graph) || (frontier_point_valid))
    {
      VOXEL_LOC LocalPositionTem_(state_new[0], state_new[1], 0);
      pplanner_vector_map_for_local_map_[LocalPositionTem_].push_back(point_new_);
    }
    else
    {
      continue;
    }
    // ros::Time t_iterator_local_once_end = ros::Time::now();
    // double t_used_locl_once = t_iterator_local_once_end.toSec() - t_iterator_local_once_start.toSec();
    // ROS_INFO("Time used for iterator local map once is %f", t_used_locl_once);
  }
  ros::Time t_iterator_local_end = ros::Time::now();
  double t_used_locl = t_iterator_local_end.toSec() - t_iterator_local_start.toSec();
  if (t_used_locl > 0.5)
  {
    ROS_INFO("Time used for iterator local map is %f", t_used_locl);
  }

  if ((pplanner_vector_map_for_local_map_.size() > 0) && true)
  {
    for (auto &iter_pplanner_map_ : pplanner_vector_map_for_local_map_)
    {
      // int id = iter_pplanner_map_.first;
      double elevation_post_ = 0.0;
      double traversability_scoal_post_ = 0.0;
      bool vertex_is_frontier = false;
      bool vertex_is_bound = false;
      int bound_count = 0;

      bool if_point_need_to_be_removed_ = false;

      int bad_point_count_ = 0;
      for (std::vector<Eigen::VectorXd>::iterator iterator_pplanner_vector_ =
               iter_pplanner_map_.second.begin();
           iterator_pplanner_vector_ != iter_pplanner_map_.second.end(); ++iterator_pplanner_vector_)
      {
        Eigen::VectorXd vector_middle_ = *iterator_pplanner_vector_;
        elevation_post_ += vector_middle_[2];
        traversability_scoal_post_ += vector_middle_[3];
        if (vector_middle_[4] == 1)
        {
          vertex_is_frontier = true;
        }
        if (vector_middle_[5] == 1)
        {
          bound_count++;
          // vertex_is_bound = true;
        }
        if (vector_middle_[3] < traversability_score_threshold_global_graph)
        {
          bad_point_count_++;
        }
        if (vector_middle_[3] < traversability_score_threshold_low_bound)
        {
          // ROS_ERROR("traversability_score_threshold_low_bound is %f", traversability_score_threshold_low_bound);
          if_point_need_to_be_removed_ = true;
          // if_point_need_to_be_deleted_ = true;
        }
      }

      if (bound_count > 0)
      {
        vertex_is_bound = true;
      }

      if ((bad_point_count_ > bad_point_threshold)) // && (!vertex_is_frontier))
      {
        if_point_need_to_be_removed_ = true;
        // continue;
      }

      elevation_post_ = elevation_post_ / iter_pplanner_map_.second.size();
      traversability_scoal_post_ = traversability_scoal_post_ / iter_pplanner_map_.second.size();
      if (traversability_scoal_post_ < traversability_score_threshold_global_graph)
      {
        if_point_need_to_be_removed_ = true;
      }
      if ((iter_pplanner_map_.second.size() < point_threshold) && (!vertex_is_frontier))
      {
        // ROS_ERROR("The Point might need to be removed!");
        if_point_need_to_be_removed_ = true;
      }
      if (iter_pplanner_map_.second.size() < point_threshold_frontier)
      {
        if_point_need_to_be_removed_ = true;
      }

      // if ((if_point_need_to_be_removed_) && (!if_point_need_to_be_deleted_))
      if (if_point_need_to_be_removed_)
      {
        continue;
      }

      StateVec state_to_global_(iter_pplanner_map_.first.x, iter_pplanner_map_.first.y, elevation_post_, 0);

      if (1)
      {
        std::vector<Vertex *> nearest_vertices_in_local_planning_graph_manager_;
        if (vertex_exist_in_Graph_Manager(state_to_global_,
                                          &nearest_vertices_in_local_planning_graph_manager_,
                                          Graph_manager))
        {
          if (nearest_vertices_in_local_planning_graph_manager_.size() > 1)
          {
            ROS_ERROR("THERE IS SOMETHING WRONG WHEN CHECK IF VERTEX EXISTED IN LOCAL PLANNING GRAPH MANAGER.");
            if (nearest_vertices_in_local_planning_graph_manager_.size() < 4)
            {
              continue;
            }
            else
            {
              ROS_ERROR("WHAT THE HELL! WHERE ARE THE POINTS CONMING IN LOCAL PLANNING GRAPH MANAGER?");
              std::cout << "size of nearest_vertices_in_local_planning_graph_manager_ is " << nearest_vertices_in_local_planning_graph_manager_.size() << endl;
              for (size_t hell = 0; hell < nearest_vertices_in_local_planning_graph_manager_.size(); ++hell)
              {
                std::cout << "hell is " << hell << "hell id is" << nearest_vertices_in_local_planning_graph_manager_[hell]->id << "and state is " << std::endl
                          << nearest_vertices_in_local_planning_graph_manager_[hell]->state << std::endl;
              }
              ros::shutdown();
              break;
            }
          }
          else
          {
            if (!nearest_vertices_in_local_planning_graph_manager_[0]->bound_could_not_be_changed)
            {
              nearest_vertices_in_local_planning_graph_manager_[0]->is_bound = vertex_is_bound;
              if (vertex_is_bound == false)
              {
                nearest_vertices_in_local_planning_graph_manager_[0]->bound_could_not_be_changed = true;
              }
            }

            if (!nearest_vertices_in_local_planning_graph_manager_[0]->frontier_could_not_be_changed)
            {
              nearest_vertices_in_local_planning_graph_manager_[0]->is_frontier = vertex_is_frontier;
              if (vertex_is_frontier == false)
              {
                nearest_vertices_in_local_planning_graph_manager_[0]->frontier_could_not_be_changed = true;
              }
            }
            nearest_vertices_in_local_planning_graph_manager_[0]->state = state_to_global_;

            std::vector<Vertex *> nearest_vertices_in_big_box_for_local_planning_graph_manager;
            Graph_manager->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                   limit_box_z_for_edge, &nearest_vertices_in_big_box_for_local_planning_graph_manager);
            if (nearest_vertices_in_big_box_for_local_planning_graph_manager.size() > 9)
            {
              ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box_for_local_planning_graph_manager");
            }
            for (size_t i = 0; (i < nearest_vertices_in_big_box_for_local_planning_graph_manager.size()) && (i < edges_of_one_point_max); ++i)
            {
              if (nearest_vertices_in_big_box_for_local_planning_graph_manager[0]->id == nearest_vertices_in_big_box_for_local_planning_graph_manager[i]->id)
              {
                continue;
              }
              Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box_for_local_planning_graph_manager[i]->state[0],
                                        state_to_global_[1] - nearest_vertices_in_big_box_for_local_planning_graph_manager[i]->state[1],
                                        state_to_global_[2] - nearest_vertices_in_big_box_for_local_planning_graph_manager[i]->state[2]);
              double direction_norm = direction.norm();

              // std::cout << "nearest direction is" << direction_norm << std::endl;
              Graph_manager->removeEdge(nearest_vertices_in_big_box_for_local_planning_graph_manager[0],
                                        nearest_vertices_in_big_box_for_local_planning_graph_manager[i]);
              Graph_manager->addEdge(nearest_vertices_in_big_box_for_local_planning_graph_manager[0],
                                     nearest_vertices_in_big_box_for_local_planning_graph_manager[i], direction_norm);
            }
          }
          continue;
        }
        else
        {

          Vertex *vertex_new_for_local_planning_graph_manager =
              new Vertex(Graph_manager->generateVertexID(), state_to_global_);
          vertex_new_for_local_planning_graph_manager->is_frontier = vertex_is_frontier;
          vertex_new_for_local_planning_graph_manager->is_bound = vertex_is_bound;

          if (vertex_is_frontier == false)
          {
            vertex_new_for_local_planning_graph_manager->frontier_could_not_be_changed = true;
          }
          if (vertex_new_for_local_planning_graph_manager->is_bound == false)
          {
            vertex_new_for_local_planning_graph_manager->bound_could_not_be_changed = true;
          }
          Graph_manager->addVertex(vertex_new_for_local_planning_graph_manager);
          std::vector<Vertex *> nearest_vertices_in_big_box;
          Graph_manager->getNearestVerticesInBox(&state_to_global_, limit_box_x, limit_box_y,
                                                 limit_box_z_for_edge, &nearest_vertices_in_big_box);
          if (nearest_vertices_in_big_box.size() > 9)
          {
            ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_big_box");
          }
          for (size_t i = 0; (i < nearest_vertices_in_big_box.size()) && (i < edges_of_one_point_max); ++i)
          {
            if (vertex_new_for_local_planning_graph_manager->id == nearest_vertices_in_big_box[i]->id)
            {
              continue;
            }
            Eigen::Vector3d direction(state_to_global_[0] - nearest_vertices_in_big_box[i]->state[0],
                                      state_to_global_[1] - nearest_vertices_in_big_box[i]->state[1],
                                      state_to_global_[2] - nearest_vertices_in_big_box[i]->state[2]);
            double direction_norm = direction.norm();

            Graph_manager->removeEdge(vertex_new_for_local_planning_graph_manager,
                                      nearest_vertices_in_big_box[i]);
            Graph_manager->addEdge(vertex_new_for_local_planning_graph_manager,
                                   nearest_vertices_in_big_box[i], direction_norm);
          }
        }
        if_local_planning_graph_updated = true;
      }
      else
      {
      }
    }
  }
  else
  {
    ROS_ERROR("THERE IS SOMETHING WRONG WITH pplanner_vector_map_for_local_map_");
  }
}

void PPlanner::PathTrackerSwitchTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[PathTrackerSwitchTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock12(UpdateBadTargetPointVector_Mutex_);
  std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);

  if (tracker_wait_switch)
  {
    if (Local_ppath_tracker_->getIfTrackSucceed())
    {
      // RAISE A NEW GLOBAL PATH.
      // std::lock_guard<std::mutex> lock(updateMapMutex_);
      // Cheak if the global tartget id exist and is reachable;
      if (global_graph_->vertices_map_.find(global_target_id) == global_graph_->vertices_map_.end())
      {
        ROS_ERROR("The target point is not reachable.");
        tracker_wait_switch = false;
        ppath_tracker_->continueTracker();

        return;
      }
      // getGlobalPathAndTrackItByGlobalId(global_target_id);
      bool IfPlanningAndTrackSucceed;
      getGlobalPathAndTrackItByGlobalId(global_target_id, &IfPlanningAndTrackSucceed);
      // ppath_tracker_->continueTracker();
      if (IfPlanningAndTrackSucceed)
      {
        // tracker_wait_switch = false;
      }
      else
      {
        // ppath_tracker_->continueTracker();
        ppath_tracker_->clearPathAndSetIfSucceedTrue();
        BadGlobalTargetIdVector_.push_back(global_target_id);
        if_waiting_for_updating_target_ = true;
        if_target_updated_ = false;
        IfNeedUpdateFrontierSubmap_ = true;
        IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
        IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
      }
      tracker_wait_switch = false;
    }
  }
  // ROS_INFO("[PathTrackerSwitchTimer_INFO]:END.");
}

void PPlanner::visualizeLocalPlanningBoostGraph(std::shared_ptr<Graph_For_Planning> planning_boost_graph)
{
  // std::cout << "print Local planner boost graph." << std::endl;

  std::shared_ptr<Graph_For_Planning> graph_for_visualize = planning_boost_graph;
  const std::unordered_map<int, Vertex *> &v_map_visualize = local_graph_for_safety_with_planning_->vertices_map_;

  if (planning_boost_graph->getNumVertices() == 0)
  {
    return;
  }

  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_localplanning_boost";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  std::pair<Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator, Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator> vi;
  graph_for_visualize->getVertexIterator(vi);
  for (Graph_For_Planning::GraphType_For_PLANNING::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    // TODO This id might be id_internal in 99 percent certainty by std::cout. Why in other site it might be ind internal;
    int id = graph_for_visualize->get_id_global(graph_for_visualize->getVertexProperty(it));
    // std::cout << "graph_for_visualize id is " << id << std::endl;
    //  int id = graph_for_visualize->getVertexProperty(it);
    geometry_msgs::Point p1;

    p1.x = v_map_visualize.at(id)->state[0];
    p1.y = v_map_visualize.at(id)->state[1];
    p1.z = v_map_visualize.at(id)->state[2];
    vertex_marker.points.push_back(p1);
  }

  Local_Planning_Boost_Graph_pub_.publish(vertex_marker);
  // GraphManager_pub_.publish(vertex_bound_marker);

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = Grid_cell_frame_.c_str();
  edge_marker.id = 0;
  edge_marker.ns = "edges_LocalBoost";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.02;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  std::pair<Graph_For_Planning::GraphType_For_PLANNING::edge_iterator, Graph_For_Planning::GraphType_For_PLANNING::edge_iterator>
      ei;
  graph_for_visualize->getEdgeIterator(ei);
  for (Graph_For_Planning::GraphType_For_PLANNING::edge_iterator it = ei.first; it != ei.second; ++it)
  {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = graph_for_visualize->getEdgeProperty(it);
    int src_id_global = graph_for_visualize->get_id_global(src_id);
    int tgt_id_global = graph_for_visualize->get_id_global(tgt_id);
    geometry_msgs::Point p1;
    p1.x = v_map_visualize.at(src_id_global)->state[0];
    p1.y = v_map_visualize.at(src_id_global)->state[1];
    p1.z = v_map_visualize.at(src_id_global)->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map_visualize.at(tgt_id_global)->state[0];
    p2.y = v_map_visualize.at(tgt_id_global)->state[1];
    p2.z = v_map_visualize.at(tgt_id_global)->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  Local_Planning_Boost_Graph_pub_.publish(edge_marker);
}

void PPlanner::getGlobalPathAndTrackItByGlobalId(int target_global_id)
{
  if (global_graph_->vertices_map_.find(target_global_id) == global_graph_->vertices_map_.end())
  {
    ROS_ERROR("The target point is not reachable.");
    return;
  }
  else
  {
  }
  int id_robot;
  path_tracked_vector_.clear();
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  // std::cout << "robot_state_ is " << robot_state_ << std::endl;

  Vertex *robot_nearest_vertices;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    if (robot_nearest_vertices->is_obstacle)
    {
      ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle in getGlobalPathAndTrackItByGlobalId.");
      robot_nearest_vertices->is_obstacle = false;
      robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
    }
    id_robot = robot_nearest_vertices->id;
    double x_expand_ = 2.0;
    double y_expand_ = 2.0;
    double z_expand_ = 0.3;

    double x_puls_used_ = 0.0;
    double y_puls_used_ = 0.0;
    double z_puls_used_ = 0.0;

    std::vector<Vertex *> path;

    for (int plus_count = 0; plus_count < Max_plus_count; plus_count++)
    {
      double x_expand_used_ = x_expand_ + x_puls_used_;
      double y_expand_used_ = y_expand_ + y_puls_used_;
      double z_expand_used_ = z_expand_ + z_puls_used_;
      // get_planning_graph(id_robot, target_global_id, graph_for_planning_, planning_boost_graph_,
      //                    x_expand_used_, y_expand_used_, z_expand_used_);

      get_planning_graph(id_robot, target_global_id, planning_boost_graph_,
                         x_expand_used_, y_expand_used_, z_expand_used_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);

      // if (graph_for_planning_->vertices_map_.find(target_global_id) == global_graph_->vertices_map_.end())
      if (planning_boost_graph_->vertex_descriptors_.find(target_global_id) == planning_boost_graph_->vertex_descriptors_.end())
      {
        ROS_ERROR("The target point is not reachable in graph_for_planning.");
        return;
      }

      ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: The %d th time of searching the path.", plus_count);

      // ROS_INFO("graph_for_planning_->getNumVertices is %d", graph_for_planning_->getNumVertices());
      // ROS_INFO("graph_for_planning_->getNumEdges is %d", graph_for_planning_->getNumEdges());
      // ROS_INFO("planning_boost_graph_->getNumVertices is %d", planning_boost_graph_->getNumVertices());
      // ROS_INFO("planning_boost_graph_->getNumEdges is %d", planning_boost_graph_->getNumEdges());

      ShortestPathsReportForPlanning report_for_planning;
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        // ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: report_for_planning.parent_id_map'size is %ld", report_for_planning.parent_id_map.size());
        getShortestPath(target_global_id, report_for_planning, true, path);
        if (path.size() > 0)
        {
          // ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: path'size is %ld", path.size());
          visualizePath(path);
          break;
        }
        else
        {
          x_puls_used_ += x_plus_;
          y_puls_used_ += y_plus_;
          z_puls_used_ += z_plus_;
        }
      }
      else
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the shortest path by dijkstra mehtod.");

        return;
      }
    }

    if (path.size() == 0)
    {
      ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: After %d times expand, the algorithm still could not find the path. Now try to find path in global graph.", Max_plus_count);

      // get_planning_graph(id_robot, target_global_id, graph_for_planning_, planning_boost_graph_);
      get_planning_graph(id_robot, target_global_id, planning_boost_graph_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);
      ShortestPathsReportForPlanning report_for_planning;
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        getShortestPath(target_global_id, report_for_planning, true, path);
        if (path.size() > 0)
        {
          visualizePath(path);
        }
        else
        {
          ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Can not plan the path in global graph.");
        }
      }
      else
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the shortest path by dijkstra mehtod.");
        return;
      }
    }

    if (path.size() == 0)
    {
      // ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: After %d times expand, the algorithm still could not find the path", Max_plus_count);

      return;
    }
    else
    {
      if (path_tracked_vector_.size() == 0)
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: There is something wrong with path_tracked_vector_");
      }
      else
      {
        ppath_tracker_->clearPathTracked();
        ppath_tracker_->setPathTracked(path_tracked_vector_);
      }
    }
  }
  else
  {
    ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the point which is nearest to robot state.");
    return;
  }
}

void PPlanner::getGlobalPathAndTrackItByGlobalId(int target_global_id, bool *if_succeed)
{
  if (global_graph_->vertices_map_.find(target_global_id) == global_graph_->vertices_map_.end())
  {
    ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_InFo]: The target point is not reachable.");
    *if_succeed = false;
    return;
  }
  else
  {
  }
  int id_robot;
  path_tracked_vector_.clear();
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  // std::cout << "robot_state_ is " << robot_state_ << std::endl;

  Vertex *robot_nearest_vertices;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    if (robot_nearest_vertices->is_obstacle)
    {
      ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle in getGlobalPathAndTrackItByGlobalId( , ).");
      robot_nearest_vertices->is_obstacle = false;
      robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
    }
    id_robot = robot_nearest_vertices->id;
    double x_expand_ = 2.0;
    double y_expand_ = 2.0;
    double z_expand_ = 0.3;

    double x_puls_used_ = 0.0;
    double y_puls_used_ = 0.0;
    double z_puls_used_ = 0.0;

    std::vector<Vertex *> path_middle_;
    path_middle_.clear();

    for (int plus_count = 0; plus_count < Max_plus_count; plus_count++)
    {
      double x_expand_used_ = x_expand_ + x_puls_used_;
      double y_expand_used_ = y_expand_ + y_puls_used_;
      double z_expand_used_ = z_expand_ + z_puls_used_;
      // get_planning_graph(id_robot, target_global_id, graph_for_planning_, planning_boost_graph_,
      //                    x_expand_used_, y_expand_used_, z_expand_used_);
      get_planning_graph(id_robot, target_global_id, planning_boost_graph_,
                         x_expand_used_, y_expand_used_, z_expand_used_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);

      // if (graph_for_planning_->vertices_map_.find(target_global_id) == global_graph_->vertices_map_.end())
      if (planning_boost_graph_->vertex_descriptors_.find(target_global_id) == planning_boost_graph_->vertex_descriptors_.end())
      {
        ROS_ERROR("The target point is not reachable in graph_for_planning.");
        *if_succeed = false;
        return;
      }

      ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: The %d th time of searching the path.", plus_count);

      // ROS_INFO("graph_for_planning_->getNumVertices is %d", graph_for_planning_->getNumVertices());
      // ROS_INFO("graph_for_planning_->getNumEdges is %d", graph_for_planning_->getNumEdges());
      // ROS_INFO("planning_boost_graph_->getNumVertices is %d", planning_boost_graph_->getNumVertices());
      // ROS_INFO("planning_boost_graph_->getNumEdges is %d", planning_boost_graph_->getNumEdges());

      ShortestPathsReportForPlanning report_for_planning;
      report_for_planning.reset();
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: report_for_planning.parent_id_map'size is %ld", report_for_planning.parent_id_map.size());
        getShortestPath(target_global_id, report_for_planning, true, path_middle_);
        if (path_middle_.size() > 0)
        {
          // ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: path'size is %ld", path.size());
          visualizePath(path_middle_);
          break;
        }
        else
        {
          x_puls_used_ += x_plus_;
          y_puls_used_ += y_plus_;
          z_puls_used_ += z_plus_;
        }
      }
      else
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the shortest path by dijkstra mehtod.");
        *if_succeed = false;
        return;
      }
    }

    if (path_middle_.size() == 0)
    {
      ROS_INFO("[getGlobalPathAndTrackItByGlobalId_INFO]: After %d times expand, the algorithm still could not find the path. Now try to find path in global graph.", Max_plus_count);

      // get_planning_graph(id_robot, target_global_id, graph_for_planning_, planning_boost_graph_);
      get_planning_graph(id_robot, target_global_id, planning_boost_graph_);
      visualizeGraphManager(graph_for_planning_);
      visualizePlanningBoostGraph(planning_boost_graph_);
      ShortestPathsReportForPlanning report_for_planning;
      if (Find_shortest_path_by_dijkstra(id_robot, report_for_planning, planning_boost_graph_))
      {
        getShortestPath(target_global_id, report_for_planning, true, path_middle_);
        if (path_middle_.size() > 0)
        {
          visualizePath(path_middle_);
        }
        else
        {
          ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Can not plan the path in global graph.");
          *if_succeed = false;
          return;
        }
      }
      else
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the shortest path by dijkstra mehtod.");
        *if_succeed = false;
        return;
      }
    }

    if (path_middle_.size() == 0)
    {
      // ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: After %d times expand, the algorithm still could not find the path", Max_plus_count);
      *if_succeed = false;
      return;
    }
    else
    {
      if (path_tracked_vector_.size() == 0)
      {
        ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: There is something wrong with path_tracked_vector_");
      }
      else
      {
        ppath_tracker_->clearPathTracked();
        ppath_tracker_->setPathTracked(path_tracked_vector_);
      }
    }
  }
  else
  {
    ROS_ERROR("[getGlobalPathAndTrackItByGlobalId_INFO]: Could not find the point which is nearest to robot state.");
    *if_succeed = false;
    return;
  }
  *if_succeed = true;
}

void PPlanner::updateBoundOfGraphManager(std::shared_ptr<GraphManager> Graph_manager)
{
  // std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
  if (Graph_manager->getNumVertices() > 1)
  {
    std::shared_ptr<Graph> graph_for_update = Graph_manager->Graph_;
    std::unordered_map<int, Vertex *> &v_map_update = Graph_manager->vertices_map_;
    std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
    graph_for_update->getVertexIterator(vi);
    for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
    {
      int id = graph_for_update->getVertexProperty(it);
      int id_global = graph_for_update->get_id_global(id);
      id = id_global;
      Vertex *vertex_to_update_ = Graph_manager->getVertex(id);
      if (v_map_update[id]->is_frontier)
      {
        continue;
      }
      else
      {
        std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_;
        nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.clear();
        StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
        if (Graph_manager->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   limit_box_z, &nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_))
        {
          if (nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.size() > 8)
          {
            if ((v_map_update[id]->is_bound)) //&& (!v_map_update[id]->bound_could_not_be_changed))
            {
              vertex_to_update_->is_bound = false;
              vertex_to_update_->bound_could_not_be_changed = true;
            }
          }
          else if (nearest_vertices_in_global_graph_for_update_local_planning_graph_manager_.size() < 7)
          {
            vertex_to_update_->is_bound = true;
          }
          else
          {
          }
        }
        else
        {
          // might be wrong
          vertex_to_update_->is_bound = true;
        }
      }
    }
  }
  else
  {
  }
}

void PPlanner::updateFrontierOfGraphManager(std::shared_ptr<GraphManager> Graph_manager)
{
  // std::lock_guard<std::mutex> lock4(Local_Planner_Graph_Manager_Mutex_);
  if (Graph_manager->getNumVertices() > 1)
  {
    std::shared_ptr<Graph> graph_for_update = Graph_manager->Graph_;
    std::unordered_map<int, Vertex *> &v_map_update = Graph_manager->vertices_map_;
    std::pair<Graph::GraphType::vertex_iterator, Graph::GraphType::vertex_iterator> vi;
    graph_for_update->getVertexIterator(vi);
    for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
    {
      int id = graph_for_update->getVertexProperty(it);
      int id_global = graph_for_update->get_id_global(id);
      id = id_global;
      Vertex *vertex_to_update_ = Graph_manager->getVertex(id);
      if ((v_map_update[id]->is_frontier) && (!v_map_update[id]->frontier_could_not_be_changed))
      {
        std::vector<Vertex *> nearest_vertices_in_global_graph_for_update_;
        nearest_vertices_in_global_graph_for_update_.clear();
        StateVec state_(v_map_update[id]->state[0], v_map_update[id]->state[1], v_map_update[id]->state[2], 0);
        if (Graph_manager->getNearestVerticesInBox(&state_, pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   pplaner_map_resolution + 0.25 * pplaner_map_resolution,
                                                   limit_box_z, &nearest_vertices_in_global_graph_for_update_))
        {
          if (nearest_vertices_in_global_graph_for_update_.size() > 8)
          {
            vertex_to_update_->is_frontier = false;
            vertex_to_update_->frontier_could_not_be_changed = true;
          }
          int neighbor_valid_count_ = 0;
          for (size_t i = 0; i < nearest_vertices_in_global_graph_for_update_.size(); ++i)
          {
            if (nearest_vertices_in_global_graph_for_update_[i]->is_frontier == true)
            {
              ++neighbor_valid_count_;
            }
          }
          if (neighbor_valid_count_ < 2)
          {
            vertex_to_update_->is_frontier = false;
          }
        }
        else
        {
          vertex_to_update_->is_frontier = false;
        }
      }
    }
  }
}

bool PPlanner::stdPausePathTrackerCallback(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
  if (global_path_tracker_is_paused_by_user)
  {
    ROS_ERROR("The global path tracker has been paused by user and please do not touch down again and again.");
    res.success = true;
  }
  else
  {
    if (ppath_tracker_->getStatus())
    {
      ppath_tracker_->pauseTracker();
      global_path_tracker_is_paused_by_user = true;
    }
    else
    {
      ROS_INFO("There is no global path tracker to be paused, and the global path tracker might have been paused by local planner.");
    }
  }

  if (local_path_tracker_is_paused_by_user)
  {
    ROS_ERROR("The local path tracker has been paused by user and please do not touch down again and again.");
    res.success = true;
  }
  else
  {
    if (Local_ppath_tracker_->getStatus())
    {
      Local_ppath_tracker_->pauseTracker();
      local_path_tracker_is_paused_by_user = true;
    }
    else
    {
      ROS_INFO("There is no local path tracker to be paused.");
    }
  }

  if (global_path_tracker_is_paused_by_user && local_path_tracker_is_paused_by_user)
  {
    ROS_ERROR("What the hell, the global path tracker and local path tracker could be paused by once put down.");
  }
  else
  {
  }

  res.success = true;
  // res.success &= pci_trigger_response.success;

  return true;
}

bool PPlanner::stdContinuePathTrackerCallback(std_srvs::Trigger::Request &req,
                                              std_srvs::Trigger::Response &res)
{
  // if (global_path_tracker_is_paused_by_user)
  // {
  //   ppath_tracker_->continueTracker();
  //   global_path_tracker_is_paused_by_user = false;
  // }
  // else
  // {
  //   ROS_INFO("There is no global path tracker that has been paused by user.");
  // }

  if (local_path_tracker_is_paused_by_user)
  {
    Local_ppath_tracker_->continueTracker();
    local_path_tracker_is_paused_by_user = false;
  }
  else if (global_path_tracker_is_paused_by_user)
  {
    ppath_tracker_->continueTracker();
    global_path_tracker_is_paused_by_user = false;
    // ROS_INFO("There is no local path tracker that has been paused by user.");
  }
  else
  {
    ROS_INFO("There is no path tracker that has been paused by user.");
  }

  res.success = true;
  return true;
}

bool PPlanner::stdStopPathTrackerCallback(std_srvs::Trigger::Request &req,
                                          std_srvs::Trigger::Response &res)
{
  tracker_wait_switch = false;
  local_path_tracker_is_paused_by_user = false;
  global_path_tracker_is_paused_by_user = false;
  ppath_tracker_->stopTracker();
  Local_ppath_tracker_->stopTracker();
  res.success = true;
  return true;
}

void PPlanner::checkPpathTrackerIfCouldTurn(std::shared_ptr<Ppath_Tracker> Ppath_tracker)
{
  if (is_turning)
  {
    return;
  }
  else
  {
  }
  if (Ppath_tracker->getStatus())
  {
    if (Ppath_tracker->getIfNeedTurn())
    {
      // std::cout << "Ppath_tracker->getIfNeedTurn() is " << Ppath_tracker->getIfNeedTurn() << std::endl;

      bool if_could_turn = if_robot_could_around();
      // std::cout << "if_could_turn is " << if_could_turn << std::endl;
      if (if_could_turn)
      {
        ROS_INFO("Is turning.");
        is_turning = true;
        Ppath_tracker->pauseTracker();

        try
        {
          world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
          world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                              ros::Time(0), world_base_transform);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("There is something wrong when trying get robot pose to get target orientation.");
          ROS_ERROR("%s", ex.what());
          // ros::Duration(1.0).sleep();
        }
        tf::Quaternion q_robot_target = world_base_transform.getRotation();
        double yaw_target_ = tf::getYaw(q_robot_target); // - M_PI;
        double yaw_current_ = tf::getYaw(q_robot_target);
        if (yaw_target_ < 0)
        {
          yaw_target_ = yaw_target_ + M_PI;
        }
        else
        {
          yaw_target_ = yaw_target_ - M_PI;
        }
        // std::cout << "yaw_target_ is " << yaw_target_ << std::endl;
        Eigen::Vector2d Target_Attitude_middle = Ppath_tracker->getTargetAttitude();
        // std::cout << "Target_Attitude_middle is " << Target_Attitude_middle << std::endl;
        //  yaw_target_ = yaw_current_ + Target_Attitude_middle[0] * Target_Attitude_middle[1];
        Eigen::Vector2d Target_Attitude_ = Target_Attitude_middle;
        Target_Attitude_[1] = Target_Attitude_middle[0] * Target_Attitude_[1] + yaw_current_;
        if (Target_Attitude_[1] > M_PI)
        {
          Target_Attitude_[1] = Target_Attitude_[1] - 2 * M_PI;
        }
        else if (Target_Attitude_[1] < -M_PI)
        {
          Target_Attitude_[1] = Target_Attitude_[1] + 2 * M_PI;
        }
        else
        {
        }
        // let_robot_turn_around_z_axe(yaw_target_);
        let_robot_turn_around_z_axe(Target_Attitude_);
        Ppath_tracker->continueTracker();
        usleep(500000);
        is_turning = false;
      }
      else
      {
      }
    }
    else
    {
    }
  }
  else
  {
  }
}

void PPlanner::VisualizeRobotPathPassedTimerCallBack(const ros::TimerEvent &event)
{
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("There is something wrong when trying get robot pose when get path that robot have passed.");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }

  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.header.frame_id = Grid_cell_frame_.c_str();
  this_pose_stamped.header.seq = pos_seq;
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.pose.position.x = world_base_transform.getOrigin().x();
  this_pose_stamped.pose.position.y = world_base_transform.getOrigin().y();
  this_pose_stamped.pose.position.z = world_base_transform.getOrigin().z();
  this_pose_stamped.pose.orientation.w = world_base_transform.getRotation().w();
  this_pose_stamped.pose.orientation.x = world_base_transform.getRotation().x();
  this_pose_stamped.pose.orientation.y = world_base_transform.getRotation().y();
  this_pose_stamped.pose.orientation.z = world_base_transform.getRotation().z();
  Path_Robot_Have_Passed_Msg_.poses.push_back(this_pose_stamped);
  Path_Robot_have_Passed_pub_.publish(Path_Robot_Have_Passed_Msg_);
  pos_seq++;
}

void PPlanner::visualizePointsIssued(std::vector<int> global_ids_issued)
{
  // std::lock_guard<std::mutex> lock(updateMapMutex_);
  if (global_ids_issued.size() == 0)
  {
    return;
  }
  else
  {
  }

  // if (points_issued_pub.getNumSubscribers() < 1)
  // {
  //   return;
  // }
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_issued";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 255.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  for (std::vector<int>::iterator it = global_ids_issued.begin(); it != global_ids_issued.end(); it++)
  {
    int id = *it;
    Vertex *vertex_visualized_ = global_graph_->getVertex(id);
    geometry_msgs::Point p1;
    p1.x = vertex_visualized_->state[0];
    p1.y = vertex_visualized_->state[1];
    p1.z = vertex_visualized_->state[2];
    vertex_marker.points.push_back(p1);
  }

  points_issued_pub.publish(vertex_marker);
}

void PPlanner::visualizePointsIssued(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> global_points_issued)
{
  // std::lock_guard<std::mutex> lock(updateMapMutex_);
  if (global_points_issued.size() == 0)
  {
    return;
  }
  else
  {
  }

  // if (points_issued_pub.getNumSubscribers() < 1)
  // {
  //   return;
  // }
  visualization_msgs::Marker vertex_marker;
  // visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = Grid_cell_frame_.c_str();
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices_issued";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = pplaner_map_resolution;
  vertex_marker.scale.y = pplaner_map_resolution;
  vertex_marker.scale.z = 0.01;
  vertex_marker.color.r = 255.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  for (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>::iterator it = global_points_issued.begin(); it != global_points_issued.end(); it++)
  {
    Eigen::Vector3d point = *it;
    geometry_msgs::Point p1;
    p1.x = point[0];
    p1.y = point[1];
    p1.z = point[2];
    vertex_marker.points.push_back(p1);
  }

  points_issued_pub.publish(vertex_marker);
}

void PPlanner::VisualizePointsIssuedTimerCallBack(const ros::TimerEvent &event)
{

  std::lock_guard<std::mutex> lock(updateMapMutex_);

  // visualizePointsIssued(global_ids_issued_);
  // The id is in local graph and can not be used.

  visualizePointsIssued(global_points_issued_);
}

bool PPlanner::stdStartToExploreCallback(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &res)
{
  std::lock_guard<std::mutex> lock(updateMapMutex_);

  std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);

  std::lock_guard<std::mutex> lock5(Sub_Graph_Of_Frontier_Mutex_);
  std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);

  int target_id_global = 0;
  int frontier_size = 0;

  for (auto &iter_frontier_submap_ : frontier_sub_graphs)
  {
    int frontier_class = iter_frontier_submap_.first;
    int frontier_size_middle = iter_frontier_submap_.second.size();
    if (frontier_size_middle > frontier_size)
    {
      frontier_size = frontier_size_middle;
      target_id_global = centers_of_frontier_sub_graphs_[frontier_class];
    }
    else
    {
    }
  }
  global_target_id = target_id_global;

  // std::lock_guard<std::mutex> lock(updateMapMutex_);
  // getGlobalPathAndTrackItByGlobalId(global_target_id);
  bool IfPlanningAndTrackSucceed;
  getGlobalPathAndTrackItByGlobalId(global_target_id, &IfPlanningAndTrackSucceed);
  if (IfPlanningAndTrackSucceed)
  {
    // if_waiting_for_updating_target_ = true;
    // if_target_updated_ = false;

    // if_exploration = true;
    // res.success = true;
    // return true;
  }
  else
  {
    ROS_ERROR("[stdStartToExploreCallback_INFO]: Can not plan and track successfully.");
  }
  // ppath_tracker_->clearPathAndSetIfSucceedTrue();
  if_waiting_for_updating_target_ = true;
  if_target_updated_ = false;

  if_exploration = true;
  res.success = true;
  return true;
}

bool PPlanner::stdStopExploringCallback(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
  std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);

  // stop the path tracker without lock and might be wrong.
  tracker_wait_switch = false;
  local_path_tracker_is_paused_by_user = false;
  global_path_tracker_is_paused_by_user = false;
  ppath_tracker_->stopTracker();
  Local_ppath_tracker_->stopTracker();
  //

  if_exploration = false;
  res.success = true;
  return true;
}

void PPlanner::ExplorerTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[ExplorerTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);

  std::lock_guard<std::mutex> lock3(planning_boost_graph_Mutex_);

  std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock12(UpdateBadTargetPointVector_Mutex_);
  std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);

  if (is_turning)
  {
    // ROS_INFO("[ExplorerTimer_INFO]:END.");
    return;
  }
  else
  {
  }
  // PLANNER TO NEW TARGET POINT
  // ROS_INFO("[ExplorerTimer_INFO]:DI YI GUAN.");
  if (if_exploration)
  {
    if (ppath_tracker_->getStatus())
    {
      // ROS_INFO("[ExplorerTimer_INFO]:END.");
      return;
    }
    if (Local_ppath_tracker_->getStatus())
    {
      // ROS_INFO("[ExplorerTimer_INFO]:END.");
      return;
    }
    if (if_target_updated_)
    {
      // ROS_INFO("[ExplorerTimer_INFO]:DI ER GUAN.");

      // getGlobalPathAndTrackItByGlobalId(global_target_id);
      bool IfPlanningAndTrackSucceed;
      getGlobalPathAndTrackItByGlobalId(global_target_id, &IfPlanningAndTrackSucceed);
      // ROS_INFO("[ExplorerTimer_INFO]:DI SAN GUAN.");
      if (IfPlanningAndTrackSucceed)
      {
        if_waiting_for_updating_target_ = true;
        if_target_updated_ = false;
        IfNeedUpdateFrontierSubmap_ = true;
        IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
        IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
      }
      else
      {
        ROS_ERROR("[ExplorerTimerCallBack_INFO]: Can not plann and track successfully.");
        // TODO Add id to bad target id vector and pass the id when select target id.
        BadGlobalTargetIdVector_.push_back(global_target_id);
        if_waiting_for_updating_target_ = true;
        if_target_updated_ = false;
        IfNeedUpdateFrontierSubmap_ = true;
        IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
        IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
      }
      // if_waiting_for_updating_target_ = true;
      // if_target_updated_ = false;
    }
    else
    {
      // ROS_INFO("[ExplorerTimer_INFO]:END.");
      return;
    }
  }
  else
  {
  }
  // ROS_INFO("[ExplorerTimer_INFO]:END.");
}

void PPlanner::UpdateTargetTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[UpdateTargetTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);

  std::lock_guard<std::mutex> lock5(Sub_Graph_Of_Frontier_Mutex_);
  std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);
  std::lock_guard<std::mutex> lock7(Update_Explorer_Mutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock9(HandleEmergencyByTurnAround_Mutex_);
  std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock12(UpdateBadTargetPointVector_Mutex_);
  std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);
  std::lock_guard<std::mutex> lock14(TheRoughGlobalGraph_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);

  int target_id_global = 0;
  int frontier_size = 0;
  if (if_exploration)
  {
    if (ppath_tracker_->getIfTrackSucceed())
    {
      if (if_waiting_for_updating_target_)
      {
        if ((!IfNeedUpdateFrontierSubmap_) && (!IfNeedUpdateGlobalGraphForUpdatingTarget_) && (!IfNeedUpdateFrontireGraphForUpdatingTarget_))
        {
          if_waiting_for_updating_target_ = false;
          if_target_updated_ = true;
        }
        else
        {
          // ROS_INFO("[UpdateTargetTimer_INFO]:END.");
          return;
        }
        // if_waiting_for_updating_target_ = false;
        // if_target_updated_ = true;
      }
      else
      {
        // ROS_INFO("[UpdateTargetTimer_INFO]:END.");
        return;
      }
      ros::Time t_UpdateTarget_start = ros::Time::now();
      std::vector<int> ids_target_global;
      ids_target_global.clear();
      if (frontier_sub_graphs.size() < 1)
      {
        ROS_ERROR("The size of frontier_sub_graphs is 0 and the explorer might need to be restarted.");
        // ROS_INFO("[UpdateTargetTimer_INFO]:END.");
        return;
      }
      std::unordered_map<int, ExplorationGainVec> ids_and_attribute; // Id--Size, Distance, Obstacles, DistanceInZ.
      ids_and_attribute.clear();
      ExplorationGainVec Identity;
      // Identity.resize(5);
      Identity << 1, 1, 1, 1, 1;

      for (auto &iter_frontier_submap_ : frontier_sub_graphs)
      {
        int frontier_class = iter_frontier_submap_.first;
        int frontier_size_middle = iter_frontier_submap_.second.size();
        ids_target_global.push_back(centers_of_frontier_sub_graphs_[frontier_class]);

        ids_and_attribute[centers_of_frontier_sub_graphs_[frontier_class]] = Identity;
        ids_and_attribute[centers_of_frontier_sub_graphs_[frontier_class]][0] = frontier_size_middle;

        if (frontier_size_middle > frontier_size)
        {
          frontier_size = frontier_size_middle;
          target_id_global = centers_of_frontier_sub_graphs_[frontier_class];
        }
        else
        {
        }
      }

      // global_target_id = target_id_global;
      // ROS_INFO("Update the global_target_id, the global target id is set to %d.", global_target_id);
      // ROS_INFO("Size of the frontier sub graph is %d.", frontier_size);

      // std::lock_guard<std::mutex> lock(updateMapMutex_);
      // getGlobalPathAndTrackItByGlobalId(global_target_id);

      // std::lock_guard<std::mutex> lock(updateMapMutex_);
      int id_robot_global;
      try
      {
        world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
        world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                            ros::Time(0), world_base_transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
      }
      StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
      // std::cout << "robot_state_ is " << robot_state_ << std::endl;

      Vertex *robot_nearest_vertices;
      if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertices))
      {
        if (robot_nearest_vertices->is_obstacle)
        {
          ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle when update target.");
          robot_nearest_vertices->is_obstacle = false;
          robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
        }
        id_robot_global = robot_nearest_vertices->id;
      }
      else
      {
        ROS_ERROR("Could not find the global id of robot.");
      }
      // planning_boost_graph_for_exploration_->clear();
      // ROS_INFO("HELLO");
      // GraphManager GraphManager_middle = Explorer_->getGraphManager();
      // GraphManager *GraphManager_ptr_middle;
      // GraphManager_ptr_middle = &GraphManager_middle;
      // GetPlanningGraphForExploration(id_robot_global, target_id_global, global_graph_, planning_boost_graph_for_exploration_);
      // GetPlanningGraphForExploration(id_robot_global, target_id_global, GraphManager_ptr_middle, planning_boost_graph_for_exploration_);
      //    lock.~lock_guard();

      // Explorer_->reset();

      // ROS_INFO("Update target Timer di 1 guan.");
      GraphManager GraphManager_middle = *global_graph_;
      Explorer_->setGraphManager(GraphManager_middle);
      GraphManager WholeGraph_middle_ = *Big_global_graph_with_obstacles_;
      Explorer_->setWholeGraph(WholeGraph_middle_);
      GraphManager RoughGlobalGraph_middle_ = *RoughGlobalGraph_;
      Explorer_->setRoughGlobalGraph(RoughGlobalGraph_middle_);
      // ROS_INFO("Update target Timer di 2 guan.");
      //  lock.~lock_guard();
      //  lock8.~lock_guard();

      if (IfUsePathDistanceInsteadOfEuclideanDistance_)
      {
        // if (!Explorer_->setPlanningBoostGraph(planning_boost_graph_for_exploration_))
        // if (!Explorer_->UpdatePlanningGraphForExploration(id_robot_global, id_robot_global, limit_box_x, limit_box_y, limit_box_z_for_edge, edges_of_one_point_max, Local_Attractive_gain_, Local_Repulsive_gain_, Local_Bound_radiu_))
        // {
        //   ROS_ERROR("Failed to set the planning boost graph to explorer.");
        // }
        // else
        // {
        // }
        // // ROS_INFO("Update target Timer di 3 guan.");
        // if (!Explorer_->setRobotGlobalId(id_robot_global))
        // {
        //   ROS_ERROR("Failed to set the global id of robot to explorer.");
        // }
        // else
        // {
        // }
        // // ROS_INFO("Update target Timer di 4 guan.");
        // if (!Explorer_->updateDistance(ids_target_global))
        // {
        //   ROS_ERROR("Failed to update the distance of target ids.");
        // }
        // else
        // {
        // }
        if (!Explorer_->setRobotGlobalIdAndConvertToRoughGlobalGraphId(id_robot_global))
        {
          ROS_ERROR("Failed to set the global id of robot to explorer.");
        }
        if (!Explorer_->updateDistanceByRoughGlobalGraph(ids_target_global))
        {
          ROS_ERROR("Failed to set the global id of robot to explorer.");
        }
        if (!Explorer_->updateEuclideanDistanceInZAxeSolely(id_robot_global, ids_target_global))
        {
          ROS_ERROR("Failed to update the distance in z axe of target ids.");
        }
        else
        {
        }
      }
      else
      {
        if (!Explorer_->updateEuclideanDistance(id_robot_global, ids_target_global))
        {
          ROS_ERROR("Failed to update the distance of target ids.");
        }
        else
        {
        }
      }
      // ROS_INFO("Update target Timer di 5 guan.");
      std::unordered_map<int, double> id_and_distances;
      id_and_distances.clear();
      id_and_distances = Explorer_->get_distances_map();
      ROS_INFO("The size of id_and_distances is %ld", id_and_distances.size());
      for (auto &iterator_middle : id_and_distances)
      {
        int id_out = iterator_middle.first;
        double distance_out = iterator_middle.second;
        ids_and_attribute[id_out][1] = distance_out;
        // ROS_INFO("The id is %d and the distance of it is %f", id_out, distance_out);
      }
      // ROS_INFO("Update target Timer di 6 guan.");
      // Get distance in z axe;
      std::unordered_map<int, double> id_and_DistancesInZ;
      id_and_DistancesInZ.clear();
      id_and_DistancesInZ = Explorer_->get_DistanceInZ_map();
      ROS_INFO("The size of id_and_DistancesInZ is %ld", id_and_DistancesInZ.size());
      for (auto &iteratorDistance_middle : id_and_DistancesInZ)
      {
        int id_out = iteratorDistance_middle.first;
        double ZDistance_out = iteratorDistance_middle.second;
        ids_and_attribute[id_out][3] = ZDistance_out;
        // ROS_INFO("The id is %d and the distance of it is %f", id_out, distance_out);
      }

      //  Get the id_and_obstacles.
      if (!Explorer_->updateObstaclesAttribute(ids_target_global, 5.1 * pplaner_map_resolution, 5.1 * pplaner_map_resolution, limit_box_z))
      {
        ROS_ERROR("Failed to update the distance of target ids.");
      }
      else
      {
      }
      // ROS_INFO("Update target Timer di 7 guan.");
      std::unordered_map<int, double> id_and_ObstaclesAttribute;
      id_and_ObstaclesAttribute.clear();
      id_and_ObstaclesAttribute = Explorer_->get_ObstaclesAttribute_map();
      ROS_INFO("The size of id_and_ObstaclesAttribute is %ld", id_and_ObstaclesAttribute.size());
      for (auto &iterator_middle2 : id_and_ObstaclesAttribute)
      {
        int id_out = iterator_middle2.first;
        double ObstaclesAttribute_out = iterator_middle2.second;
        ids_and_attribute[id_out][2] = ObstaclesAttribute_out;
        // ROS_INFO("The id is %d and the distance of it is %f", id_out, distance_out);
      }
      // ROS_INFO("Update target Timer di 8 guan.");

      Explorer_->ComputeExplorationGain(&ids_and_attribute, LambdaSize_, LambdaDistance_, LambdaObstacle_, LambdaZDistance_);
      double ExplorationGain_ = 0.0;
      for (auto &iterator_middle3 : ids_and_attribute)
      {
        int Id_out_ = iterator_middle3.first;
        // Judge if the id is bad global target id.
        std::vector<int>::iterator it_find_middle_;
        it_find_middle_ = find(BadGlobalTargetIdVector_.begin(), BadGlobalTargetIdVector_.end(), Id_out_);
        if (it_find_middle_ == BadGlobalTargetIdVector_.end())
        {
        }
        else
        {
          continue;
        }
        ExplorationGainVec Attribute_out_ = iterator_middle3.second;
        ROS_INFO("The id is %d, and the attribute is %f, %f, %f, %f, %f.", Id_out_, Attribute_out_[0], Attribute_out_[1], Attribute_out_[2], Attribute_out_[3], Attribute_out_[4]);
        if (Attribute_out_[4] > ExplorationGain_)
        {
          target_id_global = Id_out_;
          ExplorationGain_ = Attribute_out_[4];
        }
      }

      if (global_target_id == target_id_global)
      {

        TimesOfGlobalTargetIdRepetition_++;
        // TODO The robot might need turn around. Now treat it by info operator the problem.
        if (TimesOfGlobalTargetIdRepetition_ > TimesOfGlobalTargetIdRepetitionThreshold_)
        {
          if_waiting_for_updating_target_ = false;
          if_target_updated_ = false;
          IfNeedHandleEmergencyByTurnAround_ = true;
          TimesOfGlobalTargetIdRepetition_ = 0;
          BadGlobalTargetIdVector_.push_back(target_id_global);
        }
        else
        {
          if_waiting_for_updating_target_ = true;
          if_target_updated_ = false;
          IfNeedHandleEmergencyByTurnAround_ = false;
        }
        // if_waiting_for_updating_target_ = false;
        // if_target_updated_ = false;
        // IfNeedHandleEmergencyByTurnAround_ = true;
      }
      else
      {
        TimesOfGlobalTargetIdRepetition_ = 0;
        global_target_id = target_id_global;
      }

      // global_target_id = target_id_global;
      ROS_INFO("Update the global_target_id, the global target id is set to %d.", global_target_id);
      ROS_INFO("Size of the frontier sub graph is %d.", frontier_size);

      ros::Time t_UpdateTarget_end = ros::Time::now();
      double t_used_update_target_once_ = t_UpdateTarget_end.toSec() - t_UpdateTarget_start.toSec();
      ROS_INFO("Time used for updating target once is %f", t_used_update_target_once_);
    }
    else
    {
    }
  }
  else
  {
    // ROS_INFO("[UpdateTargetTimer_INFO]:END.");
    return;
  }
  // ROS_INFO("[UpdateTargetTimer_INFO]:END.");
}

bool PPlanner::stdInitializationCallback(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &res)
{
  // std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);
  // if_exploration = false;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> path_foward_;
  path_foward_.clear();

  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  Eigen::Vector3d robot_current_state(world_base_transform.getOrigin().x(),
                                      world_base_transform.getOrigin().y(),
                                      world_base_transform.getOrigin().z());
  tf::Quaternion q_robot_current = world_base_transform.getRotation();
  double robot_yaw_current = tf::getYaw(q_robot_current);
  double x_forward = 2.0;
  double x_add_in_world_frame = x_forward * cos(robot_yaw_current);
  double y_add_in_world_frame = x_forward * sin(robot_yaw_current);
  Eigen::Vector3d target_state(robot_current_state[0] + x_add_in_world_frame,
                               robot_current_state[1] + y_add_in_world_frame,
                               robot_current_state[2]);
  path_foward_.push_back(robot_current_state);
  path_foward_.push_back(target_state);
  path_tracked_vector_ = path_foward_;
  // ppath_tracker_->setPathTracked(path_foward_);
  //  Ppath_Tracker PathTracker_middle(nh_, nh_private_);//It would be deleted when the function end.
  //  PathTracker_middle.setPathTracked(path_foward_);
  Initial_Path_tracker_->setPathTracked(path_foward_);

  res.success = true;
  return true;
}

void PPlanner::GetPlanningGraphForExploration(int source_id_global, int target_id_global,
                                              GraphManager *source_graph_manager,
                                              std::shared_ptr<Graph_For_Planning> planning_boost_graph)
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
      source_graph_manager->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                    limit_box_z_for_edge, &nearest_vertices_in_small_box);
      if (nearest_vertices_in_small_box.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box in GetPlanningGraphForExploration function.");
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box[ceshi_id]->state << std::endl;
        }
      }
      for (size_t j = 0; (j < nearest_vertices_in_small_box.size()) && (j < edges_of_one_point_max); ++j)
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
    double attractive_gain = Local_Attractive_gain_;
    double repulsive_gain = Local_Repulsive_gain_;
    double bound_radiu_ = Local_Bound_radiu_;
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
      source_graph_manager->getNearestVerticesInBox(&state_planning_new, limit_box_x, limit_box_y,
                                                    limit_box_z_for_edge, &nearest_vertices_in_small_box2);
      if (nearest_vertices_in_small_box2.size() > 9)
      {
        ROS_ERROR("THERE IS SOMETHING WRONG WITH nearest_vertices_in_small_box2 in GetPlanningGraphForExploration function.");
        ROS_INFO("The size of nearest_vertices_in_small_box2 is %ld", nearest_vertices_in_small_box2.size());
        for (size_t ceshi_id = 0; ceshi_id < nearest_vertices_in_small_box2.size(); ++ceshi_id)
        {
          std::cout << "ceshi_id is " << ceshi_id << " and the state is " << std::endl;
          std::cout << nearest_vertices_in_small_box2[ceshi_id]->state << std::endl;
        }
      }
      else
      {
      }
      for (size_t jj = 0; (jj < nearest_vertices_in_small_box2.size()) && (jj < edges_of_one_point_max); ++jj)
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
    ROS_ERROR("There is something wrong when running GetPlanningGraphForExploration.");
  }
}

void PPlanner::UpdateExplorerGraphManagerTimerCallBack(const ros::TimerEvent &event)
{
  // ROS_INFO("[UpdateExplorerGraphManagerTimer_INFO]:START.");
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock7(Update_Explorer_Mutex_);
  std::lock_guard<std::mutex> lock8(BigWholeGrpah_Mutex_);
  std::lock_guard<std::mutex> lock14(TheRoughGlobalGraph_Mutex_);

  ros::Time t_UpdateExplorer_start = ros::Time::now();
  GraphManager GraphManager_middle = *global_graph_;
  Explorer_->setGraphManager(GraphManager_middle);
  GraphManager WholeGraph_middle_ = *Big_global_graph_with_obstacles_;
  Explorer_->setWholeGraph(WholeGraph_middle_);
  GraphManager RoughGlobalGraph_middle_ = *RoughGlobalGraph_;
  Explorer_->setRoughGlobalGraph(RoughGlobalGraph_middle_);
  ros::Time t_UpdateExplorer_end = ros::Time::now();
  double t_used_update_explorer_once_ = t_UpdateExplorer_end.toSec() - t_UpdateExplorer_start.toSec();
  if (t_used_update_explorer_once_ > 0.1)
  {
    ROS_INFO("Time used for updating Explorer once is %f", t_used_update_explorer_once_);
  }
  else
  {
  }
  // ROS_INFO("[UpdateExplorerGraphManagerTimer_INFO]:END.");
}

bool PPlanner::stdUpdateTargetPointCallback(std_srvs::Trigger::Request &req,
                                            std_srvs::Trigger::Response &res)
{
  std::lock_guard<std::mutex> lock(updateMapMutex_);
  std::lock_guard<std::mutex> lock5(Sub_Graph_Of_Frontier_Mutex_);
  std::lock_guard<std::mutex> lock6(If_exploration_Mutex_);
  std::lock_guard<std::mutex> lock7(Update_Explorer_Mutex_);
  UpdateTargetPoint();
  res.success = true;
  return true;
}

void PPlanner::UpdateTargetPoint()
{
  int target_id_global = 0;
  int frontier_size = 0;
  ros::Time t_UpdateTarget_start = ros::Time::now();
  std::vector<int> ids_target_global;
  ids_target_global.clear();
  for (auto &iter_frontier_submap_ : frontier_sub_graphs)
  {
    int frontier_class = iter_frontier_submap_.first;
    int frontier_size_middle = iter_frontier_submap_.second.size();
    ids_target_global.push_back(centers_of_frontier_sub_graphs_[frontier_class]);
    if (frontier_size_middle > frontier_size)
    {
      frontier_size = frontier_size_middle;
      target_id_global = centers_of_frontier_sub_graphs_[frontier_class];
    }
    else
    {
    }
  }

  int id_robot_global;
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  StateVec robot_state_(world_base_transform.getOrigin().x(), world_base_transform.getOrigin().y(), world_base_transform.getOrigin().z() - RobotHeight_, 0);
  // std::cout << "robot_state_ is " << robot_state_ << std::endl;

  Vertex *robot_nearest_vertices;
  if (global_graph_->getNearestVertex(&robot_state_, &robot_nearest_vertices))
  {
    if (robot_nearest_vertices->is_obstacle)
    {
      ROS_ERROR("[PPLANNER_INFO]: The robot vertex finded is obstacle when update target point.");
      robot_nearest_vertices->is_obstacle = false;
      robot_nearest_vertices->membership_degree << 0, 0, 0, 1;
    }
    id_robot_global = robot_nearest_vertices->id;
  }
  else
  {
    ROS_ERROR("Could not find the global id of robot.");
  }

  if (!Explorer_->UpdatePlanningGraphForExploration(id_robot_global, id_robot_global, limit_box_x, limit_box_y, limit_box_z_for_edge, edges_of_one_point_max, Local_Attractive_gain_, Local_Repulsive_gain_, Local_Bound_radiu_))
  {
    ROS_ERROR("Failed to set the planning boost graph to explorer.");
  }
  else
  {
  }
  if (!Explorer_->setRobotGlobalId(id_robot_global))
  {
    ROS_ERROR("Failed to set the global id of robot to explorer.");
  }
  else
  {
  }
  if (!Explorer_->updateDistance(ids_target_global))
  {
    ROS_ERROR("Failed to update the distance of target ids.");
  }
  else
  {
  }
  std::unordered_map<int, double> id_and_distances;
  id_and_distances.clear();
  id_and_distances = Explorer_->get_distances_map();
  ROS_INFO("The size of id_and_distances is %ld", id_and_distances.size());
  for (auto &iterator_middle : id_and_distances)
  {
    int id_out = iterator_middle.first;
    double distance_out = iterator_middle.second;
    ROS_INFO("The id is %d and the distance of it is %f", id_out, distance_out);
  }
  global_target_id = target_id_global;
  ROS_INFO("Update the global_target_id, the global target id is set to %d.", global_target_id);
  ROS_INFO("Size of the frontier sub graph is %d.", frontier_size);

  ros::Time t_UpdateTarget_end = ros::Time::now();
  double t_used_update_target_once_ = t_UpdateTarget_end.toSec() - t_UpdateTarget_start.toSec();
  ROS_INFO("Time used for updating target once is %f", t_used_update_target_once_);
}

void PPlanner::HandleEmergencyByTurnAroundTimerCallBack(const ros::TimerEvent &event)
{
  std::lock_guard<std::mutex> lock9(HandleEmergencyByTurnAround_Mutex_);
  std::lock_guard<std::mutex> lock10(UpdateFrontierSubmapForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock11(UpdateGlobalGraphForExplorer_Mutex_);
  std::lock_guard<std::mutex> lock13(TheStateOfIfWaitingAndUpdatedTarget_Mutex_);
  std::lock_guard<std::mutex> lock15(UpdateFrontierGraphForExplorer_Mutex_);
  if (IfNeedHandleEmergencyByTurnAround_)
  {
    HandleEmergencyByTurnAround();
    // HandleEmergencyByTurnAround();
    IfNeedHandleEmergencyByTurnAround_ = false;
    if_waiting_for_updating_target_ = true;
    if_target_updated_ = false;
    IfNeedUpdateFrontierSubmap_ = true;
    IfNeedUpdateGlobalGraphForUpdatingTarget_ = true;
    IfNeedUpdateFrontireGraphForUpdatingTarget_ = true;
  }
}

void PPlanner::HandleEmergencyByTurnAround()
{
  try
  {
    world_base_listener.waitForTransform(World_frame_.c_str(), Track_frame_.c_str(), ros::Time(0), ros::Duration(1.0));
    world_base_listener.lookupTransform(World_frame_.c_str(), Track_frame_.c_str(),
                                        ros::Time(0), world_base_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[HandleEmergencyByTurnAround_INFO]:There is something wrong when trying get robot pose to get target orientation.");
    ROS_ERROR("%s", ex.what());
    // ros::Duration(1.0).sleep();
  }
  tf::Quaternion q_robot_target = world_base_transform.getRotation();
  double yaw_current_ = tf::getYaw(q_robot_target);
  Eigen::Vector2d Target_Attitude_middle(1, M_PI);
  Eigen::Vector2d Target_Attitude_ = Target_Attitude_middle;
  Target_Attitude_[1] = Target_Attitude_middle[0] * Target_Attitude_[1] + yaw_current_;
  if (Target_Attitude_[1] > M_PI)
  {
    Target_Attitude_[1] = Target_Attitude_[1] - 2 * M_PI;
  }
  else if (Target_Attitude_[1] < -M_PI)
  {
    Target_Attitude_[1] = Target_Attitude_[1] + 2 * M_PI;
  }
  else
  {
  }
  let_robot_turn_around_z_axe(Target_Attitude_);
}