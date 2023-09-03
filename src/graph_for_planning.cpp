#include "pplanner/graph_for_planning.h"
// #include "planner_common/graph_base.h"

Graph_For_Planning::Graph_For_Planning()
{
  id_internal = 0;
  vertex_internal_id_.clear();
  vertex_global_id_.clear();
}
Graph_For_Planning::~Graph_For_Planning() {}

Graph_For_Planning::VertexDescriptor Graph_For_Planning::addSourceVertex(int id)
{
  source_ = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = source_;

  vertex_internal_id_[id_internal] = id;
  vertex_global_id_[id] = id_internal;
  id_internal++;

  return source_;
}

Graph_For_Planning::VertexDescriptor Graph_For_Planning::addVertex(int id)
{
  VertexDescriptor v = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = v;

  vertex_internal_id_[id_internal] = id;
  vertex_global_id_[id] = id_internal;
  id_internal++;

  return v;
}

void Graph_For_Planning::removeVertex(int id)
{
  boost::clear_vertex(vertex_descriptors_[id], graph_);
  boost::remove_vertex(vertex_descriptors_[id], graph_);
  // VertexDescriptor v = boost::add_vertex(id, graph_);
  // vertex_descriptors_[id] = NULL;

  // boost::remove_vertex(vertex_descriptors_[id], graph_);

  vertex_descriptors_.erase(id);
  // return v;
}

Graph_For_Planning::EdgeDescriptorPair Graph_For_Planning::addEdge(int u_id, int v_id, double weight)
{
  if (vertex_descriptors_.find(u_id) == vertex_descriptors_.end())
  {
    addVertex(u_id);
  }
  if (vertex_descriptors_.find(v_id) == vertex_descriptors_.end())
  {
    addVertex(v_id);
  }
  return boost::add_edge(vertex_descriptors_[u_id], vertex_descriptors_[v_id],
                         weight, graph_);
}

void Graph_For_Planning::removeEdge(int u_id, int v_id)
{
  if (vertex_descriptors_.find(u_id) == vertex_descriptors_.end())
  {
    return;
  }
  if (vertex_descriptors_.find(v_id) == vertex_descriptors_.end())
  {
    return;
  }

  boost::remove_edge(vertex_descriptors_[u_id], vertex_descriptors_[v_id],
                     graph_);
}

bool Graph_For_Planning::findDijkstraShortestPaths(int src_id, ShortestPathsReportForPlanning &rep)
{
  rep.source_id = src_id;
  rep.status = false;

  // if (vertex_descriptors_.size() <= src_id)
  // {
  //   std::cout << "Source index [" << src_id << "] is not in the graph."
  //             << std::endl;
  //   return false;
  // }

  std::vector<VertexDescriptor> shortest_paths;
  std::vector<double> shortest_distances;
  if (findDijkstraShortestPaths(vertex_descriptors_[src_id], shortest_paths,
                                shortest_distances))
  {
    rep.status = true;
    rep.parent_id_map.clear();
    rep.distance_map.clear();
    int num_vertices = getNumVertices();
    for (int ind = 0; ind < num_vertices; ++ind)
    {
      int ind_global = get_id_global(ind);

      rep.parent_id_map[ind_global] = get_id_global(getVertexID(shortest_paths[ind]));

      if (ind_global == rep.parent_id_map[ind_global])
      {
        // std::cout << "num_vertices is " << num_vertices << std::endl;
        // std::cout << "point int_global is " << ind_global << " and it's a bad point!" << std::endl;
      }

      // rep.parent_id_map[ind_global] = getVertexID(shortest_paths[ind]);
      // std::cout << "rep.parent_id_map " << ind_global << " is " << rep.parent_id_map[ind_global] << std::endl;
      rep.distance_map[ind_global] = shortest_distances[ind];

      // rep.parent_id_map[ind] = getVertexID(shortest_paths[ind]);
      // std::cout << "rep.parent_id_map " << ind << " is " << rep.parent_id_map[ind] << std::endl;
      // rep.distance_map[ind] = shortest_distances[ind];
    }
  }
  if (rep.parent_id_map.size() < 2)
  {
    return false;
  }

  return rep.status;
}

bool Graph_For_Planning::findDijkstraShortestPaths(
    VertexDescriptor &source, std::vector<VertexDescriptor> &shortest_paths,
    std::vector<double> &shortest_distances)
{
  num_vertices_ = boost::num_vertices(graph_);
  if (num_vertices_ < 2)
    return false;

  shortest_paths.clear();
  shortest_distances.clear();
  shortest_paths.resize(num_vertices_);
  shortest_distances.resize(num_vertices_);

  auto v_index = boost::get(boost::vertex_index, graph_);
  auto weight = boost::get(boost::edge_weight, graph_);
  boost::dijkstra_shortest_paths(
      graph_, source,
      boost::predecessor_map(
          boost::make_iterator_property_map(shortest_paths.begin(),
                                            get(boost::vertex_index, graph_)))
          .distance_map(boost::make_iterator_property_map(
              shortest_distances.begin(), get(boost::vertex_index, graph_))));
  return true;
}

int Graph_For_Planning::getVertexID(VertexDescriptor v)
{
  return boost::get(boost::get(boost::vertex_index, graph_), v);
}

int Graph_For_Planning::getNumVertices() { return boost::num_vertices(graph_); }

int Graph_For_Planning::getNumEdges() { return boost::num_edges(graph_); }

void Graph_For_Planning::printResults(const ShortestPathsReportForPlanning &rep)
{
  std::cout << "Shortest paths:" << std::endl;
  for (int i = 0; i < num_vertices_; ++i)
  {
    int id = rep.parent_id_map.at(i);
    std::cout << "Path: "
              << "[cost: " << rep.distance_map.at(i) << "] " << i << "<-" << id;
    if (id == 0)
    {
    }
    else if (id == i)
      std::cout << "[isolated-vertex]";
    else
    {
      int id_parent = id;
      while (id_parent != 0)
      {
        id_parent = rep.parent_id_map.at(id_parent);
        std::cout << "<-" << id_parent;
      }
    }
    std::cout << std::endl;
  }
}

void Graph_For_Planning::getEdgeIterator(
    std::pair<GraphType_For_PLANNING::edge_iterator, GraphType_For_PLANNING::edge_iterator> &ei)
{
  ei = boost::edges(graph_);
}

std::tuple<int, int, double> Graph_For_Planning::getEdgeProperty(
    GraphType_For_PLANNING::edge_iterator e)
{
  return std::make_tuple(
      getVertexID(boost::source(*e, graph_)),
      getVertexID(boost::target(*e, graph_)),
      boost::get(boost::get(boost::edge_weight, graph_), *e));
}

void Graph_For_Planning::getVertexIterator(
    std::pair<GraphType_For_PLANNING::vertex_iterator, GraphType_For_PLANNING::vertex_iterator> &vi)
{
  vi = boost::vertices(graph_);
}

int Graph_For_Planning::getVertexProperty(GraphType_For_PLANNING::vertex_iterator v)
{
  return boost::get(boost::get(boost::vertex_index, graph_), *v);
}

void Graph_For_Planning::clear()
{
  graph_.clear();
  vertex_descriptors_.clear();
  vertices_.clear();
  id_internal = 0;
  vertex_internal_id_.clear();
  vertex_global_id_.clear();
}

void Graph_For_Planning::getcomponent_and_num(std::vector<int> *component, int *num_components)
{
  std::vector<int> component2(num_vertices(graph_));
  int num_components2 = boost::connected_components(graph_, &component2[0]);
  // std::cout << "num_components2 is " << num_components2 << std::endl;

  // std::vector<int>::size_type i;
  // for (i = 0; i != component2.size(); ++i)
  // {
  //   std::cout << "vertex " << i << ": ";
  //   std::cout << component2[i];
  //   std::cout << std::endl;
  // }

  *component = component2;
  *num_components = num_components2;
}

int Graph_For_Planning::get_id_global(int IdInternal)
{
  return vertex_internal_id_[IdInternal];
}

int Graph_For_Planning::get_id_internal(int IdGlobal)
{
  return vertex_global_id_[IdGlobal];
}

void Graph_For_Planning::get_total_force(const Eigen::Vector3d robot_pos_,
                                         const Eigen::Vector3d target_pos_,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bound_obstacles_,
                                         Eigen::Vector3d *force_,
                                         double attractive_gain,
                                         double repulsive_gain,
                                         double bound_radiu_)
{
  Eigen::Vector3d force_internal;
  force_internal << 0, 0, 0;
  // fx = fy = 0;
  //  Calculate attractive force towards the goal
  // double gx = 5, gy = 5; // Goal position
  double dx_att = target_pos_[0] - robot_pos_[0];
  double dy_att = target_pos_[1] - robot_pos_[1];
  double dz_att = target_pos_[2] - robot_pos_[2];

  double d = std::sqrt(dx_att * dx_att + dy_att * dy_att + dz_att * dz_att);
  if (d == 0)
  {
    force_internal[0] += 0;
    force_internal[1] += 0;
    force_internal[2] += 0;
  }
  else
  {
    force_internal[0] += attractive_gain * dx_att / d;
    force_internal[1] += attractive_gain * dy_att / d;
    force_internal[2] += attractive_gain * dz_att / d;
  }

  // Calculate repulsive forces from obstacles
  for (auto bound = bound_obstacles_.begin(); bound != bound_obstacles_.end(); ++bound)
  {
    Eigen::Vector3d obstacle_middle_ = *bound;
    double ox = obstacle_middle_[0];
    double oy = obstacle_middle_[1];
    double oz = obstacle_middle_[2];
    double radiu = bound_radiu_;

    double dx = robot_pos_[0] - ox;
    double dy = robot_pos_[1] - oy;
    double dz = robot_pos_[2] - oz;
    double d_re = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d_re < radiu)
    {
      if (d_re == 0)
      {
        force_internal[0] += 100;
        force_internal[1] += 100;
        force_internal[2] += 100;
      }
      else
      {
        double f = repulsive_gain * (1 / d_re - 1 / radiu) / (d_re * d_re);
        force_internal[0] += f * dx / d_re;
        force_internal[1] += f * dy / d_re;
        force_internal[2] += f * dz / d_re;
      }
    }
  }
  *force_ = force_internal;
}
void Graph_For_Planning::get_cost_map(const GraphType_For_PLANNING graph_,
                                      std::vector<double> *cost_map,
                                      int robot_global_id_,
                                      int target_global_id_,
                                      const Eigen::Vector3d robot_pos_,
                                      const Eigen::Vector3d target_pos_,
                                      double attractive_gain,
                                      double repulsive_gain,
                                      double bound_radiu_)
{
  cost_map->resize(num_vertices(graph_));
  int robot_internal_id_ = get_id_internal(robot_global_id_);
  int target_internal_id_ = get_id_internal(target_global_id_);
  // Calculate the shortest path distance from each vertex to the goal
  std::vector<int> parent(num_vertices(graph_));
  std::vector<double> d(num_vertices(graph_));
  boost::dijkstra_shortest_paths(graph_, vertex_descriptors_[target_global_id_], boost::predecessor_map(&parent[0]).distance_map(&d[0]));
  // Calculate the cost map based on the potential field
  // for (int i = 0; i < num_vertices(graph_); i++)
  // {
  //   const Vertex &v = g[i];
  //   double fx, fy;
  //   total_force(v, g, fx, fy, attractive_gain, repulsive_gain);
  //   double force = std::sqrt(fx * fx + fy * fy);
  //   double distance = d[i];
  //   cost_map[i] = force * distance;
  // }
}

void Graph_For_Planning::dijkstra_process(int target_global_id_, std::vector<int> *parent_, std::vector<double> *d_)
{
  std::vector<int> parent(num_vertices(graph_));
  std::vector<double> d(num_vertices(graph_));
  boost::dijkstra_shortest_paths(graph_, vertex_descriptors_[target_global_id_], boost::predecessor_map(&parent[0]).distance_map(&d[0]));

  // std::vector<VertexDescriptor> shortest_paths;
  // std::vector<double> shortest_distances;
  // shortest_paths.clear();
  // shortest_distances.clear();
  // shortest_paths.resize(num_vertices(graph_));
  // shortest_distances.resize(num_vertices(graph_));

  // boost::dijkstra_shortest_paths(
  //     graph_, vertex_descriptors_[target_global_id_],
  //     boost::predecessor_map(
  //         boost::make_iterator_property_map(shortest_paths.begin(),
  //                                           get(boost::vertex_index, graph_)))
  //         .distance_map(boost::make_iterator_property_map(
  //             shortest_distances.begin(), get(boost::vertex_index, graph_))));

  // rep.parent_id_map[ind] = getVertexID(shortest_paths[ind]);
  // rep.distance_map[ind] = shortest_distances[ind];

  *parent_ = parent;
  *d_ = d;
}

bool Graph_For_Planning::findAstaerShortestPaths(int start_global_id, int target_global_id)
{
  // std::vector<GraphType_For_PLANNING::vertex_descriptor> p(num_vertices(graph_));
  // std::vector<double> d(num_vertices(graph_));
  // try
  // {
  //   // call astar named parameter interface
  //   boost::astar_search_tree(graph_, vertex_descriptors_[start_global_id],
  //                            0,
  //                            predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(boost::vertex_index, graph_)))
  //                                .distance_map(boost::make_iterator_property_map(d.begin(), boost::get(boost::vertex_index, graph_)))
  //                                .visitor(astar_goal_visitor<VertexDescriptor>(vertex_descriptors_[target_global_id])));
  // }
  // catch (found_goal fg)
  // { // found a path to the goal
  //   std::list<VertexDescriptor> shortest_path;
  //   for (VertexDescriptor v = vertex_descriptors_[target_global_id];; v = p[v])
  //   {
  //     shortest_path.push_front(v);
  //     if (p[v] == v)
  //       break;
  //   }
  //   std::cout << "Shortest path from " << start_global_id << " to "
  //             << target_global_id << ": ";
  // std::list<VertexDescriptor>::iterator spi = shortest_path.begin();
  // std::cout << start_global_id;
  // for (++spi; spi != shortest_path.end(); ++spi)
  //   cout << " -> " << name[*spi];
  // cout << endl
  //      << "Total travel time: " << d[goal] << endl;
  // return 0;
  // }
  // vertex_descriptors_[start_global_id];
  // vertex_descriptors_[start_global_id];
  // vertex_descriptors_[start_global_id];
  // vertex_descriptors_[start_global_id];
  // vertex_descriptors_[start_global_id];
  // vertex_descriptors_[start_global_id];
  return true;
}
