#include "pplanner/graph_manager.h"

GraphManager::GraphManager()
{
    kdtree_ptr = NULL;
    reset();
}

void GraphManager::reset()
{
    // KD_TREE<PointType> newkdtree_ptr;

    if (kdtree_ptr != NULL)
    {
        // kdtree_ptr->~KD_TREE();
        delete kdtree_ptr;
    }

    kdtree_ptr = new KD_TREE<PointType>;

    // Reset graph.
    Graph_.reset(new Graph());

    // Vertex mapping.
    // Clear memory for all vertex pointers
    if (vertices_map_.size() > 0)
    {
        for (int i = 0; i < vertices_map_.size(); ++i)
        {
            delete vertices_map_[i];
        }
    }

    vertices_map_.clear();
    // edge_map_.clear();

    // Other params.
    subgraph_ind_ = -1;
    id_count_ = -1;

    StateVec state(0, 0, 0, 0);
    Vertex *root_vertex = new Vertex(generateVertexID(), state);
    PointVector point_initial_insert;
    PointType point_initial;
    point_initial.x = root_vertex->state.x();
    point_initial.y = root_vertex->state.y();
    point_initial.z = root_vertex->state.z();
    point_initial.data = root_vertex;
    point_initial_insert.push_back(point_initial);
    kdtree_ptr->Build(point_initial_insert);
    // std::cout << "root_vertex id is " << root_vertex->id << endl;
    vertices_map_[root_vertex->id] = root_vertex;

    if (root_vertex->id == 0)
        Graph_->addSourceVertex(0);
    else
        Graph_->addVertex(root_vertex->id);
}

int GraphManager::generateSubgraphIndex()
{
    return ++subgraph_ind_;
}

int GraphManager::generateVertexID()
{
    return ++id_count_;
}

void GraphManager::addVertex(Vertex *v)
{
    // kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
    PointVector point_insert;
    PointType new_point;

    new_point.x = v->state.x();
    new_point.y = v->state.y();
    new_point.z = v->state.z();

    new_point.data = v;

    point_insert.push_back(new_point);

    kdtree_ptr->Add_Points(point_insert, false);

    if (v->id == 0)
        Graph_->addSourceVertex(0);
    else
        Graph_->addVertex(v->id);
    vertices_map_[v->id] = v;
}

void GraphManager::removeVertex(Vertex *v)
{
    PointVector point_delete;
    PointType old_point;
    old_point.x = v->state.x();
    old_point.y = v->state.y();
    old_point.z = v->state.z();
    vector<float> PointDist;
    kdtree_ptr->Nearest_Search(old_point, 1, point_delete, PointDist, 0.01);

    if (point_delete.size() < 2)
    {
        kdtree_ptr->Delete_Points(point_delete);
        Graph_->removeVertex(v->id);
        vertices_map_.erase(v->id);
        delete v;
    }
    else
    {
        ROS_ERROR("When trying remove vertex, size of point_delete is %ld", point_delete.size());
        kdtree_ptr->Delete_Points(point_delete);
        Graph_->removeVertex(v->id);
        vertices_map_.erase(v->id);
        delete v;
    }
    // kdtree_ptr->Delete_Points(point_delete);
    // Graph_->removeVertex(v->id);
    // vertices_map_.erase(v->id);
    // delete v;
}

void GraphManager::addEdge(Vertex *v, Vertex *u, double weight)
{
    Graph_->addEdge(v->id, u->id, weight);
    // edge_map_[v->id].push_back(std::make_pair(u->id, weight));
    // edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void GraphManager::removeEdge(Vertex *v, Vertex *u)
{
    Graph_->removeEdge(v->id, u->id);
}

int GraphManager::getNumVertices()
{
    return Graph_->getNumVertices();
}
int GraphManager::getNumEdges()
{
    return Graph_->getNumEdges();
}

Vertex *GraphManager::getVertex(int id)
{
    // TODO Add a judgment of if the id exist.
    return vertices_map_[id];
}

void GraphManager::getLeafVertices(std::vector<Vertex *> &leaf_vertices)
{
    leaf_vertices.clear();
    for (int id = 0; id < getNumVertices(); ++id)
    {
        Vertex *v = getVertex(id);
    }
}

void GraphManager::findLeafVertices(const ShortestPathsReport &rep)
{
    int num_vertices = getNumVertices();
    for (int id = 0; id < num_vertices; ++id)
    {
        int pid = getParentIDFromShortestPath(id, rep);
    }
}

bool GraphManager::findShortestPaths(ShortestPathsReport &rep)
{
    return Graph_->findDijkstraShortestPaths(0, rep);
}

bool GraphManager::findShortestPaths(int source_id, ShortestPathsReport &rep)
{
    return Graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport &rep,
                                   bool source_to_target_order,
                                   std::vector<Vertex *> &path)
{
    std::vector<int> path_id;
    getShortestPath(target_id, rep, source_to_target_order, path_id);
    for (auto p = path_id.begin(); p != path_id.end(); ++p)
    {
        path.push_back(vertices_map_[*p]);
    }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport &rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d> &path)
{
    std::vector<int> path_id;
    getShortestPath(target_id, rep, source_to_target_order, path_id);
    for (auto p = path_id.begin(); p != path_id.end(); ++p)
    {
        path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                          vertices_map_[*p]->state.y(),
                                          vertices_map_[*p]->state.z()));
    }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport &rep,
                                   bool source_to_target_order,
                                   std::vector<StateVec> &path)
{
    std::vector<int> path_id;
    getShortestPath(target_id, rep, source_to_target_order, path_id);
    for (auto p = path_id.begin(); p != path_id.end(); ++p)
    {
        path.emplace_back(vertices_map_[*p]->state);
    }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport &rep,
                                   bool source_to_target_order,
                                   std::vector<int> &path)
{
    path.clear();
    if (!rep.status)
    {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
        return;
    }

    if (rep.parent_id_map.size() <= target_id)
    {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertext with ID [%d] doesn't exist in the graph", target_id);
        return;
    }

    if (target_id == rep.source_id)
    {
        path.push_back(target_id);
        return;
    }

    int parent_id = rep.parent_id_map.at(target_id);
    if (parent_id == target_id)
    {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] is isolated from the graph", target_id);
        return;
    }

    path.push_back(target_id); // current vertex id first
    path.push_back(
        parent_id); // its first parent, the rest is recursively looked up
    while (parent_id != rep.source_id)
    {
        parent_id = rep.parent_id_map.at(parent_id);
        path.push_back(parent_id);
    }

    // Initially, the path follows target to source order. Reverse if required.
    if (source_to_target_order)
    {
        std::reverse(path.begin(), path.end());
    }
}

double GraphManager::getShortestDistance(int target_id,
                                         const ShortestPathsReport &rep)
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

int GraphManager::getParentIDFromShortestPath(int target_id,
                                              const ShortestPathsReport &rep)
{
    if (!rep.status)
    {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
        return target_id;
    }

    if (rep.parent_id_map.size() <= target_id)
    {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] doesn't exist in the graph", target_id);
        return target_id;
    }

    return rep.parent_id_map.at(target_id);
}

bool GraphManager::getNearestVertex(const StateVec *state, Vertex **v_res)
{
    if (getNumVertices() <= 0)
        return false;
    // kdres *nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());

    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;

    vector<float> PointDist;

    kdtree_ptr->Nearest_Search(point_target, Nearest_Num0, search_result, PointDist, max_dist0);

    if (search_result.size() <= 0)
    {
        // kd_res_free(nearest);
        return false;
    }

    PointType point_out;
    point_out = search_result[0];
    *v_res = (Vertex *)point_out.data;
    // kd_res_free(nearest);
    return true;
}

bool GraphManager::getNearestVertexInRange(const StateVec *state, double range,
                                           Vertex **v_res)
{

    if (getNumVertices() <= 0)
        return false;

    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;
    vector<float> PointDist;
    kdtree_ptr->Nearest_Search(point_target, Nearest_Num0, search_result, PointDist, range);

    if (search_result.size() <= 0)
    {
        // kd_res_free(nearest);
        return false;
    }
    PointType point_out;
    point_out = search_result[0];
    *v_res = (Vertex *)point_out.data;

    Eigen::Vector3d dist;
    dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
        state->z() - (*v_res)->state.z();
    // kd_res_free(nearest);
    if (dist.norm() > range)
        return false;
    return true;
}

bool GraphManager::getNearestVertices(const StateVec *state, double range,
                                      std::vector<Vertex *> *v_res)
{
    // Notice that this might include the same vertex in the result.
    // if that vertex is added to the tree before.
    // Use the distance 0 or small threshold to filter out.

    if (getNumVertices() <= 0)
    {
        return false;
    }

    PointType point_target;
    point_target.x = state->x();
    point_target.y = state->y();
    point_target.z = state->z();

    PointVector search_result;

    vector<float> PointDist;

    kdtree_ptr->Nearest_Search(point_target, Nearest_Num1, search_result, PointDist, range);

    if (search_result.size() <= 0)
    {
        return false;
    }

    PointType point_out;
    v_res->clear();
    for (int i = 0; i < search_result.size(); ++i)
    {
        point_out = search_result[i];
        Vertex *new_neighbor = (Vertex *)point_out.data;
        v_res->push_back(new_neighbor);
    }

    return true;
}

bool GraphManager::getNearestVerticesInBox(const StateVec *state, double limitx_, double limity_,
                                           double limitz_, std::vector<Vertex *> *v_res)
{
    if (getNumVertices() <= 0)
    {
        return false;
    }

    PointVector search_result;
    BoxPointType BoxPoint;

    BoxPoint.vertex_min[0] = state->x() - limitx_;
    BoxPoint.vertex_min[1] = state->y() - limity_;
    BoxPoint.vertex_min[2] = state->z() - limitz_;

    BoxPoint.vertex_max[0] = state->x() + limitx_;
    BoxPoint.vertex_max[1] = state->y() + limity_;
    BoxPoint.vertex_max[2] = state->z() + limitz_;

    // BoxPoint->vertex_min[0] = state->x() - limitx_;
    // BoxPoint->vertex_min[1] = state->y() - limity_;
    // BoxPoint->vertex_min[2] = state->z() - limitz_;

    // BoxPoint->vertex_max[0] = state->x() + limitx_;
    // BoxPoint->vertex_max[1] = state->y() + limity_;
    // BoxPoint->vertex_max[2] = state->z() + limitz_;

    kdtree_ptr->Box_Search(BoxPoint, search_result);

    if (search_result.size() <= 0)
    {
        return false;
    }

    PointType point_out;
    v_res->clear();
    // std::cout << "size of graph manager box search is " << search_result.size() << std::endl;
    for (int i = 0; i < search_result.size(); ++i)
    {
        point_out = search_result[i];
        Vertex *new_neighbor = (Vertex *)point_out.data;
        v_res->push_back(new_neighbor);
        // std::cout << "id is " << new_neighbor->id << std::endl;
    }

    return true;
}

void GraphManager::updateVertexTypeInRange(StateVec &state, double range)
{
    std::vector<Vertex *> nearest_vertices;
    getNearestVertices(&state, range, &nearest_vertices);
}

void GraphManager::addVertex_to_ikdtree(Vertex *v)
{
    PointVector point_insert;
    PointType new_point;

    new_point.x = v->state.x();
    new_point.y = v->state.y();
    new_point.z = v->state.z();

    new_point.data = v;

    point_insert.push_back(new_point);

    kdtree_ptr->Add_Points(point_insert, false);
}

void GraphManager::removeVertex_from_ikdtree(Vertex *v)
{
    PointVector point_delete;

    BoxPointType BoxPoint;

    BoxPoint.vertex_min[0] = v->state.x() - 0.05;
    BoxPoint.vertex_min[1] = v->state.y() - 0.05;
    BoxPoint.vertex_min[2] = v->state.z() - 0.5;

    BoxPoint.vertex_max[0] = v->state.x() + 0.05;
    BoxPoint.vertex_max[1] = v->state.y() + 0.05;
    BoxPoint.vertex_max[2] = v->state.z() + 0.5;

    kdtree_ptr->Box_Search(BoxPoint, point_delete);
    // std::cout << "removing vertex and the size is " << point_delete.size() << std::endl;

    if (point_delete.size() > 1)
    {
        ROS_ERROR("THERE IS SOMETHING WRONG WHEN REMOVING POINT FROM IKD_TREE");
    }

    kdtree_ptr->Delete_Points(point_delete);
}

void GraphManager::saveGraph(const std::string &path)
{
    // Convert data to the msg type first
    // planner_msgs::Graph graph_msg;
    // convertGraphToMsg(graph_msg);

    // // Serialize the data, then write to a file
    // uint32_t serial_size = ros::serialization::serializationLength(graph_msg);
    // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    // ros::serialization::OStream stream(buffer.get(), serial_size);
    // ros::serialization::serialize(stream, graph_msg);

    // // Write to a file
    // std::ofstream wrt_file(path, std::ios::out | std::ios::binary);
    // wrt_file.write((char *)buffer.get(), serial_size);
    // wrt_file.close();
    // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Save the graph with [%d] vertices and [%d] edges to a file: %s",
    //               getNumVertices(), getNumEdges(), path.c_str());
}

void GraphManager::loadGraph(const std::string &path)
{
    // // Fill buffer with a serialized UInt32
    // std::ifstream read_file(path, std::ifstream::binary);
    // // Get the total number of bytes:
    // // http://www.cplusplus.com/reference/fstream/ifstream/rdbuf/
    // std::filebuf *pbuf = read_file.rdbuf();
    // std::size_t size = pbuf->pubseekoff(0, read_file.end, read_file.in);
    // pbuf->pubseekpos(0, read_file.in);
    // // Read the whole file to a buffer
    // boost::shared_array<uint8_t> buffer(new uint8_t[size]);
    // // char* buffer1=new char[size];
    // pbuf->sgetn((char *)buffer.get(), size);
    // read_file.close();

    // // Deserialize data into msg
    // planner_msgs::Graph graph_msg;
    // ros::serialization::IStream stream_in(buffer.get(), size);
    // ros::serialization::deserialize(stream_in, graph_msg);

    // // Reconstruct the graph
    // reset();
    // convertMsgToGraph(graph_msg);
    // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Load the graph with [%d] vertices and [%d] edges from a file: %s",
    //               getNumVertices(), getNumEdges(), path.c_str());
}
