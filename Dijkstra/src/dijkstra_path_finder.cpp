#include "dijkstra_path_finder.h"

namespace pathfinder {
std::vector<Eigen::Vector2i> DijkstraPathFinder::find(const Eigen::MatrixXf &obstacle_map,
                                                      const Eigen::Vector2i &start_point,
                                                      const Eigen::MatrixXi &end_points,
                                                      float obstacle_cost) {
    obstacle_cost_ = obstacle_cost;
    reset(obstacle_map, end_points);
    expand(obstacle_map, start_point, end_points);
    return extractPath(start_point);
}

void DijkstraPathFinder::expand(const Eigen::MatrixXf &obstacle_map,
                                const Eigen::Vector2i &start_point,
                                const Eigen::MatrixXi &end_points) {
    if (!checkWithinBound(start_point) || checkWithinBound(end_points)) {
        throw std::runtime_error("Invalid start/end points");
    }
    node_queue_.emplace(start_point, 0.0);
    distance_grid_(start_point[0], start_point[1]) = 0;

    nearest_points_ = Eigen::MatrixXi(2, end_points.cols());
    nearest_points_.row(0).array() = start_point[0];
    nearest_points_.row(1).array() = start_point[1];
    Eigen::Vector2i expanding_point;

    while (!node_queue_.empty()) {
        ExpandedNode expanding_node = node_queue_.top();
        node_queue_.pop();
        expanding_point = expanding_node.first;
        if (expanding_node.second > distance_grid_(expanding_point[0], expanding_point[1]))
            continue;

        bool all_found = true;
        for (int i = 0; i < end_points.cols(); ++i) {
            bool is_found = (expanding_point == end_points.col(i));
            if (is_found) nearest_points_.col(i) = expanding_point;
            all_found = all_found && is_found;
        }
        if (all_found) break;

        for (int dx = -1; dx < 2; ++dx) {
            for (int dy = -1; dy < 2; ++dy) {
                if (0 == dx && 0 == dy) continue;
                bool is_vertical = ((abs(dx) + abs(dy)) != 2);
                Eigen::Vector2i next_expanding_point = expanding_point + Eigen::Vector2i(dx, dy);
                if (!checkWithinBound(next_expanding_point)) continue;

                addNode(obstacle_map, expanding_point, next_expanding_point, end_points,
                        is_vertical);
            }
        }
    }
}

void DijkstraPathFinder::addNode(const Eigen::MatrixXf &obstacle_map,
                                 const Eigen::Vector2i &previous_point,
                                 const Eigen::Vector2i &next_point,
                                 const Eigen::MatrixXi &end_points, bool is_vertical) {
    if ((obstacle_cost_ < 0 && obstacle_map(next_point[0], next_point[1]) > 0.01) ||
        obstacle_map(next_point[0], next_point[1]) < 0.95) {
        return;
    }
    double step_distance = (is_vertical ? step_distance_ : diagonal_step_distance_);
    double new_distance = distance_grid_(previous_point[0], previous_point[1]) + step_distance +
                          obstacle_map(next_point[0], next_point[1]) * obstacle_cost_;
    if (new_distance >= distance_grid_(next_point[0], next_point[1]) + 1e-3) {
        return;
    }
    distance_grid_(next_point[0], next_point[1]) = new_distance;
    node_queue_.emplace(next_point, new_distance);
    expanded_trace_[next_point] = previous_point;
    for (int i = 0; i < end_points.cols(); ++i) {
        double distance = (end_points.col(i) - next_point).norm();
        if (distance < min_distances_[i]) {
            min_distances_[i] = distance;
            nearest_points_.col(i) = next_point;
        }
    }
}

std::vector<Eigen::Vector2i> DijkstraPathFinder::extractPath(const Eigen::Vector2i &start_point) {
    std::vector<Eigen::Vector2i> result;
    int path_num = nearest_points_.cols();
    for (int i = 0, node_cnt = 0; i < path_num; ++i) {
        Eigen::Vector2i current_point = nearest_points_.col(i);
        while (current_point != start_point) {
            result.push_back(current_point);
            current_point = expanded_trace_.find(current_point)->second;
            ++node_cnt;
        }
        path_splits[i] = ++node_cnt;
        result.push_back(start_point);
    }

    std::reverse(result.begin(), result.end());
    path_splits = Eigen::VectorXi::Constant(path_num, result.size()) - path_splits;
    return result;
}

void DijkstraPathFinder::reset(const Eigen::MatrixXf &obstacle_map,
                               const Eigen::MatrixXi &end_points) {
    if (obstacle_map.cols() != distance_grid_.cols() ||
        obstacle_map.rows() != distance_grid_.rows()) {
        distance_grid_ = Eigen::MatrixXf::Constant(obstacle_map.rows(), obstacle_map.cols(),
                                                   std::numeric_limits<float>::max());
    } else {
        distance_grid_.fill(std::numeric_limits<float>::max());
    }
    expanded_trace_ = std::unordered_map<Eigen::Vector2i, Eigen::Vector2i>{};
    path_splits = Eigen::VectorXi::Constant(end_points.cols(), 0);
    node_queue_ = std::priority_queue<ExpandedNode, std::vector<ExpandedNode>,
                                      decltype(nodePriorityComparator) *>{nodePriorityComparator};
    min_distances_ =
        Eigen::VectorXf::Constant(end_points.cols(), std::numeric_limits<float>::max());
}

bool DijkstraPathFinder::checkWithinBound(const Eigen::Vector2i &query_point) {
    return !((query_point[0] < 0) || query_point[1] < 0 || query_point[0] > distance_grid_.rows() ||
             query_point[1] > distance_grid_.cols());
}

bool DijkstraPathFinder::checkWithinBound(const Eigen::MatrixXi &query_points) {
    if (query_points.rows() != 2) {
        throw std::runtime_error("The dimension of end points should be (2, n)");
    }
    bool mask = true;
    for (int i = 0; i < query_points.cols(); ++i) {
        mask = mask && !((query_points(0, i) < 0) || query_points(1, i) < 0 ||
                         query_points(0, i) > distance_grid_.rows() ||
                         query_points(1, i) > distance_grid_.cols());
    }
    return mask;
}
}  // namespace pathfinder
//

// int main() {
//     pathfinder::DijkstraPathFinder planner;
//     Eigen::MatrixXf obstacle_map = Eigen::MatrixXf::Constant(100, 100, 0);
//     obstacle_map.block(20, 20, 30, 70).setOnes();
//     obstacle_map.block(20, 20, 70, 30).setOnes();
//     Eigen::Vector2i start_point{10, 10};
//     Eigen::MatrixXi end_points{{60, 80}, {80, 60}};
//     float obstacle_cost = -1.0f;
//     planner.find(obstacle_map, start_point, end_points, obstacle_cost);
//     return 0;
// }
