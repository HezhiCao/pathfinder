#ifndef DIJKSTRA_PATH_FINDER_H
#define DIJKSTRA_PATH_FINDER_H

#include <stdio.h>

#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <vector>
#include "eigen_hash.h"

namespace pathfinder {
class DijkstraPathFinder {
   public:
    std::vector<Eigen::Vector2i> find(const Eigen::MatrixXf &obstacle_map,
                                      const Eigen::Vector2i &start_point,
                                      const Eigen::MatrixXi &end_points, float obstacle_cost);
    Eigen::VectorXi path_splits;

   private:
    void reset(const Eigen::MatrixXf &obstacle_map, const Eigen::MatrixXi &end_points);
    void expand(const Eigen::MatrixXf &obstacle_map, const Eigen::Vector2i &start_point,
                const Eigen::MatrixXi &end_points);
    void addNode(const Eigen::MatrixXf &obstacle_map, const Eigen::Vector2i &previous_point,
                 const Eigen::Vector2i &next_point, const Eigen::MatrixXi &end_points,
                 bool is_vertical);
    std::vector<Eigen::Vector2i> extractPath(const Eigen::Vector2i &start_point);

    bool checkWithinBound(const Eigen::Vector2i &query_point);
    bool checkWithinBound(const Eigen::MatrixXi &query_points);

    float obstacle_cost_ = 5.0f;
    const float step_distance_ = 1.0f;
    const float diagonal_step_distance_ = std::sqrt(2.0f);

    Eigen::VectorXf min_distances_;
    Eigen::MatrixXf distance_grid_;  // distance to the start point
    Eigen::MatrixXi nearest_points_;

    using ExpandedNode = std::pair<Eigen::Vector2i, float>;
    static bool nodePriorityComparator(const ExpandedNode &lhs, const ExpandedNode &rhs) {
        return lhs.second > rhs.second;
    }

    std::priority_queue<ExpandedNode, std::vector<ExpandedNode>, decltype(nodePriorityComparator) *>
        node_queue_{nodePriorityComparator};

    std::unordered_map<Eigen::Vector2i, Eigen::Vector2i> expanded_trace_;
};
}

#endif  // DIJKSTRA_PATH_FINDER_H
