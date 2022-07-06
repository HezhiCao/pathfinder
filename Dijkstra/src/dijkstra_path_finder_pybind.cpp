#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dijkstra_path_finder.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace pathfinder {
PYBIND11_MODULE(path_finder, m) {
    py::class_<DijkstraPathFinder>(m, "DijkstraPathFinder")
        .def(py::init<>())
        // .def("path_splits", &DijkstraPathFinder::path_splits)
        .def("find", &DijkstraPathFinder::find, "find a navigation path using Dijkstra algorithm",
             "obstacle_map"_a, "start_point"_a, "end_points"_a, "obstacle_cost"_a = 1000.0);
}
}  // namespace pathfinder
