// roynaor10@gmail.com

#ifndef EX1_NEW_ALGORITHMS_HPP
#define EX1_NEW_ALGORITHMS_HPP

#include "Graph.hpp"

namespace algorithms {

    class Algorithms {
        public:
            static graph::Graph bfs(const graph::Graph& graph, int source);
            static graph::Graph dfs(const graph::Graph& graph, int source);
            static graph::Graph dijkstra(const graph::Graph& graph, int source);
            static graph::Graph prim(const graph::Graph& graph);
            static graph::Graph kruskal(const graph::Graph& graph);
    };

} // namespace Algorithms


#endif //EX1_NEW_ALGORITHMS_HPP
