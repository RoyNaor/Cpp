// roynaor10@gmail.com

#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp"

namespace algorithms {

    class Algorithms {
    public:
        graph::Graph bfs(const graph::Graph& g, int start);
        graph::Graph dfs(const graph::Graph& g, int start);
        graph::Graph dijkstra(const graph::Graph& g, int start);
        void prim(const graph::Graph& g);
        void kruskal(const graph::Graph& g);
    };

} // namespace Algorithms

#endif // ALGORITHMS_HPP
