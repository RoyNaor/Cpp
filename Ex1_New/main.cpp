//
// Created by royna on 26/03/2025.
//

#include "Graph.hpp"
#include "Algorithms.hpp"
#include <iostream>

using namespace graph;
using namespace algorithms;

int main() {
    // Create a sample graph with 6 vertices
    Graph g(6);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 4, 3);
    g.addEdge(3, 5, 1);
    g.addEdge(4, 3, 2);
    g.addEdge(4, 5, 5);

    std::cout << "Original Graph:\n";
    g.print_graph();
    std::cout << "\n";

    // Test BFS
    std::cout << "BFS Tree from node 0:\n";
    Graph bfsTree = Algorithms::bfs(g, 0);
    bfsTree.print_graph();
    std::cout << "\n";

    // Test DFS
    std::cout << "DFS Tree from node 0:\n";
    Graph dfsTree = Algorithms::dfs(g, 0);
    dfsTree.print_graph();
    std::cout << "\n";

    // Test Dijkstra
    std::cout << "Dijkstra Shortest Path Tree from node 0:\n";
    Graph dijkstraTree = Algorithms::dijkstra(g, 0);
    dijkstraTree.print_graph();
    std::cout << "\n";

    // Test Prim
    std::cout << "Prim MST:\n";
    Graph primTree = Algorithms::prim(g);
    primTree.print_graph();
    std::cout << "\n";

    // Test Kruskal
    std::cout << "Kruskal MST:\n";
    Graph kruskalTree = Algorithms::kruskal(g);
    kruskalTree.print_graph();
    std::cout << "\n";

}
