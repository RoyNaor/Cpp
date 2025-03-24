// roynaor10@gmail.com
// Graph.hpp
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <stdexcept>

namespace graph {

    const int MAX_NODES = 50;
    const int MAX_EDGES = 100;

    class Graph {
        private:
            struct Edge {
                int dest;   // Destination node
                int weight; // Weight of the edge
                int next;   // Next edge index
            };

            Edge edges[MAX_EDGES]; // Edge list
            int head[MAX_NODES];   // Array that stores the first edge index per node
            int edgeCount;         // Total number of edges added

        public:
            Graph();  // Constructor

            bool edgeExists(int src, int dest) const;
            void addEdge(int src, int dest, int weight); // Add an edge
            void removeEdge(int src, int dest); // Remove an edge
            void printGraph(); // Prints the Graph

            const Edge& getEdge(int index) const { return edges[index]; }
            int getHead(int node) const { return head[node]; }
            int getEdgeCount() const { return edgeCount; }
            int getMaxNodes() const { return MAX_NODES; }
            int getMaxEdges() const { return MAX_EDGES; }
    };

} // namespace graph

#endif // GRAPH_HPP
