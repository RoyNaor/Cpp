// roynaor10@gmail.com

#include "Algorithms.hpp"
#include "Queue.hpp"
#include "Graph.hpp"
#include <iostream>

namespace algorithms {

    // returns a new graph that represents the BFS tree.
    graph::Graph Algorithms::bfs(const graph::Graph& g, int start) {
        Queue q;
        bool visited[graph::MAX_NODES] = {false}; // Tracks visited nodes

        graph::Graph tree;

        visited[start] = true;  // Mark the start node as visited
        q.enqueue(start);

        while (!q.isEmpty()) {
            int node = q.dequeue();

            // Traverse all neighbors of the current node
            for (int i = g.getHead(node); i != -1; i = g.getEdge(i).next) {
                int neighbor = g.getEdge(i).dest;

                if (!visited[neighbor]) {
                    visited[neighbor] = true;  // Mark neighbor as visited
                    q.enqueue(neighbor);

                    tree.addEdge(node, neighbor, g.getEdge(i).weight);
                }
            }
        }

        return tree;
    }

    void dfsVisit(const graph::Graph& g, int node, bool visited[], graph::Graph& tree) {
        visited[node] = true; // Mark the current node as visited

        for (int i = g.getHead(node); i != -1; i = g.getEdge(i).next) {
            int neighbor = g.getEdge(i).dest;

            if (!visited[neighbor]) {
                // Add the edge from current node to neighbor to the DFS tree
                tree.addEdge(node, neighbor, g.getEdge(i).weight);

                // Recursively visit the neighbor
                dfsVisit(g, neighbor, visited, tree);
            }
        }
    }

    graph::Graph Algorithms::dfs(const graph::Graph& g, int start) {
        graph::Graph tree;
        bool visited[graph::MAX_NODES] = {false};

        // Start DFS traversal from the given start node
        dfsVisit(g, start, visited, tree);

        return tree;
    }

    // Helper function
    void relax(int u, int v, int weight, int dist[], int parent[]) {
        if (dist[u] + weight < dist[v]) {
            dist[v] = dist[u] + weight;
            parent[v] = u;
        }
    }

    graph::Graph Algorithms::dijkstra(const graph::Graph& g, int start) {
        const int INF = 1e9; // infinity to set in the first traversal
        int dist[graph::MAX_NODES];
        int parent[graph::MAX_NODES];
        bool visited[graph::MAX_NODES] = {false};

        // Initialize distances and parents
        for (int i = 0; i < graph::MAX_NODES; i++) {
            dist[i] = INF;
            parent[i] = -1;
        }

        dist[start] = 0;

        for (int count = 0; count < graph::MAX_NODES; count++) {
            // Find the unvisited node with the smallest distance
            int u = -1;
            int minDist = INF;
            for (int i = 0; i < graph::MAX_NODES; i++) {
                if (!visited[i] && dist[i] < minDist) {
                    minDist = dist[i];
                    u = i;
                }
            }

            if (u == -1) break; // No more reachable nodes

            visited[u] = true;

            // Relax
            for (int i = g.getHead(u); i != -1; i = g.getEdge(i).next) {
                int v = g.getEdge(i).dest;
                int weight = g.getEdge(i).weight;

                if (!visited[v]) {
                    relax(u, v, weight, dist, parent);
                }
            }
        }

        // Build shortest path tree
        graph::Graph tree;
        for (int i = 0; i < graph::MAX_NODES; i++) {
            if (parent[i] != -1) {
                tree.addEdge(parent[i], i, dist[i] - dist[parent[i]]);
            }
        }

        return tree;
    }





//prim, kruskal

} // namespace algorithms
