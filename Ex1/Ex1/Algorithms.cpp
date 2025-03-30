// roynaor10@gmail.com

#include "Algorithms.hpp"
#include "Queue.hpp"
#include "UnionFind.hpp"
#include <stdexcept>
#include <limits>

namespace algorithms {

    // -------------------------------------------------------------
    // BFS: builds a tree by level from the starting vertex
    graph::Graph Algorithms::bfs(const graph::Graph& graph, int source) {
        int n = graph.getNumVertices();
        graph::Graph tree(n);

        if (source < 0 || source >= graph.getNumVertices())
            throw std::out_of_range("Invalid source index");

        bool* visited = new bool[n](); // visited[i] = true if node i was visited
        Queue q;

        q.enqueue(source);
        visited[source] = true;

        while (!q.isEmpty()) {
            int current = q.dequeue();

            // Explore neighbors
            const graph::NeighborList& neighbors = graph.getNeighbors(current);
            for (int i = 0; i < neighbors.size; ++i) {
                int neighborId = neighbors.neighbors[i].id;
                int weight = neighbors.neighbors[i].weight;

                if (!visited[neighborId]) {
                    visited[neighborId] = true;
                    tree.addEdge(current, neighborId, weight); // add edge to BFS tree
                    q.enqueue(neighborId);
                }
            }
        }

        delete[] visited;
        return tree;
    }

    // -------------------------------------------------------------
    // DFS helper: recursively visits nodes and builds tree
    void dfsVisit(const graph::Graph& graph, graph::Graph& tree, int current, bool* visited) {
        visited[current] = true;

        const graph::NeighborList& neighbors = graph.getNeighbors(current);
        for (int i = 0; i < neighbors.size; ++i) {
            int neighborId = neighbors.neighbors[i].id;
            int weight = neighbors.neighbors[i].weight;

            if (!visited[neighborId]) {
                tree.addDirectedEdge(current, neighborId, weight); // add edge to DFS tree
                dfsVisit(graph, tree, neighborId, visited);        // visit next node
            }
        }
    }

    // DFS: builds a tree using depth-first traversal
    graph::Graph Algorithms::dfs(const graph::Graph& graph, int source) {
        int n = graph.getNumVertices();
        graph::Graph tree(n);

        if (source < 0 || source >= graph.getNumVertices())
            throw std::out_of_range("Invalid source index");

        bool* visited = new bool[n]();
        dfsVisit(graph, tree, source, visited);

        delete[] visited;
        return tree;
    }

    // -------------------------------------------------------------
    // Helper: relax step for Dijkstra — update distance & parent if shorter path is found
    void relax(int u, int v, int weight, int* dist, int* parent) {
        if (dist[u] + weight < dist[v]) {
            dist[v] = dist[u] + weight;
            parent[v] = u;
        }
    }

    // Dijkstra: builds shortest-path tree from source vertex
    graph::Graph Algorithms::dijkstra(const graph::Graph& graph, int source) {
        const int INF = std::numeric_limits<int>::max();
        int n = graph.getNumVertices();

        if (source < 0 || source >= graph.getNumVertices())
            throw std::out_of_range("Invalid source index");

        int* dist = new int[n];     // shortest distances from source
        int* parent = new int[n];   // parent in shortest-path tree
        bool* visited = new bool[n];

        for (int i = 0; i < n; ++i) {
            dist[i] = INF;
            parent[i] = -1;
            visited[i] = false;
        }

        dist[source] = 0;

        // Main loop — repeat for all nodes
        for (int i = 0; i < n; ++i) {
            int u = -1;
            int minDist = INF;

            // Pick the unvisited node with the smallest distance
            for (int j = 0; j < n; ++j) {
                if (!visited[j] && dist[j] < minDist) {
                    minDist = dist[j];
                    u = j;
                }
            }

            if (u == -1) break; // no reachable nodes left
            visited[u] = true;

            // Relax all neighbors of u
            const graph::NeighborList& neighbors = graph.getNeighbors(u);
            for (int j = 0; j < neighbors.size; ++j) {
                int v = neighbors.neighbors[j].id;
                int weight = neighbors.neighbors[j].weight;
                relax(u, v, weight, dist, parent);
            }
        }

        // Reconstruct shortest-path tree from parent[]
        graph::Graph tree(n);
        for (int v = 0; v < n; ++v) {
            if (parent[v] != -1) {
                int u = parent[v];
                int weight = dist[v] - dist[u];
                tree.addDirectedEdge(u, v, weight);
            }
        }

        delete[] dist;
        delete[] visited;
        delete[] parent;

        return tree;
    }

    // -------------------------------------------------------------
    // Prim's Algorithm: builds MST from starting node 0
    graph::Graph Algorithms::prim(const graph::Graph& graph) {
        const int INF = std::numeric_limits<int>::max();
        int n = graph.getNumVertices();

        if (n == 0)
            throw std::out_of_range("Graph is empty");

        bool* visited = new bool[n];  // track included nodes
        int* key = new int[n];        // min edge weight to tree
        int* parent = new int[n];     // parent in MST

        for (int i = 0; i < n; ++i) {
            visited[i] = false;
            key[i] = INF;
            parent[i] = -1;
        }

        key[0] = 0; // start from node 0

        for (int count = 0; count < n; ++count) {
            int u = -1, minKey = INF;

            // Pick unvisited node with smallest key value
            for (int v = 0; v < n; ++v) {
                if (!visited[v] && key[v] < minKey) {
                    minKey = key[v];
                    u = v;
                }
            }

            if (u == -1) break;
            visited[u] = true;

            // Update keys of neighbors
            const graph::NeighborList& neighbors = graph.getNeighbors(u);
            for (int i = 0; i < neighbors.size; ++i) {
                int v = neighbors.neighbors[i].id;
                int weight = neighbors.neighbors[i].weight;

                if (!visited[v] && weight < key[v]) {
                    key[v] = weight;
                    parent[v] = u;
                }
            }
        }

        // Build MST from parent[]
        graph::Graph mst(n);
        for (int v = 0; v < n; ++v) {
            if (parent[v] != -1) {
                mst.addDirectedEdge(parent[v], v, key[v]);
            }
        }

        delete[] visited;
        delete[] key;
        delete[] parent;

        return mst;
    }

    // -------------------------------------------------------------
    // Struct used to store edges for Kruskal's algorithm
    struct Edge {
        int u, v, weight;
    };

    // Collect all undirected edges and sort by weight
    Edge* getSortedEdges(const graph::Graph& graph, int& edgeCount) {
        int n = graph.getNumVertices();
        Edge* edges = new Edge[n * n];
        edgeCount = 0;

        for (int u = 0; u < n; ++u) {
            const graph::NeighborList& neighbors = graph.getNeighbors(u);
            for (int i = 0; i < neighbors.size; ++i) {
                int v = neighbors.neighbors[i].id;
                int w = neighbors.neighbors[i].weight;

                if (u < v) {  // avoid duplicate edges
                    edges[edgeCount++] = {u, v, w};
                }
            }
        }

        // Bubble sort edges by weight
        for (int i = 0; i < edgeCount - 1; ++i) {
            for (int j = 0; j < edgeCount - i - 1; ++j) {
                if (edges[j].weight > edges[j + 1].weight) {
                    Edge temp = edges[j];
                    edges[j] = edges[j + 1];
                    edges[j + 1] = temp;
                }
            }
        }

        return edges;
    }

    // Kruskal's Algorithm: builds MST using sorted edges and Union-Find
    graph::Graph Algorithms::kruskal(const graph::Graph& graph) {
        int n = graph.getNumVertices();
        graph::Graph mst(n);
        UnionFind uf(n);  // Disjoint set to detect cycles

        int edgeCount = 0;
        Edge* edges = getSortedEdges(graph, edgeCount);

        for (int i = 0; i < edgeCount; ++i) {
            int u = edges[i].u;
            int v = edges[i].v;
            int w = edges[i].weight;

            // Only add if u and v are in different sets
            if (!uf.connected(u, v)) {
                uf.unite(u, v);
                mst.addDirectedEdge(u, v, w);
            }
        }

        delete[] edges;
        return mst;
    }

} // namespace algorithms
