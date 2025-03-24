// roynaor10@gmail.com

#include "Graph.hpp"

namespace graph {

// Constructor
    Graph::Graph() {
        std::fill(head, head + MAX_NODES, -1); // Initialize head array to -1
        edgeCount = 0;
    }

    bool Graph::edgeExists(int src, int dest) const {
        for (int i = head[src]; i != -1; i = edges[i].next) {
            if (edges[i].dest == dest) return true;
        }
        return false;
    }

    // Function to add an edge from 'src' to 'dest' with a given 'weight'
    void Graph::addEdge(int src, int dest, int weight) {
        if (src < 0 || src >= MAX_NODES || dest < 0 || dest >= MAX_NODES) {
            throw std::out_of_range("Error: Source or Destination node out of range.");
        }
        if (edgeCount + 2 > MAX_EDGES) {
            throw std::overflow_error("Error: Edge limit reached.");
        }
        if (edgeExists(src, dest) || edgeExists(dest, src)) {
            throw std::runtime_error("Error: Edge already exists.");
        }

        // adding both sides for undirected graph
        edges[edgeCount] = {dest, weight, head[src]};
        head[src] = edgeCount++;

        edges[edgeCount] = {src, weight, head[dest]};
        head[dest] = edgeCount++;
    }

    // Function to remove an edge from 'src' to 'dest'
    void Graph::removeEdge(int src, int dest) {
        if (src < 0 || src >= MAX_NODES || dest < 0 || dest >= MAX_NODES) {
            throw std::out_of_range("Error: Source or Destination node out of range.");
        }

        bool removed1 = false;
        bool removed2 = false;

        // Remove edge src to dest
        int prev = -1;
        for (int i = head[src]; i != -1; i = edges[i].next) {
            if (edges[i].dest == dest) {
                if (prev == -1) head[src] = edges[i].next;
                else edges[prev].next = edges[i].next;
                removed1 = true;
                break;
            }
            prev = i;
        }

        // Remove edge dest to src
        prev = -1;
        for (int i = head[dest]; i != -1; i = edges[i].next) {
            if (edges[i].dest == src) {
                if (prev == -1) head[dest] = edges[i].next;
                else edges[prev].next = edges[i].next;
                removed2 = true;
                break;
            }
            prev = i;
        }

        if (!removed1 || !removed2) {
            throw std::runtime_error("Error: Edge not found in one or both directions.");
        }
    }

    // Function to print the adjacency list
    void Graph::printGraph() {
        for (int i = 0; i < MAX_NODES; i++) {
            for (int j = head[i]; j != -1; j = edges[j].next) {
                int dest = edges[j].dest;
                if (i < dest) {  // Print each edge only once
                    std::cout << "Edge (" << i << " - " << dest << ") weight: " << edges[j].weight << "\n";
                }
            }
        }
    }

} // namespace graph
