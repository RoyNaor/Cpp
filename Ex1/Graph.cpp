// roynaor10@gmail.com

#include "Graph.hpp"
#include <iostream>

namespace graph {

    // --- NeighborList Implementation --- //

    // Constructor: initialize neighbor list with small default capacity
    NeighborList::NeighborList() {
        size = 0;
        capacity = 2;
        neighbors = new Neighbor[capacity];
    }

    // Destructor: clean up dynamically allocated array
    NeighborList::~NeighborList() {
        delete[] neighbors;
    }

    // Add a neighbor (with id and weight) to the list
    void NeighborList::add(int id, int weight) {
        // Resize if array is full
        if (size == capacity) {
            capacity *= 2;
            Neighbor* newArray = new Neighbor[capacity];
            for (int i = 0; i < size; ++i) {
                newArray[i] = neighbors[i];
            }
            delete[] neighbors;
            neighbors = newArray;
        }

        neighbors[size++] = {id, weight};
    }

    // Remove a neighbor by id (returns true if removed)
    bool NeighborList::remove(int id) {
        for (int i = 0; i < size; ++i) {
            if (neighbors[i].id == id) {
                // Shift elements left to fill the gap
                for (int j = i; j < size - 1; ++j) {
                    neighbors[j] = neighbors[j + 1];
                }
                size--;
                return true;
            }
        }
        return false;
    }

    // Print all neighbors
    void NeighborList::print() const {
        for (int i = 0; i < size; ++i) {
            std::cout << " -> " << neighbors[i].id << " (w=" << neighbors[i].weight << ")";
        }
    }

    // --- Graph Implementation --- //

    // Constructor: initialize graph with given number of vertices
    Graph::Graph(int vertices) : numVertices(vertices) {
        adjacency = new NeighborList[vertices];
    }

    // Destructor: release memory for adjacency list
    Graph::~Graph() {
        delete[] adjacency;
    }

    // Return the number of vertices
    int Graph::getNumVertices() const {
        return numVertices;
    }

    // Check if an edge exists between src and dest
    bool Graph::hasEdge(int src, int dest) const {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            return false;
        }

        const NeighborList& neighbors = adjacency[src];
        for (int i = 0; i < neighbors.size; ++i) {
            if (neighbors.neighbors[i].id == dest) {
                return true;
            }
        }
        return false;
    }

    // Add an undirected edge between src and dest
    void Graph::addEdge(int src, int dest, int weight) {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            std::cerr << "Invalid vertex index\n";
            return;
        }
        adjacency[src].add(dest, weight);
        adjacency[dest].add(src, weight);
    }

    // Add a directed edge from src to dest
    void Graph::addDirectedEdge(int src, int dest, int weight) {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            std::cerr << "Invalid vertex index\n";
            return;
        }
        adjacency[src].add(dest, weight);
    }

    // Get the neighbor list for a vertex
    const NeighborList& Graph::getNeighbors(int vertex) const {
        if (vertex < 0 || vertex >= numVertices) {
            throw std::out_of_range("Invalid vertex index");
        }
        return adjacency[vertex];
    }

    // Remove undirected edge between src and dest
    void Graph::removeEdge(int src, int dest) {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            throw std::invalid_argument("Invalid vertex index");
        }

        bool removedFromSrc = adjacency[src].remove(dest);
        bool removedFromDest = adjacency[dest].remove(src);

        if (!removedFromSrc || !removedFromDest) {
            throw std::runtime_error("Edge does not exist");
        }
    }

    // Print the entire graph as adjacency list
    void Graph::print_graph() const {
        for (int i = 0; i < numVertices; ++i) {
            std::cout << "Vertex " << i << ":";
            adjacency[i].print();
            std::cout << "\n";
        }
    }

} // namespace graph
