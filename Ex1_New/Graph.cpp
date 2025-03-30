//
// Created by royna on 26/03/2025.
//

#include "Graph.hpp"
// Graph2.cpp
#include "Graph.hpp"
#include <iostream>

namespace graph {

    // --- NeighborList Implementation --- //

    NeighborList::NeighborList() {
        size = 0;
        capacity = 2;
        neighbors = new Neighbor[capacity];
    }

    NeighborList::~NeighborList() {
        delete[] neighbors;
    }

    void NeighborList::add(int id, int weight) {
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

    bool NeighborList::remove(int id) {
        for (int i = 0; i < size; ++i) {
            if (neighbors[i].id == id) {
                for (int j = i; j < size - 1; ++j) {
                    neighbors[j] = neighbors[j + 1];
                }
                size--;
                return true;
            }
        }
        return false;
    }

    void NeighborList::print() const {
        for (int i = 0; i < size; ++i) {
            std::cout << " -> " << neighbors[i].id << " (w=" << neighbors[i].weight << ")";
        }
    }

    // --- Graph Implementation --- //

    Graph::Graph(int vertices) : numVertices(vertices) {
        adjacency = new NeighborList[vertices];
    }

    Graph::~Graph() {
        delete[] adjacency;
    }

    int Graph::getNumVertices() const {
        return numVertices;
    }

    void Graph::addEdge(int src, int dest, int weight) {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            std::cerr << "Invalid vertex index\n";
            return;
        }
        adjacency[src].add(dest, weight);
        adjacency[dest].add(src, weight);
    }

    void Graph::addDirectedEdge(int src, int dest, int weight) {
        if (src < 0 || src >= numVertices || dest < 0 || dest >= numVertices) {
            std::cerr << "Invalid vertex index\n";
            return;
        }
        adjacency[src].add(dest, weight);
    }

    const NeighborList& Graph::getNeighbors(int vertex) const {
        if (vertex < 0 || vertex >= numVertices) {
            throw std::out_of_range("Invalid vertex index");
        }
        return adjacency[vertex];
    }



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

    void Graph::print_graph() const {
        for (int i = 0; i < numVertices; ++i) {
            std::cout << "Vertex " << i << ":";
            adjacency[i].print();
            std::cout << "\n";
        }
    }
}
