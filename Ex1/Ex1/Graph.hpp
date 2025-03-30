// roynaor10@gmail.com

#ifndef GRAPH_HPP
#define GRAPH_HPP

namespace graph {

    // Neighboring vertex in the graph
    struct Neighbor {
        int id;
        int weight;
    };

    // A dynamic array of Neighbors
    struct NeighborList {
        Neighbor* neighbors;
        int size;
        int capacity;

        NeighborList();
        ~NeighborList();
        void add(int id, int weight);
        bool remove(int id);
        void print() const;
    };

    // Graph class
    class Graph {
    private:
        int numVertices;
        NeighborList* adjacency;

    public:
        Graph(int vertices);
        ~Graph();

        void addEdge(int src, int dest, int weight = 1);
        void addDirectedEdge(int src, int dest, int weight = 1);
        void removeEdge(int src, int dest);
        void print_graph() const;

        int getNumVertices() const;
        bool hasEdge(int src, int dest) const;
        const NeighborList& getNeighbors(int vertex) const;
    };

}

#endif // GRAPH_HPP
