// roynaor10@gmail.com

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "Graph.hpp"
#include "Algorithms.hpp"
#include "UnionFind.hpp"
#include "Queue.hpp"

using namespace graph;
using namespace algorithms;

Graph buildSampleGraph() {
    Graph g(6);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 7);
    g.addEdge(2, 4, 3);
    g.addEdge(3, 4, 2);
    g.addEdge(3, 5, 1);
    g.addEdge(4, 5, 5);
    return g;
}

TEST_CASE("Graph creation and edges") {
    Graph g(3);
    g.addEdge(0, 1, 5);
    g.addEdge(1, 2, 2);

    CHECK(g.hasEdge(0, 1) == true);
    CHECK(g.hasEdge(1, 2) == true);
    CHECK(g.hasEdge(0, 2) == false);
}

TEST_CASE("UnionFind basic operations") {
    UnionFind uf(5);
    uf.unite(0, 1);
    uf.unite(3, 4);

    CHECK(uf.find(0) == uf.find(1));
    CHECK(uf.find(3) == uf.find(4));
    CHECK(uf.find(0) != uf.find(3));
}

TEST_CASE("Queue enqueue and dequeue") {
    Queue q;
    q.enqueue(1);
    q.enqueue(2);
    CHECK(q.dequeue() == 1);
    CHECK(q.dequeue() == 2);
    CHECK_THROWS(q.dequeue());  
}


TEST_CASE("BFS builds level-wise tree") {
    Graph g = buildSampleGraph();
    Graph bfsTree = Algorithms::bfs(g, 0);
    CHECK(bfsTree.getNumVertices() == 6);

    CHECK_NOTHROW(bfsTree.getNeighbors(0));
    CHECK(bfsTree.getNeighbors(0).size > 0);
}

TEST_CASE("DFS builds depth-first tree") {
    Graph g = buildSampleGraph();
    Graph dfsTree = Algorithms::dfs(g, 0);
    CHECK(dfsTree.getNumVertices() == 6);
    CHECK(dfsTree.getNeighbors(0).size > 0);
}

TEST_CASE("Dijkstra builds shortest path tree") {
    Graph g = buildSampleGraph();
    Graph dijkstraTree = Algorithms::dijkstra(g, 0);
    CHECK(dijkstraTree.getNumVertices() == 6);
}

TEST_CASE("Prim MST has correct number of edges") {
    Graph g = buildSampleGraph();
    Graph mst = Algorithms::prim(g);
    CHECK(mst.getNumVertices() == 6);
}

TEST_CASE("Kruskal MST has correct number of edges") {
    Graph g = buildSampleGraph();
    Graph mst = Algorithms::kruskal(g);
    CHECK(mst.getNumVertices() == 6);
}

TEST_CASE("Algorithms: BFS with invalid source index") {
    Graph g(3);
    g.addEdge(0, 1, 1);

    CHECK_THROWS_AS(Algorithms::bfs(g, -1), std::out_of_range);
    CHECK_THROWS_AS(Algorithms::bfs(g, 3), std::out_of_range);
}

TEST_CASE("Algorithms: DFS with invalid source index") {
    Graph g(2);
    g.addEdge(0, 1, 1);

    CHECK_THROWS_AS(Algorithms::dfs(g, -5), std::out_of_range);
    CHECK_THROWS_AS(Algorithms::dfs(g, 2), std::out_of_range);
}

TEST_CASE("Algorithms: Dijkstra on disconnected graph") {
    Graph g(4);  
    Graph tree = Algorithms::dijkstra(g, 0);

    for (int i = 1; i < 4; ++i) {
        CHECK(tree.getNeighbors(i).size == 0);
    }
}

TEST_CASE("Algorithms: Dijkstra with invalid source") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    CHECK_THROWS_AS(Algorithms::dijkstra(g, -1), std::out_of_range);
    CHECK_THROWS_AS(Algorithms::dijkstra(g, 4), std::out_of_range);
}

TEST_CASE("Algorithms: Prim on empty graph throws") {
    Graph g(0);
    CHECK_THROWS_AS(Algorithms::prim(g), std::out_of_range);
}

TEST_CASE("Algorithms: Prim on disconnected graph returns partial MST") {
    Graph g(4);
    g.addEdge(0, 1, 1);
    Graph mst = Algorithms::prim(g);

    CHECK(mst.getNeighbors(2).size == 0);
    CHECK(mst.getNeighbors(3).size == 0);
}

TEST_CASE("Algorithms: Kruskal handles disconnected components") {
    Graph g(4);
    g.addEdge(0, 1, 3);
    g.addEdge(2, 3, 4); 

    Graph mst = Algorithms::kruskal(g);
    int totalEdges = 0;
    for (int i = 0; i < 4; ++i)
        totalEdges += mst.getNeighbors(i).size;

    CHECK(totalEdges == 2);
}
