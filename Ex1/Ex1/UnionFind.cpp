// roynaor10@gmail.com
#include "UnionFind.hpp"

// Constructor: initialize parent and rank arrays
UnionFind::UnionFind(int n) : size(n) {
    parent = new int[n]; 
    rank = new int[n];   

    for (int i = 0; i < n; ++i) {
        parent[i] = i; 
        rank[i] = 0;
    }
}

// Destructor: free allocated memory
UnionFind::~UnionFind() {
    delete[] parent;
    delete[] rank;
}

// Find the root of element x with path compression
int UnionFind::find(int x) {
    if (parent[x] != x) {
        // Recursively set parent[x] to its root
        parent[x] = find(parent[x]);
    }
    return parent[x];
}

// Unite the sets containing x and y using union by rank
void UnionFind::unite(int x, int y) {
    int rootX = find(x);
    int rootY = find(y);

    if (rootX == rootY) return; // already in the same set

    // Attach the smaller tree to the larger tree
    if (rank[rootX] < rank[rootY]) {
        parent[rootX] = rootY;
    } else if (rank[rootX] > rank[rootY]) {
        parent[rootY] = rootX;
    } else {
        // If ranks are equal, choose one and increase its rank
        parent[rootY] = rootX;
        rank[rootX]++;
    }
}

// Check whether x and y belong to the same set
bool UnionFind::connected(int x, int y) {
    return find(x) == find(y);
}
