# Ex1 – Graph Algorithms in C++

A C++ project that implements fundamental graph algorithms from scratch,
including BFS, DFS, Dijkstra, Prim, and Kruskal, along with data structures like a queue and union-find.

---

## 📚 Features

- ✅ Custom `Graph` and `NeighborList` classes
- ✅ BFS & DFS traversal trees
- ✅ Dijkstra's shortest path algorithm
- ✅ Prim's and Kruskal's algorithms for Minimum Spanning Trees
- ✅ Fixed-size queue (for BFS)
- ✅ Union-Find with path compression and union by rank (for Kruskal)
- ✅ Doctest unit tests with edge cases
- ✅ Clean and modular design

---

## 📁 Project Structure

```
Ex1/
├── Algorithms.cpp / .hpp     # All graph algorithms
├── Graph.cpp / .hpp          # Graph + NeighborList classes
├── Queue.cpp / .hpp          # Fixed-size circular queue
├── UnionFind.cpp / .hpp      # Union-Find DS for Kruskal
├── main.cpp                  # Entry point (example usage)
├── test.cpp                  # Doctest test file
├── Makefile                  # Build + test automation
└── README.md                 # You’re reading it!
```

---

## 👷️ Building the Project

### Compile & Run the Main Program

```bash
make
./Ex1
```
---

## 🔪 Running Unit Tests

The project uses [doctest](https://github.com/doctest/doctest) for lightweight unit testing.

### Run all tests:

```bash
make test
```
---

## 🔍 Memory Leak Checking (Valgrind)

```bash
make valgrind
```

> Make sure Valgrind and doctest is installed:  
> `sudo apt install valgrind`
> `sudo apt install doctest`

---

## 🧱 Algorithms Implemented

| Algorithm | File | Description |
|----------|------|-------------|
| BFS      | `Algorithms.cpp` | Builds a BFS tree from a source |
| DFS      | `Algorithms.cpp` | Recursive DFS tree builder |
| Dijkstra | `Algorithms.cpp` | Builds shortest path tree |
| Prim     | `Algorithms.cpp` | MST using greedy approach |
| Kruskal  | `Algorithms.cpp` | MST using union-find |

---

## 👨‍💼 Author

**Roy Naor**  
📧 roynaor10@gmail.com  

---


