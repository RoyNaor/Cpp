cmake_minimum_required(VERSION 3.30)
project(Ex1_New)

set(CMAKE_CXX_STANDARD 20)

add_executable(Ex1_New main.cpp
        Queue.hpp
        Queue.cpp
        Graph.cpp
        Graph.hpp
        main.cpp
        Algorithms.cpp
        Algorithms.hpp
        UnionFind.cpp
        UnionFind.hpp)
