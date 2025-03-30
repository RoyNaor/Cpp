CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g
TARGET = Ex1

SRCS = main.cpp Graph.cpp Algorithms.cpp Queue.cpp UnionFind.cpp
HEADERS = Graph.hpp Algorithms.hpp Queue.hpp UnionFind.hpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

valgrind: $(TARGET)
	valgrind --leak-check=full --track-origins=yes ./$(TARGET)

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: clean valgrind
