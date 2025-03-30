CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g
TARGET = Ex1

SRCS = main.cpp Graph.cpp Algorithms.cpp Queue.cpp UnionFind.cpp
HEADERS = Graph.hpp Algorithms.hpp Queue.hpp UnionFind.hpp
OBJS = $(SRCS:.cpp=.o)

TEST_SRC = test.cpp
TEST_OBJ = $(TEST_SRC:.cpp=.o)
TEST_EXEC = run_tests

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_EXEC): $(TEST_OBJ) Graph.o Algorithms.o Queue.o UnionFind.o
	$(CXX) $(CXXFLAGS) -o $@ $^


.PHONY: test
test: $(TEST_EXEC)
	./$(TEST_EXEC)

valgrind: $(TARGET)
	valgrind --leak-check=full --track-origins=yes ./$(TARGET)

clean:
	rm -f $(TARGET) $(OBJS) $(TEST_EXEC) $(TEST_OBJ)

.PHONY: clean valgrind
