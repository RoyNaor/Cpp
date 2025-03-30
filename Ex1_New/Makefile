# Project setup
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g
TARGET = Ex1_New

# Source files
SRCS = main.cpp \
       Graph.cpp \
       Algorithms.cpp \
       Queue.cpp \
       UnionFind.cpp

# Header dependencies
HEADERS = Graph.hpp \
          Algorithms.hpp \
          Queue.hpp \
          UnionFind.hpp

# Object files (replace .cpp with .o)
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Link the target
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $<

# Run valgrind on the built executable
valgrind: $(TARGET)
	valgrind --leak-check=full --track-origins=yes ./$(TARGET)

# Clean build files
clean:
	rm -f $(TARGET) $(OBJS)
