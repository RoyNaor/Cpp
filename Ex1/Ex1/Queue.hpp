// roynaor10@gmail.com

#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <stdexcept>

const int MAX_QUEUE_SIZE = 100;

struct Queue {
    int data[MAX_QUEUE_SIZE];
    int front;
    int rear;
    int size;

    Queue(); // constructor
    ~Queue(); // Destructor

    bool isEmpty() const;
    bool isFull() const;
    void enqueue(int value);
    int dequeue();
};

#endif // QUEUE_HPP
