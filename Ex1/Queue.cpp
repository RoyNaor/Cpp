// roynaor10@gmail.com

#include "Queue.hpp"

// Constructor: initialize queue pointers and size
Queue::Queue() : front(0), rear(-1), size(0) {}

// Check if the queue is empty
bool Queue::isEmpty() const {
    return size == 0;
}

// Check if the queue is full (based on fixed MAX_QUEUE_SIZE)
bool Queue::isFull() const {
    return size == MAX_QUEUE_SIZE;
}

// Add an element to the rear of the queue
void Queue::enqueue(int value) {
    if (isFull()) 
        throw std::overflow_error("Queue is full");

    // Circular increment of rear pointer
    rear = (rear + 1) % MAX_QUEUE_SIZE;
    data[rear] = value;
    size++;
}

// Remove and return the front element
int Queue::dequeue() {
    if (isEmpty()) 
        throw std::underflow_error("Queue is empty");

    int value = data[front];

    // Circular increment of front pointer
    front = (front + 1) % MAX_QUEUE_SIZE;
    size--;
    return value;
}

// Destructor: not needed here but added for best practice
Queue::~Queue() {
    // No cleanup required
}
