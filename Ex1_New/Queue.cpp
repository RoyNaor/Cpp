// roynaor10@gmail.com

#include "Queue.hpp"

Queue::Queue() : front(0), rear(-1), size(0) {}

bool Queue::isEmpty() const {
    return size == 0;
}

bool Queue::isFull() const {
    return size == MAX_QUEUE_SIZE;
}

void Queue::enqueue(int value) {
    if (isFull()) throw std::overflow_error("Queue is full");
    rear = (rear + 1) % MAX_QUEUE_SIZE;
    data[rear] = value;
    size++;
}

int Queue::dequeue() {
    if (isEmpty()) throw std::underflow_error("Queue is empty");
    int value = data[front];
    front = (front + 1) % MAX_QUEUE_SIZE;
    size--;
    return value;
}

Queue::~Queue() {
    // Empty
}
