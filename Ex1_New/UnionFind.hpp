// roynaor10@gmail.com

#ifndef UNION_FIND_HPP
#define UNION_FIND_HPP

class UnionFind {
    private:
        int* parent;
        int* rank;
        int size;

    public:
        UnionFind(int n);
        ~UnionFind();

        int find(int x);
        void unite(int x, int y);
        bool connected(int x, int y);
    };

#endif // UNION_FIND_HPP
