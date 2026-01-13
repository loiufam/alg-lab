#ifndef BLOCK_H
#define BLOCK_H
#pragma once

#include <bits/stdc++.h>
using namespace std;

struct Block {
    set<int> rows;  // 舞蹈链行id集合 
    set<int> cols;  // 从1开始编号,对应舞蹈链列数id
    bool is_spilited = false;
    
    Block() = default;

    size_t size() const { return rows.size(); }

    Block(set<int> r, set<int> c) : rows(move(r)), cols(move(c)) {}

    template <typename RowsContainer, typename ColsContainer>
    Block(const RowsContainer& r, const ColsContainer& c)
        : rows(r.begin(), r.end()), cols(c.begin(), c.end()) {}

    Block(const Block& other) {
        cols = other.cols;
        rows = other.rows;
    };

    void printBlock(int block_id) {
        if (rows.empty() || cols.empty()) return;
        cout << "Block " << block_id << ": { ";

        cout<< "cols: [ ";
        for(int c : cols){
            cout << c << " ";
        }
        cout << "], ";
        cout<< "rows: [ ";
        for(int r : rows){
            cout<< r << " ";
        }
        cout << "] }" << endl;
    }
};

#endif // BLOCK_H