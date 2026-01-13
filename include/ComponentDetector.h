#ifndef COMPONENT_DETECTOR_H
#define COMPONENT_DETECTOR_H

#pragma once

#include "SplayETT.h"
#include "Block.h"
#include <map>
#include <set>
#include <stack>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <shared_mutex>
#include <memory>
#include <atomic>
#include <optional>
#include <cstdint>
#include <stdexcept>
#include <algorithm>

using comps = vector<Block>;

// 边的层级信息(用于分层搜索替代边)
struct EdgeInfo {
    int level;           // 边的层级
    bool is_tree;        // 是否为树边
    int component_root;  // 所属连通分量的root节点

    EdgeInfo(int lvl = 0, bool tree = false) : level(lvl), is_tree(tree) {}
};

// 分层非树边存储结构
struct LayeredNonTreeEdges {
    std::vector<std::unordered_set<unsigned long long>> levels;  // levels[i] = 第i层的非树边集合
    std::unordered_map<unsigned long long, int> edge_to_level;   // 边 -> 当前所在层级
    int max_level;
    
    LayeredNonTreeEdges(int max_lvl) : max_level(max_lvl) {
        levels.resize(max_lvl + 1);
    }
    
    // 添加边到指定层级
    void addEdge(unsigned long long key, int level) {
        if (level < 0 || level > max_level) return;
        levels[level].insert(key);
        edge_to_level[key] = level;
    }
    
    // 删除边
    void removeEdge(unsigned long long key) {
        auto it = edge_to_level.find(key);
        if (it != edge_to_level.end()) {
            int level = it->second;
            levels[level].erase(key);
            edge_to_level.erase(it);
        }
    }
    
    // 降级边到下一层
    bool demoteEdge(unsigned long long key) {
        auto it = edge_to_level.find(key);
        if (it == edge_to_level.end()) return false;
        
        int curr_level = it->second;
        if (curr_level <= 0) return false;  // 已经在最底层
        
        levels[curr_level].erase(key);
        levels[curr_level - 1].insert(key);
        edge_to_level[key] = curr_level - 1;
        return true;
    }

    bool promoteEdge(unsigned long long key) {
        auto it = edge_to_level.find(key);
        if (it == edge_to_level.end()) return false;
        
        int curr_level = it->second;
        if (curr_level >= max_level) return false;
        
        levels[curr_level].erase(key);
        levels[curr_level + 1].insert(key);
        edge_to_level[key] = curr_level + 1;
        return true;
    }
    
    // 获取边的当前层级
    int getLevel(unsigned long long key) const {
        auto it = edge_to_level.find(key);
        return (it != edge_to_level.end()) ? it->second : -1;
    }
    
    // 获取指定层级的所有边
    const std::unordered_set<unsigned long long>& getEdgesAtLevel(int level) const {
        static const std::unordered_set<unsigned long long> empty_set;
        if (level < 0 || level > max_level) return empty_set;
        return levels[level];
    }
    
    void clear() {
        for (auto& level : levels) {
            level.clear();
        }
        edge_to_level.clear();
    }
};

// 并查集实现
class UnionFind {
private:
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
    
public:
    void make_set(int x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            rank[x] = 0;
        }
    }
    
    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // 路径压缩
        }
        return parent[x];
    }
    
    void unite(int x, int y) {
        int px = find(x);
        int py = find(y);
        if (px == py) return;
        
        // 按秩合并
        if (rank[px] < rank[py]) {
            parent[px] = py;
        } else if (rank[px] > rank[py]) {
            parent[py] = px;
        } else {
            parent[py] = px;
            rank[px]++;
        }
    }

    unordered_map<int, set<int>> get_components() {
        unordered_map<int, set<int>> components;
        for (const auto& [node, _] : parent) {
            int root = find(node);
            components[root].insert(node);
        }
        return components;
    }
};

// 连通分量信息（线程私有）
struct ComponentInfo {
    std::unordered_set<int> vertices;
    std::unordered_set<unsigned long long> tree_edges;
    std::unique_ptr<LayeredNonTreeEdges> non_tree_edges;
    int tree_root;
    
    ComponentInfo() = default;
    ComponentInfo(const ComponentInfo&) = delete;
    ComponentInfo& operator=(const ComponentInfo&) = delete;
    ComponentInfo(ComponentInfo&&) = default;
    ComponentInfo& operator=(ComponentInfo&&) = default;
};

// 用于保存 Cover 操作的历史记录（线程私有）
struct CoverHistory {
    int col;
    std::set<int> prev_rows;  // Cover前的行集合
    std::vector<int> removed_rows;  // 被删除的行
    
    // ETT操作记录（用于回滚）
    std::vector<std::pair<int, int>> cut_tree_edges;
    std::vector<std::pair<int, int>> removed_nontree_edges;
    std::vector<std::pair<int, int>> added_replacement_edges;
    
    // 连通分量信息
    std::vector<std::set<int>> prev_components;  // Cover前的连通分量
    std::vector<std::set<int>> new_components;   // Cover后的连通分量
    
    void clear() {
        col = -1;
        prev_rows.clear();
        removed_rows.clear();
        cut_tree_edges.clear();
        removed_nontree_edges.clear();
        added_replacement_edges.clear();
        prev_components.clear();
        new_components.clear();
    }
    
    bool isEmpty() const {
        return removed_rows.empty();
    }
};

// 边操作历史（记录操作序列，用于反向恢复）
struct EdgeOperationHistory {
    // 删除的边序列（按删除顺序）
    std::vector<std::pair<int, int>> deleted_edges;
    
    // 每条删除的边的详细信息
    struct EdgeOperation {
        unsigned long long key;
        int u, v;
        bool was_tree_edge;      // 删除前是否为树边
        int original_level;      // 删除前的层级
        int component_id;        // 所属分量ID
        bool found_replacement;  // 是否找到了替代边
        std::pair<int, int> replacement_edge;  // 替代边（如果有）
        int replacement_level;   // 替代边原来的层级
        std::vector<std::pair<unsigned long long, int>> demoted_edges;  // 被降级的边及其原层级
    };
    
    std::vector<EdgeOperation> operations;
    
    void clear() {
        deleted_edges.clear();
        operations.clear();
    }
};

class ComponentDetector {
private:
    int num_rows;
    int num_cols;

    // 列到行的映射（静态，初始化后不变）
    std::unordered_map<int, std::vector<int>> col_to_rows;
    std::unordered_map<int, std::unordered_set<int>> row_to_cols;

    // 图的邻接表
    vector<unordered_set<int>> adj_list;  // adj_list[row] = {邻居}
    vector<bool> row_active;
    std::unordered_set<int> active_cols;

    unique_ptr<SplayETT> ett;

    std::shared_mutex access_ett_mutex;

    std::unordered_set<unsigned long long> tree_edges;      // ETT中的实际树边
    std::unordered_set<unsigned long long> non_tree_edges;  // 逻辑存在但不在ETT中的边
    std::unordered_map<unsigned long long, EdgeInfo> edge_info_map;  // edge_key -> EdgeInfo
    // ComponentInfo component_info;  // 单个分量信息（线程私有，无需锁）
    std::unordered_map<int, ComponentInfo> component_map;            // root -> ComponentInfo

    // 线程私有的历史栈
    // static thread_local std::vector<CoverHistory> cover_stack_;

    // 操作历史栈（每层递归对应一个历史记录）
    std::vector<EdgeOperationHistory> operation_stack;

    // 添加调试标志
    bool debug_mode = false;
    std::ofstream debug_log;

    inline unsigned long long makeEdgeKey(int u, int v) const {
        if (u > v) std::swap(u, v);
        return (static_cast<unsigned long long>(u) << 32) | 
               static_cast<unsigned long long>(v);
    }

    inline void decodeEdgeKey(unsigned long long key, int& u, int& v) const {
        u = static_cast<int>(key >> 32);
        v = static_cast<int>(key & 0xFFFFFFFF);
    }

    /**
     * 添加一条边到邻接表（双向）
     */
    void addEdgeToAdjList(int u, int v) {
        if (u == v) return;
        adj_list[u].insert(v);
        adj_list[v].insert(u);
    }
    
    /**
     * 从邻接表中删除一条边（双向）
     */
    void removeEdgeFromAdjList(int u, int v) {
        if (u == v) return;
        adj_list[u].erase(v);
        adj_list[v].erase(u);
    }

    // 计算分层的最大层级
    int calculateMaxLevel(int num_vertices) {
        if (num_vertices <= 1) return 0;
        return static_cast<int>(std::ceil(std::log2(num_vertices)));
    }

    // BFS构建生成森林
    void BuildSpanningForest();

    // 减量式生成连通分量 - 返回受影响的连通分量
    // std::vector<std::set<int>> DecGenerateCC(
    //     const std::set<int>& deleted_vertices,
    //     const std::set<int>& prev_component);
    
    // 在指定分量中查找替代边
    // std::optional<std::pair<int,int>> FindReplacementInComponent(
    //     int u, int v, int comp_root);

    // 分层查找替代边 - 从高层向低层查找
    std::optional<std::pair<int,int>> FindReplacementLayered(ComponentInfo& comp_info);

    std::vector<Block> convertComponentsToBlocks(const std::vector<std::set<int>>& components);

public:

    // static thread_local bool is_updated;

    ComponentDetector(const int n, const int m);
    ~ComponentDetector() = default;

    void Initialize(const std::unordered_map<int, std::vector<int>>& col_rows_map);
  
    /**
     * 删除边操作（在Cover之后、递归前调用）
     * 记录所有操作以便反向恢复
     */
    void DeleteEdges(const std::set<int>& deleted_rows);
    
    /**
     * 添加边操作（在Uncover时调用）
     * 反向执行DeleteEdges的操作
     */
    void AddEdges(const std::set<int>& add_rows);

    // 同步更新矩阵行信息
    void Cover(int c);
    void Uncover();

    // 在特定行集合内部寻找分块(统一对外接口)
    std::vector<Block> GetBlocks(const std::set<int>& block_rows);
  
    // 工具函数
    bool IsConnected(int u, int v) const {
        return ett->connected(u, v);
    }

    int GetComponentId(int u) const {
        return ett->getComponentId(u);
    }

    size_t GetNumComponents() const {
        return component_map.size();
    }

    vector<Block> detect_blocks(const set<int>& block_rows);

};

#endif // COMPONENTDETECTOR_H