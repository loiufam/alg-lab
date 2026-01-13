#ifndef SPLAY_ETT_H
#define SPLAY_ETT_H

#include <omp.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <functional>
#include <iostream>
#include <climits>
#include <shared_mutex>
#include <mutex>
#include <atomic>


class SplayETT {
private:
    struct Node {
        Node *left, *right, *parent;
        int vertex;      // 对应的顶点编号（-1表示边节点）
        unsigned long long edgeKey;  // 边的唯一标识 (u,v) -> key
        bool reversed;   // 标记翻转
        size_t node_id; // 节点唯一id
        
        Node(int v = -1, unsigned long long key = 0) 
            : left(nullptr), right(nullptr), parent(nullptr),
              vertex(v), edgeKey(key), reversed(false) {}
    };
    
    struct Edge {
        Node *first, *second;
        int u, v;
    };
    
    std::vector<Node*> vertexNodes;
    std::unordered_map<unsigned long long, Edge> edges;
    std::vector<Node*> nodePool;

    // --- 线程局部容器结构 ---
    struct ThreadLocalData {
        std::vector<Node*> nodePool; // 负责该线程分配的 Node 内存管理
        std::unordered_map<unsigned long long, Edge> edges; // 该线程负责的边
        
        ~ThreadLocalData() {
            for (Node* n : nodePool) delete n;
        }
    };

    std::vector<std::unique_ptr<ThreadLocalData>> threadDataSlots;
    int max_threads;

    inline ThreadLocalData& getLocalData() {
        int tid = omp_get_thread_num();
        return *threadDataSlots[tid];
    }

    // 添加一个映射:树根 -> 连通分量ID
    std::unordered_map<Node*, int> rootToComponentId;
    int nextComponentId = 0;
    mutable std::shared_mutex mtx;

    // 辅助函数：将边(u,v)编码为唯一键
    inline unsigned long long makeEdgeKey(int u, int v) const {
        if (u > v) std::swap(u, v);
        return (static_cast<unsigned long long>(u) << 32) | static_cast<unsigned long long>(v);
    }
    
    inline void pushDown(Node* x) {
        if (!x || !x->reversed) return;
        std::swap(x->left, x->right);
        if (x->left) x->left->reversed ^= true;
        if (x->right) x->right->reversed ^= true;
        x->reversed = false;
    }
    
    void rotate(Node* x) {
        Node* p = x->parent;
        Node* g = p->parent;
        
        pushDown(p);
        pushDown(x);
        
        if (p->left == x) {
            p->left = x->right;
            if (x->right) x->right->parent = p;
            x->right = p;
        } else {
            p->right = x->left;
            if (x->left) x->left->parent = p;
            x->left = p;
        }
        
        x->parent = g;
        p->parent = x;
        
        if (g) {
            if (g->left == p) g->left = x;
            else g->right = x;
        }
    }
    
    void splay(Node* x) {
        while (x->parent) {
            Node* p = x->parent;
            Node* g = p->parent;
            
            if (g) pushDown(g);
            pushDown(p);
            pushDown(x);
            
            if (!g) {
                rotate(x);
            } else if ((g->left == p) == (p->left == x)) {
                rotate(p);
                rotate(x);
            } else {
                rotate(x);
                rotate(x);
            }
        }
        pushDown(x);
    }
    
    Node* split(Node* x) {
        splay(x);
        pushDown(x);
        Node* right = x->right;
        if (right) {
            right->parent = nullptr;
            x->right = nullptr;
        }
        return right;
    }
    
    Node* merge(Node* left, Node* right) {
        if (!left) return right;
        if (!right) return left;
        
        Node* curr = left;
        while (curr->right) {
            pushDown(curr);
            curr = curr->right;
        }
        splay(curr);
        pushDown(curr);
        
        curr->right = right;
        right->parent = curr;
        return curr;
    }
    
    inline Node* allocNode(int v = -1, unsigned long long key = 0) {
        auto& local = getLocalData();
        
        Node* node = new Node(v, key);

        static std::atomic<size_t> global_counter{0};
        node->node_id = global_counter.fetch_add(1, std::memory_order_relaxed);
        
        local.nodePool.push_back(node);
        return node;
    }

    // 获取或创建连通分量ID
    int getOrCreateComponentId(Node* root) {
        // std::unique_lock<std::shared_mutex> lock(mutex_);
        auto it = rootToComponentId.find(root);
        if (it != rootToComponentId.end()) {
            return it->second;
        }
        
        // 新连通分量,分配ID
        int compId = nextComponentId++;
        rootToComponentId[root] = compId;
        return compId;
    }
    
    // 更新rootToComponentId映射
    void updateComponentMapping(Node* oldRoot1, Node* oldRoot2, Node* newRoot) {
        // std::unique_lock<std::shared_mutex> lock(mutex_);
        int compId1 = -1, compId2 = -1;
        
        auto it1 = rootToComponentId.find(oldRoot1);
        if (it1 != rootToComponentId.end()) {
            compId1 = it1->second;
            rootToComponentId.erase(it1);
        }
        
        auto it2 = rootToComponentId.find(oldRoot2);
        if (it2 != rootToComponentId.end()) {
            compId2 = it2->second;
            rootToComponentId.erase(it2);
        }
        
        // 使用较小的ID
        int finalId = (compId1 != -1 && compId2 != -1) ? 
                      std::min(compId1, compId2) : 
                      (compId1 != -1 ? compId1 : compId2);
        
        if (finalId == -1) {
            finalId = nextComponentId++;
        }
        
        rootToComponentId[newRoot] = finalId;
    }

    void debugDfs(Node* x) {
        if (!x) return;
        pushDown(x);
        debugDfs(x->left);
        if (x->vertex != -1) 
            std::cout << x->vertex << " ";
        else {
            auto it = edges.find(x->edgeKey);
            if (it != edges.end()) {
                std::cout << "(" << it->second.u << "," << it->second.v << ") ";
            }
        };
        debugDfs(x->right);
    }

    inline Node* getRoot(Node* x) {
        while (x->parent) x = x->parent;
        return x;
    }
    
public:
    explicit SplayETT(int n, int num_threads = 32) {
        vertexNodes.resize(n);
        // nodePool.reserve(n * 4);
        max_threads = num_threads;
        threadDataSlots.resize(max_threads);
        for (int i = 0; i < max_threads; ++i) {
            threadDataSlots[i] = std::make_unique<ThreadLocalData>();
        }
        
        for (int i = 0; i < n; i++) {
            vertexNodes[i] = allocNode(i);
        }
    }
    
    ~SplayETT() {
        // 释放所有节点内存
        for (auto& slot : threadDataSlots) {
            slot.reset();
        }
    }

    void debugPrintEuler(int u) {
        Node* root = getRoot(vertexNodes[u]);
        splay(root);
        debugDfs(root);
        std::cout << "\n";
    }
    
    /**
     * 连接两个顶点 - 返回void以提升性能
     * @param u 顶点u
     * @param v 顶点v
     * @return 是否成功连接（false表示已连通）
     */
    bool link(int u, int v) {
        if (u == v) return false;

        auto& local = getLocalData();
        unsigned long long key = makeEdgeKey(u, v);
        
        if (local.edges.count(key)) return false;

        if (connected(u, v)) {
            return false;
        }
        
        Node* e1 = allocNode(-1, key);
        Node* e2 = allocNode(-1, key);

        Node* rootU = getRoot(vertexNodes[u]);
        Node* rootV = getRoot(vertexNodes[v]);
        
        Node* rightU = split(vertexNodes[u]);
        
        Node* newRoot = merge(vertexNodes[u], e1);
        newRoot = merge(newRoot, rootV);
        newRoot = merge(newRoot, e2);
        newRoot = merge(newRoot, rightU);

        local.edges[key] = {e1, e2, u, v};
        return true;
    }
    
    /**
     * 删除一条边 - 直接通过顶点对删除
     * @param u 顶点u
     * @param v 顶点v
     * @return 是否成功删除
     */
    bool cut(int u, int v) {
        if (u == v) return false;

        auto& local = getLocalData();
        unsigned long long key = makeEdgeKey(u, v);
        
        auto it = local.edges.find(key);
        if (it == local.edges.end()) return false;
        
        Edge& edge = it->second;
        Node* e1 = edge.first;
        Node* e2 = edge.second;

        splay(e1);
        splay(e2);

        if (e2->parent) {
            std::swap(e1, e2);
            splay(e1);
        }
        
        Node* right = split(e2);
        Node* middle = split(e1);
        
        Node* left = e1->left;
        if (left) left->parent = nullptr;
        
        merge(left, right);
        // Node* newRoot2 = middle;

        local.edges.erase(it);
        return true;
    }

    inline bool connected(int u, int v) {
        return getRoot(vertexNodes[u]) == getRoot(vertexNodes[v]);
    }

    /**
     * 获取节点所在连通分量根节点节点id
     */
    int getComponentId(int u) {        
        Node* root = getRoot(vertexNodes[u]);
        return root->node_id;
    }

    bool isTreeEdge(int u, int v) {
        auto& local = getLocalData();
        unsigned long long key = makeEdgeKey(u, v);
        return local.edges.find(key) != local.edges.end();
    }

    void batchLink(const std::vector<std::pair<int, int>>& edges_to_add) {
        for (const auto& [u, v] : edges_to_add) {
            link(u, v);
        }
    }

    /**
     * 获取连通分量大小 - 可选功能
     */
    int componentSize(int u) {
        Node* root = getRoot(vertexNodes[u]);
        splay(root);
        
        int size = 0;
        std::function<void(Node*)> dfs = [&](Node* x) {
            if (!x) return;
            pushDown(x);
            dfs(x->left);
            if (x->vertex != -1) size++;
            dfs(x->right);
        };
        dfs(root);
        
        return size;
    }
    
};

#endif // SPLAY_ETT_H