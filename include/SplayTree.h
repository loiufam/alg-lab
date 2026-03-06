#ifndef SPLAYTREE_H
#define SPLAYTREE_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <algorithm>

namespace splaytree {

// Splay节点：表示顶点在欧拉回路中的出现
struct Node {
    int u;                         // 起始顶点
    int v;                         // 结束顶点（-1表示单个顶点节点）
    Node *left, *right, *parent;
    int size;
    
    // 构造单个顶点节点
    Node(int vertex) : u(vertex), v(-1), left(nullptr), 
          right(nullptr), parent(nullptr), size(1) {}
    
    // 构造边节点
    Node(int _u, int _v) : u(_u), v(_v), left(nullptr), 
          right(nullptr), parent(nullptr), size(1) {}
    
    bool isEdge() const { return v != -1; }
};

// 边的表示（标准化：u < v）
struct Edge {
    int u, v;
    Edge(int _u, int _v) : u(std::min(_u, _v)), v(std::max(_u, _v)) {}
    
    bool operator==(const Edge& other) const {
        return u == other.u && v == other.v;
    }
    
    bool operator<(const Edge& other) const {
        if (u != other.u) return u < other.u;
        return v < other.v;
    }
};

// 边的哈希函数
struct EdgeHash {
    size_t operator()(const Edge& e) const {
        return std::hash<long long>()((long long)e.u << 32 | e.v);
    }
};

// struct EdgeOrder {
//     Node* first;   // 在欧拉序列中先出现的边节点
//     Node* second;  // 在欧拉序列中后出现的边节点
    
//     EdgeOrder() : first(nullptr), second(nullptr) {}
//     EdgeOrder(Node* f, Node* s) : first(f), second(s) {}
// };

// 欧拉回路树类（无锁版本，每个线程独立处理）
class EulerTourTree {

public:
    Node* root;
    int treeId;
    int  compId   = -1;
    
    // 连通分量信息
    std::unordered_set<int> vertices;                          // 顶点集合
    std::unordered_set<Edge, EdgeHash> nonTreeEdges;           // 非树边集合
    
    // 每个顶点的occurrence节点列表（用于O(1)查找）
    // std::unordered_map<int, std::vector<Node*>> vertexOccurrences;
    // 存储所有树边节点，便于快速查找和更新
    std::unordered_map<int, std::unordered_map<int, Node*>> edgeNodes;

    // std::unordered_map<Edge, EdgeOrder, EdgeHash> edgeOrder;
    
    // Splay树基本操作
    void updateSize(Node* x);
    void rotate(Node* n);
    void splay(Node* n);
    void updateSizeToRoot(Node* n);

    Node* findMax(Node* x);
    Node* findMin(Node* x);
    Node* findRoot(Node* x);

    Node* findRightmost(Node* x);
    Node* concatenate(Node* leftTree, Node* rightTree);
    void mergeTrees(EulerTourTree* otherTree);

    Node* split(Node* x);
    Node* join(Node* leftTree, Node* rightTree);
    
    // 辅助函数
    inline int getSize(Node* x) const {
        return x ? x->size : 0;
    }
    Node* getRepresentative(int u);
    Node* findEdgeNode(Node* subtree, int u, int v);
    Node* findEdgeNodeDFS(Node* subtree, int u, int v);
    Node* buildFromNodes(std::vector<Node*>& nodes, int start, int end);
    void deleteTree(Node* x);
    
    // 寻找替代边
    Edge findReplacementEdge(Node* rootU, Node* rootV);
    
public:
    explicit EulerTourTree(int id);
    ~EulerTourTree();

    // 禁止拷贝，只能移动
    // EulerTourTree(const EulerTourTree&) = delete;
    // EulerTourTree& operator=(const EulerTourTree&) = delete;
    EulerTourTree(const EulerTourTree& o)
    : treeId(o.treeId), compId(o.compId), root(o.root),
        vertices(o.vertices), nonTreeEdges(o.nonTreeEdges), edgeNodes(o.edgeNodes) {}
    
    // 基本操作
    int getTreeId() const { return treeId; }
    Node* getRoot() const { return root; }
    void collectNodes(Node* x, std::vector<Node*>& nodes) const;
    int getAnyVertex() const {
        return vertices.empty() ? -1 : *vertices.begin();
    }

    void addVertex(int v);
    void reroot(int u);
    void link(int u, int v, EulerTourTree* otherTree);
    Node* joinTreesViaEdge(Node* root1, Node* root2, int u, int v);
    std::pair<std::unique_ptr<EulerTourTree>, std::unique_ptr<EulerTourTree>> cut(int u, int v, int newTreeIdU, int newTreeIdV);
    
    // 查询操作
    bool isConnected(int u, int v) const;
    void setVertices(const std::unordered_set<int>& verts) { vertices = verts; }
    std::unordered_set<int> getVertices() const { return vertices; }
    void setNonTreeEdges(const std::unordered_set<Edge, EdgeHash>& edges) { nonTreeEdges = edges; }
    std::unordered_set<Edge, EdgeHash> getNonTreeEdges() const { return nonTreeEdges; }
    bool isEmpty() const { return vertices.empty(); }
    bool hasVertex(int v) const { return vertices.count(v) > 0; }
   
    // 获取顶点的代表节点（用于快速更新 vertexToComponent）
    Node* getVertexRepresentative(int v) const {
        auto it = edgeNodes.find(v);
        if (it != edgeNodes.end() && it->second.count(v)) {
            return it->second.at(v);
        }
        return nullptr;
    }

    inline bool containsVertex(int v) const {
        return vertices.count(v) > 0;
    }

    // 快速获取边界顶点的树边邻居（如果存在）
    inline int getBoundaryVertexTreeNeighbor(int v) const {
        auto it = edgeNodes.find(v);
        if (it != edgeNodes.end()) {
            for (const auto& [neighbor, node] : it->second) {
                if (neighbor != v) {
                    return neighbor; // 返回唯一的树边邻居
                }
            }
        }
        return -1;
    }

    // 非树边操作
    void addNonTreeEdge(const Edge& e) { nonTreeEdges.insert(e); }
    void removeNonTreeEdge(const Edge& e) { nonTreeEdges.erase(e); }
    bool hasNonTreeEdge(const Edge& e) const { return nonTreeEdges.count(e) > 0; }
    bool isTreeEdge(int u, int v) const {
        if (u > v) std::swap(u, v);
        return edgeNodes.count(u) && edgeNodes.at(u).count(v);
    }
    
    // 动态更新相关
    void deleteOccurrence(Node* node);
    std::pair<Node*, Node*> deleteEdge(int u, int v);
    void insertEdge(int u, int v);
    void removeVertex(int v);
    void cutBoundary(int v, int u);
    std::unique_ptr<EulerTourTree> cutWithReplacement(int u, int v);

    // 调试
    void printEulerTour() const;
    void testSplay(int v);
    int getVertexDegree(int v) const;
};

}
#endif // SPLAYTREE_H