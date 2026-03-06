#include "../include/SplayTree.h"
#include <iostream>
#include <queue>

namespace splaytree {

// 构造和析构
EulerTourTree::EulerTourTree(int id) : root(nullptr), treeId(id) {}

EulerTourTree::~EulerTourTree() {
    deleteTree(root);
}

void EulerTourTree::deleteTree(Node* x) {
    if (!x) return;
    deleteTree(x->left);
    deleteTree(x->right);
    delete x;
}

// Splay树基本操作
void EulerTourTree::updateSize(Node* x) {
    if (!x) return;
    x->size = 1;
    if (x->left) x->size += x->left->size;
    if (x->right) x->size += x->right->size;
}

void EulerTourTree::rotate(Node* n) {
    Node* p = n->parent;
    if (!p) return;

    Node* g = p->parent;

    if (n == p->left) {
        // Right rotation
        p->left = n->right;
        if (n->right) {
            n->right->parent = p;
        }
        n->right = p;
    } else {
        // Left rotation
        p->right = n->left;
        if (n->left) {
            n->left->parent = p;
        }
        n->left = p;
    }

    p->parent = n;
    n->parent = g;

    if (g) {
        if (p == g->left) g->left = n;
        else g->right = n;
    } else {
        root = n;
    }

    updateSize(p);
    updateSize(n);
}

// Splay操作：将节点n旋转到根位置
void EulerTourTree::splay(Node* n) {
    if (!n) return;

    while (n->parent) {
        Node* p = n->parent;
        Node* g = p->parent;

        if (g) {
            if ((n == p->left && p == g->left) ||
                 (n == p->right && p == g->right)) {
                rotate(p);
            } else {
                rotate(n);
            }
        }
        rotate(n);
    }
    root = n;
}

void EulerTourTree::updateSizeToRoot(Node* n) {
    while (n) {
        updateSize(n);
        n = n->parent;
    }
}


Node* EulerTourTree::findMax(Node* x) {
    if (!x) return nullptr;
    while (x->right) x = x->right;
    splay(x);
    return x;
}

Node* EulerTourTree::findMin(Node* x) {
    if (!x) return nullptr;
    while (x->left) x = x->left;
    splay(x);
    return x;
}

Node* EulerTourTree::findRoot(Node* x) {
    if (!x) return nullptr;
    while (x->parent) x = x->parent;
    return x;
}

Node* EulerTourTree::split(Node* x) {
    if (!x) return nullptr;
    splay(x);

    Node* rightTree = x->right;
    if (rightTree) {
        rightTree->parent = nullptr;
        x->right = nullptr;
        updateSize(x);
    }
    return rightTree;
}

// Euler Tour Cancatenate
Node* EulerTourTree::join(Node* leftTree, Node* rightTree) {
    if (!leftTree) {
        return rightTree;
    }
    if (!rightTree) {
        return leftTree;
    }

    Node* maxNode = findMax(leftTree);
    maxNode->right = rightTree;
    rightTree->parent = maxNode;
    updateSize(maxNode);
    return maxNode;
}

Node* EulerTourTree::findRightmost(Node* x) {
    if (!x) return nullptr;
    while (x->right) x = x->right;
    return x;
}

Node* EulerTourTree::concatenate(Node* leftTree, Node* rightTree) {
    if (!leftTree) return rightTree;
    if (!rightTree) return leftTree;

    Node* r = findRightmost(leftTree);
    r->right = rightTree;
    rightTree->parent = r;
    // updateSizeToRoot(rightTree);
    return leftTree;
}

void EulerTourTree::mergeTrees(EulerTourTree* other) {
    vertices.insert(other->vertices.begin(), other->vertices.end());
    nonTreeEdges.insert(other->nonTreeEdges.begin(), other->nonTreeEdges.end());

    for (auto& [v, edges] : other->edgeNodes) {
        for (auto& [u, node] : edges) {
            edgeNodes[v][u] = node;
        }
    }

    other->root = nullptr;
    other->vertices.clear();
    other->nonTreeEdges.clear();
    other->edgeNodes.clear();
}


Node* EulerTourTree::getRepresentative(int u) {
    // 返回顶点 u 的代表节点
    if (edgeNodes.count(u) && edgeNodes[u].count(u)) {
        return edgeNodes[u][u];
    }
    return nullptr;
}

// 中序遍历得到欧拉序列
void EulerTourTree::collectNodes(Node* x, std::vector<Node*>& nodes) const {
    if (!x) return;
    collectNodes(x->left, nodes);
    nodes.push_back(x);
    collectNodes(x->right, nodes);
}

Node* EulerTourTree::buildFromNodes(std::vector<Node*>& nodes, int start, int end) {
    if (start > end) return nullptr;
    int mid = (start + end) / 2;
    Node* node = nodes[mid];
    node->left = buildFromNodes(nodes, start, mid - 1);
    node->right = buildFromNodes(nodes, mid + 1, end);
    node->parent = nullptr;
    if (node->left) node->left->parent = node;
    if (node->right) node->right->parent = node;
    updateSize(node);
    return node;
}

// 添加单个顶点
void EulerTourTree::addVertex(int v) {
    if (vertices.count(v)) return;
    
    vertices.insert(v);
    Node* node = new Node(v);
    edgeNodes[v][v] = node;
    root = node;
}

void EulerTourTree::reroot(int u) {
    Node* node = getRepresentative(u);
    if (!node) return; // u 不在本树中

    splay(node);
    // 此时树结构： [L] node [R]
    // 欧拉序： ... L ... node ... R ...
    // 目标序： node ... R ... L ... 
    
    Node* leftPart = node->left;
    if (leftPart) {
        leftPart->parent = nullptr;
        node->left = nullptr;
        updateSize(node);
        
        root = join(node, leftPart);
    }
}

// Link操作：连接两棵不同树 (u in this tree, v in otherTree)
void EulerTourTree::link(int u, int v, EulerTourTree* otherTree) {
    if (!otherTree || otherTree == this) {
        // std::cout << "Error: Cannot link to null or the same tree.\n";
        return;
    }
    
    // std::cout << "Before linking edge (" << u << ", " << v << "):\n";
    // printEulerTour();
    
    reroot(u);
    otherTree->reroot(v);
    
    Node* edgeUV = new Node(u, v);
    Node* edgeVU = new Node(v, u);
    
    edgeNodes[u][v] = edgeUV;
    edgeNodes[v][u] = edgeVU;

    // 保存当前的 root
    Node* tree1Root = root;
    Node* tree2Root = otherTree->root;
    
    // 合并: root -> edgeUV -> otherTree->root -> edgeVU
    tree1Root = concatenate(tree1Root, edgeUV);
    tree1Root = concatenate(tree1Root, tree2Root);
    tree1Root = concatenate(tree1Root, edgeVU);

    updateSizeToRoot(edgeVU);
    root = findRoot(edgeVU);

    
    // 合并数据
    mergeTrees(otherTree);

    // std::cout << "After linking edge (" << u << ", " << v << "):\n";
    // printEulerTour();
}

Node* EulerTourTree::joinTreesViaEdge(Node* root1, Node* root2, int u, int v) {
    reroot(u);
    reroot(v);
   
    root1 = findRoot(getRepresentative(u));
    root2 = findRoot(getRepresentative(v));

    Node* edgeUV = new Node(u, v);
    Node* edgeVU = new Node(v, u);
    
    edgeNodes[u][v] = edgeUV;
    edgeNodes[v][u] = edgeVU;
    
    // 合并: root1 -> edgeUV -> root2 -> edgeVU
    Node* newRoot = concatenate(root1, edgeUV);
    newRoot = concatenate(newRoot, root2);
    newRoot = concatenate(newRoot, edgeVU);

    updateSizeToRoot(edgeVU);
    return findRoot(edgeVU);
}

Node* EulerTourTree::findEdgeNode(Node* subtree, int u, int v) {
    if (!subtree) return nullptr;
    
    // 使用队列进行BFS查找
    std::queue<Node*> q;
    q.push(subtree);
    
    while (!q.empty()) {
        Node* current = q.front();
        q.pop();
        
        // 检查当前节点是否是目标边节点
        if (current->isEdge() && current->u == u && current->v == v) {
            return current;
        }
        
        if (current->left) q.push(current->left);
        if (current->right) q.push(current->right);
    }
    
    return nullptr;
}

// 更高效的版本：使用DFS
Node* EulerTourTree::findEdgeNodeDFS(Node* subtree, int u, int v) {
    if (!subtree) return nullptr;
    
    // 先检查当前节点
    if (subtree->isEdge() && subtree->u == u && subtree->v == v) {
        return subtree;
    }
    
    // 递归查找左子树
    Node* leftResult = findEdgeNodeDFS(subtree->left, u, v);
    if (leftResult) return leftResult;
    
    // 递归查找右子树
    return findEdgeNodeDFS(subtree->right, u, v);
}

// Cut操作：删除边u-v
std::pair<std::unique_ptr<EulerTourTree>, std::unique_ptr<EulerTourTree>> EulerTourTree::cut(int u, int v, int newTreeIdU, int newTreeIdV) {
    
    reroot(u);
    
    if (!edgeNodes.count(u) || !edgeNodes[u].count(v)) {
        // std::cerr << "Error: edge not found.\n";
        return {nullptr, nullptr};
    }

    Node* edgeUV = edgeNodes[u][v];
    Node* edgeVU = edgeNodes[v][u];

    splay(edgeUV);
    // 此时结构：[L(u的其它边)] uv [R(v的子树 + vu + u的其它边)]
    Node* L = edgeUV->left;
    Node* R = edgeUV->right;
    
    edgeUV->left = nullptr;
    edgeUV->right = nullptr;
    if (L) L->parent = nullptr;
    if (R) R->parent = nullptr;

    splay(edgeVU);
    // 此时结构：[Mid(v的子树)] vu [R2(u的剩余部分)]
    Node* T_v = edgeVU->left;
    Node* R2 = edgeVU->right;

    if (T_v) T_v->parent = nullptr;
    if (R2) R2->parent = nullptr;

    // 重新连接 L 和 R2，形成 T_u
    Node* T_u = join(L, R2);

    delete edgeUV;
    delete edgeVU;
    
    edgeNodes[u].erase(v);
    edgeNodes[v].erase(u);
    if (edgeNodes[u].empty()) edgeNodes.erase(u);
    if (edgeNodes[v].empty()) edgeNodes.erase(v);
    
    auto treeU = std::make_unique<EulerTourTree>(newTreeIdU);
    auto treeV = std::make_unique<EulerTourTree>(newTreeIdV);

    treeU->root = T_u;
    treeV->root = T_v;

    // 后续的操作不需要在每次Cut进行，应该推迟到所有Cut完成后统一处理
    EulerTourTree* T_small;
    EulerTourTree* T_large;

    if ((T_u ? T_u->size : 0) <= (T_v ? T_v->size : 0)) {
        T_small = treeU.get();
        T_large = treeV.get();
    } else {
        T_small = treeV.get();
        T_large = treeU.get();
    }

    // 分配顶点
    std::vector<Node*> nodes;
    T_small->collectNodes(T_small->root, nodes);
    auto oldEdgeNodes = std::move(edgeNodes);
    
    for (Node* node : nodes) {
        T_small->vertices.insert(node->u);
        vertices.erase(node->u);

        if (node->isEdge()) {
            T_small->vertices.insert(node->v);
            vertices.erase(node->v);

            T_small->edgeNodes[node->u][node->v] = node;
            oldEdgeNodes[node->u].erase(node->v);
            if (oldEdgeNodes[node->u].empty())
                oldEdgeNodes.erase(node->u);
        } else {
            T_small->edgeNodes[node->u][node->u] = node;
            oldEdgeNodes[node->u].erase(node->u);
            if (oldEdgeNodes[node->u].empty())
                oldEdgeNodes.erase(node->u);
        }
    }

    T_large->vertices = vertices;
    T_large->edgeNodes = std::move(oldEdgeNodes);

    for (const Edge& e : nonTreeEdges) {
        bool inSmallU = T_small->vertices.count(e.u);
        bool inSmallV = T_small->vertices.count(e.v);

        if (inSmallU && inSmallV) {
            T_small->nonTreeEdges.insert(e);
        } else {
            T_large->nonTreeEdges.insert(e);
        }
    }
    
    root = nullptr;
    vertices.clear();
    nonTreeEdges.clear();
    edgeNodes.clear();
    
    return {std::move(treeU), std::move(treeV)};
}

void EulerTourTree::insertEdge(int u, int v) {
    if (!vertices.count(u) || !vertices.count(v)) return;
    
    reroot(u);
    
    Node* nodeV = getRepresentative(v);
    if (!nodeV) return;
    
    splay(nodeV);
    Node* left = nodeV->left;
    Node* right = nodeV->right;
    
    if (left) left->parent = nullptr;
    if (right) right->parent = nullptr;
    nodeV->left = nodeV->right = nullptr;
    updateSize(nodeV);
    
    // 创建边节点
    Node* edgeUV = new Node(u, v);
    Node* edgeVU = new Node(v, u);
    edgeNodes[u][v] = edgeUV;
    edgeNodes[v][u] = edgeVU;
    
    // 组装：left -> (u,v) -> [v] -> right -> (v,u)
    root = left;
    root = concatenate(root, edgeUV);
    root = concatenate(root, nodeV);
    root = concatenate(root, right);
    root = concatenate(root, edgeVU);
    
    updateSizeToRoot(edgeVU);
    root = findRoot(edgeVU);
}

std::pair<Node*, Node*> EulerTourTree::deleteEdge(int u, int v) {
    if (!edgeNodes.count(u) || !edgeNodes[u].count(v)) {
        return {nullptr, nullptr};
    }
    
    Node* edgeUV = edgeNodes[u][v];
    Node* edgeVU = edgeNodes[v][u];
    
    // 删除 (u,v) 节点，返回分离的两部分
    splay(edgeUV);
    Node* L1 = edgeUV->left;
    Node* R1 = edgeUV->right;
    if (L1) L1->parent = nullptr;
    if (R1) R1->parent = nullptr;
    edgeUV->left = edgeUV->right = nullptr;
    
    // 在剩余部分中删除 (v,u) 节点
    if (!R1) return {L1, nullptr};
    
    splay(edgeVU);
    Node* L2 = edgeVU->left;
    Node* R2 = edgeVU->right;
    if (L2) L2->parent = nullptr;
    if (R2) R2->parent = nullptr;
    edgeVU->left = edgeVU->right = nullptr;
    
    // 合并产生两棵子树
    Node* treeU = join(L1, R2);  // u 侧的树
    Node* treeV = L2;             // v 侧的树
    
    // 删除边节点
    delete edgeUV;
    delete edgeVU;
    edgeNodes[u].erase(v);
    edgeNodes[v].erase(u);
    
    return {treeU, treeV};
}

// 寻找替代边
Edge EulerTourTree::findReplacementEdge(Node* rootU, Node* rootV) {

    for (const Edge& e : nonTreeEdges) {
        Node* nodeU = getRepresentative(e.u);
        Node* nodeV = getRepresentative(e.v);

        if (!nodeU || !nodeV) continue;
    
        Node* rU = findRoot(nodeU);
        Node* rV = findRoot(nodeV);
        
        if ((rU == rootU && rV == rootV) || (rU == rootV && rV == rootU)) {
            return e;
        }
    }
    
    return Edge(-1, -1);
}

// 带替换的Cut操作
std::unique_ptr<EulerTourTree> EulerTourTree::cutWithReplacement(int u, int v) {
    reroot(u); 

    auto [treeU, treeV] = deleteEdge(u, v);
    if (!treeU || !treeV) {
        // 边不存在或删除失败
        return nullptr;
    }

    root = treeU;  // 当前对象保留为 u 所在的树

    Edge replacement = findReplacementEdge(treeU, treeV);

    if (replacement.u != -1) {
        nonTreeEdges.erase(replacement);

        root = joinTreesViaEdge(treeU, treeV, replacement.u, replacement.v);
        return nullptr; // 没有产生新分量
    } else {
        auto newTree = std::make_unique<EulerTourTree>(-1);

        Node* T_small = (getSize(treeU) < getSize(treeV)) ? treeU : treeV;
        Node* T_large = (T_small == treeU) ? treeV : treeU;

        // 分配顶点
        std::vector<Node*> nodes;
        collectNodes(T_small, nodes);
        
        // 遍历 small 树的节点，将其移出
        for (Node* node : nodes) {
            int currU = node->u;
            int currV = node->v;
        
            // 移动 Vertices
            if (vertices.count(currU)) {
                vertices.erase(currU);
                newTree->vertices.insert(currU);
            }
            if (currV != -1 && vertices.count(currV)) { // 处理边的另一端
                 vertices.erase(currV);
                 newTree->vertices.insert(currV);
            }

            // 移动EdgeNodes
            if (currV != -1) {
                newTree->edgeNodes[currU][currV] = node;
                edgeNodes[currU].erase(currV);
                if (edgeNodes[currU].empty()) edgeNodes.erase(currU);
            } else {
                newTree->edgeNodes[currU][currU] = node;
                edgeNodes[currU].erase(currU);
                if (edgeNodes[currU].empty()) edgeNodes.erase(currU);
            }
        }

        auto it = nonTreeEdges.begin();
        while (it != nonTreeEdges.end()) {
            bool inSmallU = newTree->vertices.count(it->u);
            bool inSmallV = newTree->vertices.count(it->v);

            if (inSmallU && inSmallV) {
                newTree->nonTreeEdges.insert(*it);
                it = nonTreeEdges.erase(it); // 安全删除并移动迭代器
            } else {
                ++it;
            }
        }

        newTree->root = T_small;
        root = T_large;

        if (newTree->vertices.count(u)) {
            newTree->edgeNodes[u].erase(v);
            edgeNodes[v].erase(u);
        } else { 
            newTree->edgeNodes[v].erase(u);
            edgeNodes[u].erase(v);
        }

        return newTree;
    }
}

// 快速切除边界边
void EulerTourTree::cutBoundary(int v, int u) {
    reroot(v);

    // 确保边存在
    if (!edgeNodes.count(v) || !edgeNodes[v].count(u)) return;

    // 删除 (v,u)
    if (edgeNodes[v].count(u)) {
        Node* edgeVU = edgeNodes[v][u];
        deleteOccurrence(edgeVU);  // root 会自动更新
        edgeNodes[v].erase(u);
    }
    
    // 删除 (u,v)
    if (edgeNodes[u].count(v)) {
        Node* edgeUV = edgeNodes[u][v];
        deleteOccurrence(edgeUV);  // root 会自动更新
        edgeNodes[u].erase(v);
    }

    return;
}

// 查询操作
bool EulerTourTree::isConnected(int u, int v) const {
    return vertices.count(u) && vertices.count(v);
}

void EulerTourTree::deleteOccurrence(Node* node) {
    if (node) {
        splay(node);
        Node* left = node->left;
        Node* right = node->right;
        if (left) left->parent = nullptr;
        if (right) right->parent = nullptr;
        
        if (left && right) {
            root = join(left, right);
        } else if (left) {
            root = left;
        } else if (right) {
            root = right;
        } else {
            root = nullptr;
        }

        delete node;
    }
}

// 移除顶点
void EulerTourTree::removeVertex(int v) {
    if (!vertices.count(v)) return;
    
    // std::cout << "Removing vertex " << v << "\n";

    Node* nodeV = getRepresentative(v);
    if (nodeV) {
        deleteOccurrence(nodeV);
    }
    
    vertices.erase(v);
    edgeNodes.erase(v);
    
    // 删除相关的非树边 - 使用迭代器安全删除
    auto edgeIt = nonTreeEdges.begin();
    while (edgeIt != nonTreeEdges.end()) {
        if (edgeIt->u == v || edgeIt->v == v) {
            edgeIt = nonTreeEdges.erase(edgeIt);
        } else {
            ++edgeIt;
        }
    }
}

// 获取顶点度数（用于判断是否为边界顶点）
int EulerTourTree::getVertexDegree(int v) const {
    if (!vertices.count(v)) return 0;

    int degree = 0;
    // std::vector<int> neighbors;
    auto it = edgeNodes.find(v);
    if (it != edgeNodes.end()) {
        for (const auto& [neighbor, node] : it->second) {
            if (neighbor != v) {
                degree++;
                // neighbors.push_back(neighbor);
            }
        }
    }
    // std::cout << "Vertex " << v << " has " << degree << " tree edges: ";
    // for (int neighbor : neighbors) {
    //     std::cout << "( " << v << ", " << neighbor << " ) ";
    // }
    // std::cout << std::endl;

    return degree;
}

// 调试：打印欧拉回路
void EulerTourTree::printEulerTour() const {
    if (!root) {
        std::cout << "Tree " << treeId << " is empty.\n";
        return;
    }
    
    std::vector<Node*> nodes;
    collectNodes(root, nodes);
    
    // std::cout << "=== Euler Tour (Tree " << treeId << ") ===" << std::endl;
    // std::cout << "Total nodes: " << nodes.size() << std::endl;
    std::cout << "root id: (" << root->u << "," << root->v << ")" << std::endl;
    std::cout << "Size: " << (root ? root->size : 0) << std::endl;
    std::cout << "Vertices: { ";
    for (int v : vertices) {
        std::cout << v << " ";
    }
    std::cout << "}" << std::endl;
    
    std::cout << "Tour sequence: ";
    for (size_t i = 0; i < nodes.size(); ++i) {
        Node* node = nodes[i];
        if (node->isEdge()) {
            std::cout << "(" << node->u << "->" << node->v << ")";
        } else {
            std::cout << "[" << node->u << "]";
        }
        if (i < nodes.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;
    
    // 打印非树边
    if (!nonTreeEdges.empty()) {
        std::cout << "Non-tree edges: ";
        for (const Edge& e : nonTreeEdges) {
            std::cout << "(" << e.u << "," << e.v << ") ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "==============================" << std::endl;
}

void EulerTourTree::testSplay(int v) {  

    std::cout << "Before splaying vertex " << v << ":\n";
    printEulerTour();
    
    auto node = getRepresentative(v);
    splay(node);
    
    std::cout << "After splaying vertex " << v << ":\n";
    printEulerTour();
}

} // namespace splaytree