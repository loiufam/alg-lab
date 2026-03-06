#include "../include/DancingMatrix.h"
#include "DynamicGraph.cpp"
thread_local std::unique_ptr<DancingMatrix::ThreadLocalState> DancingMatrix::tlsState = nullptr;

//构造函数
DancingMatrix::DancingMatrix( int rows, int cols, int** matrix, bool verbose )  
    : ROWS(rows), COLS(cols), enableGraphSync(verbose) {
    EXIST_ROWS = rows;  
    ColIndex = std::make_unique<ColumnHeader[]>(cols + 1);  
    RowIndex = std::make_unique<RowNode[]>(rows);  
    // if (verbose) graph = make_unique<ConnectedGraph>(rows, cols);

    root = &ColIndex[0];  
    ColIndex[0].left = &ColIndex[COLS];  
    ColIndex[0].right = &ColIndex[1];  
    ColIndex[COLS].right = &ColIndex[0];  
    ColIndex[COLS].left = &ColIndex[COLS-1];  

    for( int i = 1; i < cols; i++ )  
    {  
        ColIndex[i].left = &ColIndex[i-1];  
        ColIndex[i].right = &ColIndex[i+1];  
    }  

    for ( int i = 0; i <= cols; i++ )  
    {  
        ColIndex[i].up = &ColIndex[i];  
        ColIndex[i].down = &ColIndex[i];  
        ColIndex[i].col = i;  
    }  
    ColIndex[0].down = &RowIndex[0];  
    
    dataNodes.reserve(rows * cols / 2);

    // graph = std::make_shared<ConnectedGraph>(ROWS, COLS);
    for( int i = 0; i < rows; i++ ){
        for( int j = 0; j < cols ; j++ ) {  
            if(matrix[i][j] == 1){
                insert(  i , j+1 );  //行数与原矩阵相同，而列数加1
                rowsSet.insert(i);
                colsSet.insert(j+1); 
            }
        }
    }

    InitBlock = Block(rowsSet, colsSet);
    // if(verbose) graph = make_unique<ConnectedGraph>(*this);
    std::cout<< "初始化舞蹈链完成." << endl;
}

// 从文件构造舞蹈链矩阵
DancingMatrix::DancingMatrix( const string& file_path, int from, bool useIg , bool useETT  ) 
    : useIG(useIg), useETT(useETT)
{
    ifstream file(file_path);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << file_path << endl;
        throw runtime_error("无法打开文件");
    }

    string line;
    getline(file, line);  // 读取第一行 

    int rows, cols;
    if( from == 1 ) {
        PreProccess::extractNM( line, cols, rows );
        getline(file, line); 
    } else {
        istringstream iss(line);
        iss >> cols >> rows;
    }

    ROWS = rows;
    COLS = cols;

    if (ROWS > MAX_ROW) {
        cerr << "矩阵行数过大，无法处理: " << ROWS << " 行." << endl;
        throw runtime_error("time out");
    }

    // cout << "处理矩阵维度: " << rows << " 行, " << cols << " 列." << endl;
    ColIndex = std::make_unique<ColumnHeader[]>(cols + 1);  
    RowIndex = std::make_unique<RowNode[]>(rows);  
    root = &ColIndex[0];  

    ColIndex[0].left = &ColIndex[COLS];  
    ColIndex[0].right = &ColIndex[1];  
    ColIndex[COLS].right = &ColIndex[0];  
    ColIndex[COLS].left = &ColIndex[COLS-1];  

    for( int i = 1; i < cols; i++ )  
    {  
        ColIndex[i].left = &ColIndex[i-1];  
        ColIndex[i].right = &ColIndex[i+1];  
    }  

    for ( int i = 0; i <= cols; i++ )  
    {  
        ColIndex[i].up = &ColIndex[i];  
        ColIndex[i].down = &ColIndex[i];  
        ColIndex[i].col = i;  
    }  
    ColIndex[0].down = &RowIndex[0]; 

    dataNodes.reserve(rows * cols / 2);
    int currentRow = 0;
    while (getline(file, line)) {
        if (line.empty()) continue; // 跳过空行
        istringstream iss(line);

        string token;
        if (from == 1 || from == 3) {
            iss >> token;
        } else if (from == 2) {
            iss >> token;
            iss >> token;
        }

        int currentCol; 
        // std::vector<int> rowCols;
        while(iss >> currentCol) {
            if (currentCol < 1 || currentCol > cols) {
                cerr << "无效的列索引: " << currentCol << " 在行 " << currentRow + 1 << endl;
                exit(1);
            }
            insert(currentRow, currentCol); // 插入节点
            col_to_rows[currentCol].push_back(currentRow);
            row_to_cols[currentRow].insert(currentCol);
            ONE_COUNT++; // 统计矩阵中1的个数
            rowsSet.insert(currentRow);
            colsSet.insert(currentCol); 
            // rowCols.push_back(currentCol);
        }

        currentRow++;
        
        if (currentRow >= rows) break; // 防止超过预期行数
    }

    InitBlock = Block(rowsSet, colsSet);

    if(useETT){
        // detector = make_unique<ComponentDetector>(ROWS, COLS); 
        // detector->Initialize(col_to_rows);
        graph = make_unique<Graph>(ROWS);
        initialize();
        
        cout << "ETT initialization complete." << endl;
    }

    if (useIg) {
        incrementalGraph = make_unique<IncrementalConnectedGraph>(rows);
        incrementalGraph->initialize(*this);
    }

    file.close();
}

DancingMatrix::~DancingMatrix() = default;

void DancingMatrix::initialize() {
    buildGraphFromMatrix();
    buildSpanningForest();
}

void DancingMatrix::initThreadLocalState(const Block& block, std::unique_ptr<splaytree::EulerTourTree> tree) {
    
    // 创建或重置线程局部状态
    if (!tlsState) {
        tlsState = std::make_unique<ThreadLocalState>();
    }  
    
    // 清空旧数据
    tlsState->components.clear();
    tlsState->subgraph  = nullptr; 
    tlsState->nextTreeId = 0;

    if (!tree) { 
        std::cout << "Warning: Initializing thread local state with an empty tree." << std::endl;
        tlsState->initialized = true; 
        return; 
    }
    
    int anyV = tree->getAnyVertex();

    tlsState->components.push_back(std::move(tree));
    
    if (anyV >= 0) {
        tlsState->subgraph = graph->subgraphOf(anyV);
    }
    
    tlsState->initialized = true;
}

void DancingMatrix::cleanupThreadLocalState() {
    if (tlsState) {
        tlsState.reset();
    }
}

std::unique_ptr<splaytree::EulerTourTree> DancingMatrix::deepCopyTree(
    splaytree::EulerTourTree* original) {
    
    if (!original) return nullptr;
    
    auto newTree = std::make_unique<splaytree::EulerTourTree>(
        original->getTreeId()
    );
    
    // 拷贝顶点集合
    newTree->setVertices(original->getVertices());
    
    // 拷贝非树边集合
    newTree->setNonTreeEdges(original->getNonTreeEdges());
    
    // 收集原树的所有节点
    std::vector<splaytree::Node*> originalNodes;
    original->collectNodes(original->getRoot(), originalNodes);
    
    if (originalNodes.empty()) {
        return newTree;
    }
    
    // 创建新节点的副本
    std::vector<splaytree::Node*> newNodes;
    newNodes.reserve(originalNodes.size());
    
    for (auto* node : originalNodes) {
        splaytree::Node* newNode;
        if (node->isEdge()) {
            newNode = new splaytree::Node(node->u, node->v);
            newTree->edgeNodes[node->u][node->v] = newNode;
        } else {
            newNode = new splaytree::Node(node->u);
            newTree->edgeNodes[node->u][node->u] = newNode;
        }
        newNodes.push_back(newNode);
    }
    
    // 重建平衡的 Splay 树
    newTree->root = newTree->buildFromNodes(newNodes, 0, newNodes.size() - 1);
    
    return newTree;
}

std::unique_ptr<Graph> DancingMatrix::deepCopyGraph(splaytree::EulerTourTree* tree) {
    
    auto newGraph = std::make_unique<Graph>(ROWS);
    if (!tree) return newGraph;
    
    const auto& nonTreeEdges = tree->getNonTreeEdges();
    for (const auto& edge : nonTreeEdges) {
        newGraph->addEdge(edge.u, edge.v);
    }

    std::vector<splaytree::Node*> nodes;
    tree->collectNodes(tree->getRoot(), nodes);
    
    for (auto* node : nodes) {
        if (node->isEdge()) { // isEdge() 意味着 node->v != -1
            newGraph->addEdge(node->u, node->v);
        }
    }
    
    return newGraph;
}

// 将矩阵的行映射为无向图的顶点，构建邻接表存储所有边
void DancingMatrix::buildGraphFromMatrix() {

    std::vector<std::pair<int, int>> temp_edges;

    for (const auto& [col, rows] : col_to_rows) {
        if (rows.size() <= 1) continue;

        for (size_t i = 0; i < rows.size(); ++i) {
            for (size_t j = i + 1; j < rows.size(); ++j) {
                int u = rows[i];
                int v = rows[j];
                
                // 确保 u < v，将无向边标准化
                if (u > v) std::swap(u, v);
                
                temp_edges.emplace_back(u, v);
            }
        }
    }

    std::sort(temp_edges.begin(), temp_edges.end());
    
    auto last = std::unique(temp_edges.begin(), temp_edges.end());
    
    temp_edges.erase(last, temp_edges.end());

    for (const auto& edge : temp_edges) {
        graph->addEdge(edge.first, edge.second);
    }

    // graph->printGraph();
}

std::vector<splaytree::Edge> DancingMatrix::bfsSpanningTree(int start, std::unordered_set<int>& visited, std::unordered_set<int>& comp_vertices) {
    std::vector<splaytree::Edge> treeEdges;
    std::queue<int> q;
    q.push(start);
    visited.insert(start);
    comp_vertices.insert(start);
    
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        
        for (int v : graph->getNeighbors(u)) {
            if (!visited.count(v)) {
                visited.insert(v);
                comp_vertices.insert(v);
                treeEdges.push_back(splaytree::Edge(u, v));
                q.push(v);
            }
        }
    }
    
    return treeEdges;
}

// 由无向图得到初始生成森林，基于生成树构建Euler Tour Trees
void DancingMatrix::buildSpanningForest() {
    std::unordered_set<int> visited;
    
    for (int i = 0; i < ROWS; i++) {
        if (visited.count(i)) continue;
        
        // BFS构建生成树,得到树边集合及分量顶点集合
        unordered_set<int> comp_vertices;
        std::vector<splaytree::Edge> treeEdges = bfsSpanningTree(i, visited, comp_vertices);

        int compId = nextTreeId;
        graph->registerComponent(compId, comp_vertices);
        
        // 为每个顶点创建单独的树
        std::unordered_map<int, std::unique_ptr<splaytree::EulerTourTree>> vertexTrees;
        // 映射：顶点 -> 当前所属树的代表顶点
        std::unordered_map<int, int> vertexToRepresentative;

        for (int v : comp_vertices) {
            auto vTree = std::make_unique<splaytree::EulerTourTree>(nextTreeId++);
            vTree->addVertex(v);
            vertexTrees[v] = std::move(vTree);
            vertexToRepresentative[v] = v;  // 初始时每个顶点代表自己
        }

        // 查找顶点当前所属树的代表顶点（带路径压缩）
        std::function<int(int)> findRepresentative = [&](int v) -> int {
            if (vertexToRepresentative[v] != v) {
                vertexToRepresentative[v] = findRepresentative(vertexToRepresentative[v]);
            }
            return vertexToRepresentative[v];
        };
        
        // 连接树边，逐步合并树
        for (const splaytree::Edge& e : treeEdges) {
            int repU = findRepresentative(e.u);
            int repV = findRepresentative(e.v);
            
            // if (repU == repV) {
            //     std::cout << "Warning: Edge (" << e.u << ", " << e.v 
            //               << ") forms a cycle, skipping.\n";
            //     continue;
            // }
            
            auto& uTree = vertexTrees[repU];
            auto& vTree = vertexTrees[repV];
            
            if (uTree && vTree) {
                // std::cout << "Linking edge (" << e.u << ", " << e.v << ")\n";
                uTree->link(e.u, e.v, vTree.get());
                vertexToRepresentative[repV] = repU;
                vertexTrees[repV].reset();
                // std::cout << "  Merged tree " << repV << " into tree " << repU << "\n";
            }
        }
        
        // 找到最终的树（未被reset的那个）
        splaytree::EulerTourTree* finalTree = nullptr;
        int finalRep = -1;

        for (auto& [v, tree] : vertexTrees) {
            if (tree) {
                finalTree = tree.get();
                finalRep = v;

                // 添加非树边
                std::unordered_set<splaytree::Edge, splaytree::EdgeHash> treeEdgeSet(treeEdges.begin(), treeEdges.end());
                
                for (int u : comp_vertices) {
                    for (int v : graph->getNeighbors(u)) {
                        // 只处理 u < v 的情况，避免 (u,v) 和 (v,u) 重复计算
                        // 确认 v 也在当前连通分量中
                        if (u < v && comp_vertices.count(v)) {
                            splaytree::Edge candidate(u, v);
                            
                            // 3. 如果这条边不是树边，那就是非树边
                            if (!treeEdgeSet.count(candidate)) {
                                tree->addNonTreeEdge(candidate);
                            }
                        }
                    }
                }
                // 更新vertexToComponent映射
                // for (int vertex : comp_vertices) {
                //     vertexToComponent[vertex] = tree.get();
                // }

                components.push_back(std::move(tree)); // 保存生成的树
                break;
            }
        }

        if (!finalTree) std::cerr << "Error: No final tree found for component starting at vertex " << i << "\n";   
    }
}

splaytree::EulerTourTree* DancingMatrix::findEulerTourTree(int v) {
    auto& comps = getComponents();
    
    for (auto& tree : comps) {
        if (tree && tree->containsVertex(v)) {
            return tree.get();
        }
    }
    return nullptr;
}

void DancingMatrix::processBoundaryVertex(int v, splaytree::EulerTourTree* tree, SubGraph* g){
    // 处理所有相邻边(包含非树边)
    std::vector<int> neighbors = g->neighbors(v);
    
    // 边界顶点：删除所有非树边，无需寻找替代边
    // std::cout << "Removing boundary vertex " << v << "\n";
    int treeEdgeNeighbor = tree->getBoundaryVertexTreeNeighbor(v);
   
    if (treeEdgeNeighbor < 0) {
        // 只需删除所有非树边（如果还有的话），然后移除顶点
        for (int u : neighbors) {
            splaytree::Edge e(v, u);
            if (tree->hasNonTreeEdge(e)) tree->removeNonTreeEdge(e);
            g->deleteEdge(v, u);
        }
        tree->removeVertex(v);
        return;
    }

    // 删除非树边
    for (int u : neighbors) {
        if (u != treeEdgeNeighbor) {
            splaytree::Edge e(v, u);
            tree->removeNonTreeEdge(e);
            g->deleteEdge(v, u);
        }
    }
    
    // 删除唯一的树边
    tree->cutBoundary(v, treeEdgeNeighbor);
    g->deleteEdge(v, treeEdgeNeighbor);
    
    // 删除顶点
    tree->removeVertex(v);
}

// 减量式更新单连通分量
void DancingMatrix::DecUpdateCC(const std::set<int>& deletedVertices) {

    if (!isGraphSyncEnabled()) return;
    if (deletedVertices.empty()) return;

    // std::cout << "DecUpdateCC: Deleting vertices: {";
    // for (int v : deletedVertices) {
    //     std::cout << " " << v;
    // }
    // std::cout << " }\n";

    auto& comps = getComponents();
    auto* g = getGraph();

    if (!g) {
        //主线程 / 串行模式下 getGraph() 返回 nullptr
        // std::cerr << "getGraph() returned nullptr in DecUpdateCC\n";
        return;
    }

    std::vector<int> boundaryVertices;
    std::vector<int> otherVertices;

    // 当前连通分量
    splaytree::EulerTourTree* current_tree = comps[0].get();
    if (!current_tree) return;

    for (int v : deletedVertices) {
        int treeEdgeCount = current_tree->getVertexDegree(v);

        if (treeEdgeCount == 1) {
            boundaryVertices.push_back(v);
        } else {
            otherVertices.push_back(v);
        }
    }

    // 先处理边界
    for (int v : boundaryVertices) {
        processBoundaryVertex(v, current_tree, g);
    }

    // 继续在处理剩余的非边界顶点
    for (int v : otherVertices) {

        // 有可能出现分裂
        splaytree::EulerTourTree* tree = findEulerTourTree(v);
        if (!tree) continue; 

        // 处理所有相邻边
        std::vector<int> neighbors = g->neighbors(v);
        int currentDegree = tree->getVertexDegree(v);

        if (currentDegree == 0) {
            // 已完全孤立，直接移除
            tree->removeVertex(v);
            continue;
        }
        // 原本非边界顶点，可能变成边界顶点
        if (currentDegree == 1) {
            // std::cout << "Vertex " << v << " became boundary vertex\n";
            
            processBoundaryVertex(v, tree, g);
            continue;
        }
        
        // 仍是非边界顶点
        // std::cout << "Removing non-boundary vertex " << v << "\n";
        
        // 收集树边邻居和非树边邻居
        std::vector<int> treeNeighbors;
        std::vector<int> nonTreeNeighbors;
        
        for (int u : neighbors) {
            if (tree->isTreeEdge(v, u)) {
                treeNeighbors.push_back(u);
            } else {
                nonTreeNeighbors.push_back(u);
            }
        }
        
        // 删除所有非树边
        for (int u : nonTreeNeighbors) {
            splaytree::Edge e(v, u);
            if (tree->hasNonTreeEdge(e)) {
                tree->removeNonTreeEdge(e);
            }
            g->deleteEdge(v, u);
        }

        // 删除所有树边
        for (int u : treeNeighbors) {

            if (!g->hasEdge(v, u)) continue;

            splaytree::EulerTourTree* currentTree = findEulerTourTree(v);
            if (!currentTree) {
                std::cout << "Warning: vertex " << v << " disappeared during edge removal\n";
                break;
            }
            
            // std::cout << "  Cutting edge (" << v << ", " << u << ")\n";
            
            auto newTree = currentTree->cutWithReplacement(v, u);
            if (newTree) {
                // std::cout << "    -> Split occurred, new component created\n";
                comps.push_back(std::move(newTree));
            } else {
                // std::cout << "    -> Reconnected with replacement edge\n";
            }
            
            g->deleteEdge(v, u);
            // printComponents();
        }

        // for (auto& comp : comps) {
        //     if (comp->containsVertex(v)) {
        //         comp->removeVertex(v);
        //     }
        // }
        splaytree::EulerTourTree* finalTree = findEulerTourTree(v);
        if (finalTree) {
            finalTree->removeVertex(v);
        }
    }
    
    // 更新连通分量
    comps.erase(
        std::remove_if(comps.begin(), comps.end(),
            [](const std::unique_ptr<splaytree::EulerTourTree>& t) { 
                return t->isEmpty(); 
            }),
        comps.end()
    );

}

void DancingMatrix::IncUpdateCC(const std::set<int>& restoredVertices) {
    if (!isGraphSyncEnabled()) return;
    if (restoredVertices.empty()) return;

    // std::cout << "IncUpdateCC: Restoring vertices: {";
    // for (int v : restoredVertices) {
    //     std::cout << " " << v;
    // }
    // std::cout << " }\n";
    auto& comps = getComponents();
    auto* g = getGraph();
    if (!g) {
        // std::cerr << "getGraph() returned nullptr in IncUpdateCC\n";
        return;
    }

    std::unordered_set<int> restoredSet(restoredVertices.begin(), restoredVertices.end());

    for (int v : restoredVertices) {
        auto newTree = std::make_unique<splaytree::EulerTourTree>(nextTreeId++);
        newTree->addVertex(v);
        comps.push_back(std::move(newTree));
    }
    
    for (int v : restoredVertices) {
        std::vector<int> neighbors = g->getAllNeighbors(v); 

        for (int u : neighbors) {
            splaytree::EulerTourTree* treeU = findEulerTourTree(u);
            if (!treeU) continue;

            if (restoredSet.count(u) && v > u) continue;

            if (!g->hasEdge(v, u)) {
                g->restoreEdge(v, u);
            }
            splaytree::EulerTourTree* treeV = findEulerTourTree(v); 
            if (!treeV) continue;

            if (treeU != treeV) {
            // std::cout << "Linking restored edge (" << v << ", " << u << ")\n";
                treeU->link(u, v, treeV);
            } else {
                // std::cout << "Adding non-tree edge (" << v << ", " << u << ")\n";
                treeU->addNonTreeEdge(splaytree::Edge(v, u));
            }
        }
    }
    
    comps.erase(
        std::remove_if(comps.begin(), comps.end(),
            [](const std::unique_ptr<splaytree::EulerTourTree>& t) { return t->isEmpty(); }),
        comps.end()
    );
}

std::vector<std::unordered_set<int>> DancingMatrix::getConnectedComponents() const {
    std::vector<std::unordered_set<int>> result;
    for (const auto& tree : components) {
        if (!tree->isEmpty()) {
            result.push_back(tree->getVertices());
        }
    }
    return result;
}

void DancingMatrix::printComponents() {
    auto& comps = getComponents();
    std::cout << "Number of connected components: " << comps.size() << "\n";
    int idx = 1;
    for (const auto& tree : comps) {
        if (!tree->isEmpty()) {
            std::cout << "Component " << idx++ << " (Tree ID: " << tree->getTreeId() << "): ";
            tree->printEulerTour();
            std::cout << "\n";
        }
    }
    std::cout << "============================" << std::endl;
}

void DancingMatrix::testDynamicUpdateCC(const std::vector<int>& deletedVertices) {
    std::cout << "Before Decremental Update:\n";
    printComponents();

    set<int> deleted_set(deletedVertices.begin(), deletedVertices.end());

    DecUpdateCC(deleted_set);

    std::cout << "After Decremental Update:\n";
    printComponents();

    IncUpdateCC(deleted_set);

    std::cout << "After Incremental Update:\n";
    printComponents();
}

void DancingMatrix::testCutEdge(int u, int v) {
    
    splaytree::EulerTourTree* tree = findEulerTourTree(u);
    if (!tree) {
        std::cout << "Error: Vertex " << u << " not found in any component.\n";
        return;
    }
    
    std::cout << "Before cutting edge (" << u << ", " << v << "):\n";
    tree->printEulerTour();
    
    int treeId1 = nextTreeId++;
    int treeId2 = nextTreeId++;

    // 接收返回值
    auto [newTree1, newTree2] = tree->cut(u, v, treeId1, treeId2);
    
    // 检查是否成功
    if (!newTree1 || !newTree2) {
        std::cout << "Error: Failed to cut edge (" << u << ", " << v << ").\n";
        std::cout << "This edge may not exist or is not a tree edge.\n";
        return;
    }
    
    std::cout << "Successfully cut edge (" << u << ", " << v << ").\n";

    // ========== 更新 components ==========
    auto it = std::find_if(components.begin(), components.end(),
                          [tree](const auto& ptr) { return ptr.get() == tree; });
    
    if (it != components.end()) {
        components.erase(it);  // 这会自动调用 unique_ptr 的析构
    }
    
    splaytree::EulerTourTree* tree1Ptr = newTree1.get();
    splaytree::EulerTourTree* tree2Ptr = newTree2.get();
    
    components.push_back(std::move(newTree1));  // newTree1 现在是 nullptr
    components.push_back(std::move(newTree2));  // newTree2 现在是 nullptr
    
    std::cout << "After cutting edge (" << u << ", " << v << "):\n";
    printComponents();

    tree1Ptr->link(u, v, tree2Ptr);
    std::cout << "After linking back edge (" << u << ", " << v << "):\n";
    tree1Ptr->printEulerTour();
}

vector<Block> DancingMatrix::getComponentsByIG(const set<int> rows) {
    return incrementalGraph->computeComponentsInRows(rows);
    // return findComponents(rows);
};

vector<Block> DancingMatrix::getComponentsByETT() {
    // 使用当前线程的 components
    auto& comps = getComponents();

    vector<Block> blocks;
    for (const auto& tree : comps) {
        unordered_set<int> comp_rows = tree->getVertices();
        set<int> block_rows(comp_rows.begin(), comp_rows.end());
        set<int> block_cols;

        for (int r : block_rows) {
            for (int c : row_to_cols[r]) {
                block_cols.insert(c);
            }
        }

        blocks.emplace_back(block_rows, block_cols);
    }
    return blocks;
};

void DancingMatrix::testReRoot(int v) {
    
    splaytree::EulerTourTree* tree = findEulerTourTree(v);
    if (!tree) {
        std::cout << "Error: Vertex " << v << " not found in any component.\n";
        return;
    }
    
    std::cout << "Before rerooting at vertex " << v << ":\n";
    tree->printEulerTour();
    
    tree->reroot(v);
    
    std::cout << "After rerooting at vertex " << v << ":\n";
    tree->printEulerTour();
}

void DancingMatrix::testSplay(int v) {

    splaytree::EulerTourTree* tree = findEulerTourTree(v);

    if (!tree) {
        std::cout << "Error: Vertex " << v << " not found in any component.\n";
        return;
    }
    
    tree->testSplay(v);
}

//插入元素到双向十字链表中
void DancingMatrix::insert( int r, int c )  
{  
    ColIndex[c].size++;  
    RowIndex[r].size++;
    // ColIndex[c].rows.insert(r);
    // RowIndex[r].cols.insert(c);
    auto newNode = std::make_unique<Node>(r, c);  
    Node* newNodePtr = newNode.get();

    Node* cur = &ColIndex[c];  
    while( cur->down != &ColIndex[c] && cur->down->row < r )  
        cur = cur->down;  

    newNodePtr->down = cur->down;  
    newNodePtr->up = cur;  
    cur->down->up = newNodePtr;  
    cur->down = newNodePtr;  
    if( RowIndex[r].right == nullptr )  
    {  
        RowIndex[r].right = newNodePtr; 
        newNodePtr->row_first_node = true;
        newNode->left = newNodePtr;  
        newNode->right = newNodePtr;  
    }  
    else  
    {  
        Node* rowHead = RowIndex[r].right;  
        cur = rowHead;  

        while( cur->right != rowHead && cur->right->col < c )  
            cur = cur->right;  

        newNodePtr->right = cur->right;  
        newNodePtr->left = cur;  
        cur->right->left = newNodePtr;  
        cur->right = newNodePtr;  
    }  

    dataNodes.push_back(std::move(newNode));
}

string DancingMatrix::encodeBlockState(const unordered_set<int>& cols){
    string state(COLS, '0'); // 初始化为全0字符串
    for(int col : cols) {
        state[col - 1] = '1'; // 将对应列设置为1   
    }
    return state;
}

size_t DancingMatrix::hashBlockState(const set<int>& cols) {
    size_t hash = 0;
    for(int col : cols) {
        hash ^= std::hash<int>()(col) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

//获取当前列的状态
size_t DancingMatrix::getColumnState() const {
    size_t hash = 0;
    ColumnHeader* cur = (ColumnHeader*)root->right;
    while (cur != root) {
        hash ^= std::hash<int>()(cur->col) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        cur = (ColumnHeader*)cur->right;
    }
    return hash;
}

// 创建当前状态的签名
Signature DancingMatrix::getColumnSignature() const {
    Signature sig;
    sig.covered.resize(COLS + 1, false);
    
    // 标记已覆盖的列
    for (int j = 1; j <= COLS; j++) {
        bool found = false;
        for (int k = root->right->col; k != 0; k = ColIndex[k].right->col) {
            if (k == j) {
                found = true;
                break;
            }
        }
        sig.covered[j] = !found;
    }
    
    return sig;
}

void DancingMatrix::build_mapping_from_cols(const unordered_set<int>& blockCols, unordered_map<int, set<int>>& rowToCols, unordered_map<int, set<int>>& colToRows)
{
    for (auto col : blockCols) {
        ColumnHeader* c = &ColIndex[col];
        Node* curR = c->down;
        while (curR != c) {
            rowToCols[curR->row].insert(col);
            colToRows[col].insert(curR->row);
            curR = curR->down;
        }
    }
}

void DancingMatrix::printMatrix() const
{
    std::cout<< "Remain Matrix Nodes: " << std::endl;
    ColumnHeader* current = (ColumnHeader*)root->right;
    while(current != root)
    {
        std::cout << "Column " << current->col << " size: " << current->size << " ";
        if(current->size > 0){
            Node* cur = current->down;
            std::cout << "{ Rows: ( ";
            while(cur != current)
            {
                std::cout << cur->row + 1;
                cur = cur->down;
                if(cur != current)
                    std::cout << ", ";
            }
            std::cout << " ) } " << std::endl;
        }
        current = (ColumnHeader*)current->right;
    }
    std::cout << std::endl;
}

void DancingMatrix::printBlocks( vector<Block>& blocks) const {
    int i = 1;
    for (auto& block : blocks) {
        block.printBlock(i++);
    }
}

void DancingMatrix::cover( int c )  
{  
    ColumnHeader* col = &ColIndex[c];  
    col->right->left = col->left;  
    col->left->right = col->right; 
    colsSet.erase(c);

    Node* curR, *curC;  
    curC = col->down;  
    while( curC != col )  
    {   
        Node* noteR = curC;  

        curR = noteR->right;  
        while( curR != noteR )  
        {  
            curR->down->up = curR->up;  
            curR->up->down = curR->down;  
            --ColIndex[curR->col].size;  

            curR = curR->right;  
        }  
        rowsSet.erase(curC->row);
        curC = curC->down;  
    }  
}

void DancingMatrix::uncover( int c )  
{  
    Node* curR, *curC;  
    ColumnHeader* col = &ColIndex[c];  
    curC = col->up;  
    while( curC != col )  
    {  
        Node* noteR = curC;  

        curR = curC->left;  
        while( curR != noteR )  
        {  
            ++ColIndex[curR->col].size;  
            curR->down->up = curR;  
            curR->up->down = curR;  

            curR = curR->left;  
        }  
        rowsSet.insert(curC->row);
        curC = curC->up;  
    }  
    col->right->left = col;  
    col->left->right = col;  
    colsSet.insert(c);
}

void DancingMatrix::coverInBlock(int c, Block& block, set<int>& removed_rows){

    ColumnHeader* col = &ColIndex[c];  
    col->right->left = col->left;  
    col->left->right = col->right;  
    
    block.cols.erase(c); // 从块中移除列

    Node* curR, *curC;  
    curC = col->down;  

    while( curC != col )  
    {    
        int row_id = curC->row;
        removed_rows.insert(row_id);

        if (isGraphSyncEnabled() && useIG) {
            incrementalGraph->deactivateRow(row_id);
        }
        block.rows.erase(row_id); // 从块中移除行

        curR = curC->right;  
        while( curR != curC )  
        {          
            curR->down->up = curR->up;  
            curR->up->down = curR->down;  
            --ColIndex[curR->col].size;
            curR = curR->right;  
        }  

        curC = curC->down;  
    } 
}

void DancingMatrix::uncoverInBlock(int c, Block& block){ 
    ColumnHeader* col = &ColIndex[c];  
    Node* curC = col->up; 

    while( curC != col )  
    {  
        int row_id = curC->row;
 
        Node* curR = curC->left;  
        while( curR != curC )  
        {  
            ++ColIndex[curR->col].size;
            curR->down->up = curR;  
            curR->up->down = curR;  
            curR = curR->left;  
        }  

        block.rows.insert(row_id); // 将行添加到块中

        if (isGraphSyncEnabled() && useIG) {
            incrementalGraph->reactivateRow(row_id); 
        }

        curC = curC->up;  
    }  

    col->right->left = col;  
    col->left->right = col;  
    block.cols.insert(c);
}

ColumnHeader* DancingMatrix::selectColumnHeuristic(const set<int>& cols) {
    ColumnHeader* chosen = nullptr;
    int minSize = INT_MAX;

    for (int col : cols) {
        if (col < 1 || col > COLS) continue; // 跳过无效列
        int sz = getColSize(col);

        if (sz < minSize) {
            minSize = sz;
            chosen = getColumnHeader(col);
        }
    }
    return chosen;
}

ColumnHeader* DancingMatrix::selectColumnByLinear(const set<int>& cols, int threshold) {
    
    ColumnHeader* bestCol = getColumnHeader(*cols.begin()); // 保底选择第一列
    int bestSize = getColSize(bestCol->col);
    
    for (int colId : cols) {
        int size = getColSize(colId);
        
        if (size < bestSize) {
            bestCol = getColumnHeader(colId);
            bestSize = size;
        }
    }
    
    return bestCol; // 保证非空返回
}

ColumnHeader* DancingMatrix::selectColumnByMinHeap(const set<int>& cols, int threshold) {
    
    // 使用小根堆选择最小列
    priority_queue<ColumnInfo, vector<ColumnInfo>, greater<ColumnInfo>> minHeap;
    
    for (int colId : cols) {
        int size = getColSize(colId);
        minHeap.push({colId, size, getColumnHeader(colId)});
    }
    
    // 保底返回：记录第一个（最小size）列
    ColumnInfo firstCol = minHeap.top();
    ColumnHeader* bestCol = firstCol.header;
    int bestDistance = abs(firstCol.size - threshold);
    
    // 从小到大取出元素，寻找最接近阈值的列
    while (!minHeap.empty()) {
        ColumnInfo current = minHeap.top();
        minHeap.pop();
        
        int distance = abs(current.size - threshold);
        
        // 更新最优列
        if (distance < bestDistance) {
            bestDistance = distance;
            bestCol = current.header;
        }
        
        // 早期退出优化：
        // 1. 找到精确匹配
        if (distance == 0) break;
        
        // 2. 当前size已经超过阈值，且距离开始增大时停止
        //    因为后续元素size更大，距离只会越来越大
        if (current.size > threshold && distance > bestDistance) {
            break;
        }
    }
    
    return bestCol;  // 保证非空
}

ColumnHeader* DancingMatrix::selectOptimalColumn(const set<int>& cols) {
    
    // if (cols.size() <= HEAP_THRESHOLD) {
    //     return selectColumnByLinear(cols, TARGET_THRESHOLD);
    // }
    // if (single_thread_mode) {
    //     return selectColumnByMinHeap(cols, TARGET_THRESHOLD);
    // }

    return selectColumnByLinear(cols, TARGET_THRESHOLD);
}

ColumnHeader* DancingMatrix::selectCol()
{
    ColumnHeader* choose = getColumnHeader(root->right->col), *cur=choose;  
    while( cur != root )  
    {   //选择元素最少的列
        int chooseSize = getColSize(choose->col);
        int curSize = getColSize(cur->col);
        if( chooseSize > curSize )  
            choose = cur;  
        cur = getColumnHeader(cur->right->col);  
    } 
    return choose;
}

col_id DancingMatrix::getClosedSizeCol(const int expected_size) {
    ColumnHeader* choose = (ColumnHeader*)root->right, *cur=choose;  
    while( cur != root )  
    {   //选择接近预期大小的列
        if( abs(choose->size - expected_size) > abs(cur->size - expected_size) )  
            choose = cur;  
        cur = (ColumnHeader*)cur->right;  
    } 
    return choose->col;
}

col_id DancingMatrix::getSmallestSizeCol() {
    ColumnHeader* choose = (ColumnHeader*)root->right, *cur=choose;  
    while( cur != root )  
    {   //选择元素最少的列
        if( choose->size > cur->size )  
            choose = cur;  
        cur = (ColumnHeader*)cur->right;  
    } 
    return choose->col;
}

void PreProccess::extractNM(const std::string& line, int& n, int& m) {
    std::istringstream iss(line);
    std::string token;

    // 读取 "c"
    iss >> token;  // 读取 "c"

    iss >> token;  // 读取 "n"
    if (token == "n") {
        iss >> token;  // 读取 "="
        iss >> n;      // 读取 n 的值
    } else {
        throw std::runtime_error("格式错误: 未找到 'n'");
    }

    // 读取 "m" 和其值
    iss >> token;  // 读取 ","
    iss >> token;  // 读取 "m"
    if (token == "m") {
        iss >> token;  // 读取 "="
        iss >> m;      // 读取 m 的值
    } else {
        throw std::runtime_error("格式错误: 未找到 'm'");
    }
}

int** PreProccess::processFileToMatrix1(const std::string& filename, int& r, int& c) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行

    int n, m;
    extractNM(line, n, m);  // 读取第一行的 n 和 m

    r = m;
    c = n;

    std::getline(file, line);  // 跳过第二行(第二个数据集不用跳过)

    // 动态分配二维数组
    int** matrix = new int*[r];
    for (int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for (int j = 0; j < c; ++j) {
            matrix[i][j] = 0;  // 初始化为0
        }
    }

    int row = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != 's') continue;  // 忽略不是以 's' 开头的行
        std::istringstream iss(line.substr(2));  // 跳过前两个字符's '
        //std::istringstream iss(line);
        int col;
        while (iss >> col) {
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                matrix[row][col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        ++row;
    }

    file.close();
    return matrix;
}

int** PreProccess::processFileToMatrix2(const fs::path& filename, int& r, int& c) {
    std::ifstream file(filename.string());
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行

    int n, m;
    std::istringstream iss(line);
    iss >> n >> m;  // 假设第一行格式为 "n m"  
    c = n;
    r = m;

    // 动态分配二维数组
    vector<vector<int>> tmp_matrix;

    while (std::getline(file, line)) {
        vector<int> tmp_row(c, 0);  // 初始化当前行
        std::istringstream iss(line);
        int tmp;
        iss >> tmp;   // 跳过开头的数字
        iss >> tmp;  // 读取该行“1”的个数

        int col;
        for (int i = 0; i < tmp; ++i) {
            iss >> col;  // 读取列索引
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                tmp_row[col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        tmp_matrix.push_back(tmp_row);
    }
    file.close();
    
    r = tmp_matrix.size();
    // 动态分配二维数组
    int** matrix = new int*[r];
    for(int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for(int j = 0; j < c; ++j) {
            matrix[i][j] = tmp_matrix[i][j];  // 将1填入矩阵
        }
    }
    return matrix;
}

int** PreProccess::processFileToMatrix3(const std::string& filename, int& r, int& c) { 
    std::ifstream file(filename);
    if(!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行
    std::istringstream iss(line);
    int n, m;
    iss >> n >> m;  // 假设第一行格式为 "n m"
    c = n;
    r = m;

    // 动态分配二维数组
    int** matrix = new int*[r];
    for(int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for(int j = 0; j < c; ++j) {
            matrix[i][j] = 0;
        }
    }

    int row = 0;
    while (std::getline(file, line))
    {
        std::istringstream rowStream(line);
        int col, count;

        // 读取该行“1”的个数
        rowStream >> count;

        for (int i = 0; i < count; ++i) {
            rowStream >> col;  // 读取列索引
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                matrix[row][col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        row++;
    }
    
    file.close();
    return matrix;
}

void PreProccess::freeMatrix(int** matrix, int rows) {
    for (int i = 0; i < rows; ++i) {
        delete[] matrix[i];
    }
    delete[] matrix;
}