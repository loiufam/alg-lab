#include "../include/DancingMatrix.h"

class IncrementalConnectedGraph {
private:
    struct RowState {
        std::unordered_set<int> activeColumns;  // 当前激活的列
        std::unordered_set<int> removedColumns; // 被移除的列
    };
    
    std::vector<int> parent;
    std::vector<int> rank;
    std::vector<RowState> rowStates;
    std::vector<std::unordered_set<int>> componentColumns;
    std::unordered_map<int, std::unordered_set<int>> activeColumnToRows;
    mutable std::shared_mutex mtx;
    
    int N;
    int numComponents;
    bool needsRebuild;
    
    int find(int x) {
        while (parent[x] != x) {
            x = parent[x];
        }
        return x;
    }
    
    bool unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        
        if (rootX == rootY) return false;
        
        if (rank[rootX] < rank[rootY]) {
            std::swap(rootX, rootY);
        }
        
        parent[rootY] = rootX;
        
        if (rank[rootX] == rank[rootY]) {
            rank[rootX]++;
        }
        
        componentColumns[rootX].insert(
            componentColumns[rootY].begin(),
            componentColumns[rootY].end()
        );
        
        numComponents--;
        return true;
    }
    
    void rebuildIfNeeded() {
        if (!needsRebuild) return;
        std::unique_lock<std::shared_mutex> write_lock(mtx);
               
        // 重置
        for (int i = 0; i < N; ++i) {
            parent[i] = i;
            rank[i] = 0;
            componentColumns[i].clear();
        }
        numComponents = N;
        activeColumnToRows.clear();
        
        // 重建列索引
        for (int row = 0; row < N; ++row) {
            for (int col : rowStates[row].activeColumns) {
                activeColumnToRows[col].insert(row);
            }
        }
        
        // 重建连通分量
        for (auto& [col, rows] : activeColumnToRows) {
            if (rows.size() < 2) {
                // 单个行也需要记录其列
                if (rows.size() == 1) {
                    int row = *rows.begin();
                    int root = find(row);
                    componentColumns[root].insert(col);
                }
                continue;
            }
            
            int firstRow = *rows.begin();
            for (int row : rows) {
                if (row != firstRow) {
                    unite(firstRow, row);
                }
            }
            
            // 为整个连通分量添加该列
            int root = find(firstRow);
            componentColumns[root].insert(col);
        }
        
        needsRebuild = false;
              
    }

    /**
     * 在指定的行集合内局部重建连通分量
     * @param restrictedRows 限制的行集合
     * @param localParent 局部的并查集父节点数组（输出）
     * @param localRank 局部的并查集秩数组（输出）
     * @param localComponentColumns 局部的分量列集合（输出）
     */
    void rebuildLocally(
        const std::set<int>& restrictedRows,
        std::vector<int>& localParent,
        std::vector<int>& localRank,
        std::vector<std::unordered_set<int>>& localComponentColumns) const 
    {
        // 初始化局部并查集
        localParent.resize(N);
        localRank.resize(N, 0);
        localComponentColumns.resize(N);
        
        for (int i = 0; i < N; ++i) {
            localParent[i] = i;
        }
        
        // 构建局部列索引（仅包含在 restrictedRows 中的行）
        std::unordered_map<int, std::unordered_set<int>> localColumnToRows;
        
        for (int row : restrictedRows) {
            if (row < 0 || row >= N) continue;

            std::unordered_set<int> snapshot;
            {
                std::shared_lock<std::shared_mutex> rlk(mtx);
                snapshot = rowStates[row].activeColumns; // copy
            }
            
            for (int col : snapshot) {
                localColumnToRows[col].insert(row);
            }
        }
        
        // 局部的 find 函数
        std::function<int(int)> localFind = [&localParent](int x) -> int {
            int root = x;
            while (localParent[root] != root) root = localParent[root];
            // 压缩路径
            while (x != root) {
                int nxt = localParent[x];
                localParent[x] = root;
                x = nxt;
            }
            return root;
        };
        
        // 局部的 unite 函数
        auto localUnite = [&localParent, &localRank, &localComponentColumns, &localFind](int a, int b) {
            int ra = localFind(a);
            int rb = localFind(b);
            if (ra == rb) return false;
            // 按秩合并
            if (localRank[ra] < localRank[rb]) std::swap(ra, rb);
            localParent[rb] = ra;
            if (localRank[ra] == localRank[rb]) ++localRank[ra];
            // 小并入大：如果 rb 的列集合比 ra 大，交换它们
            if (localComponentColumns[ra].size() < localComponentColumns[rb].size())
                localComponentColumns[ra].swap(localComponentColumns[rb]);
            localComponentColumns[ra].insert(localComponentColumns[rb].begin(), localComponentColumns[rb].end());
            return true;
        };
        
        // 通过局部列索引构建连通分量
        for (const auto& [col, rows] : localColumnToRows) {
            if (rows.empty()) continue; // 空列跳过

            if (rows.size() == 1) {
                int row = *rows.begin();
                int root = localFind(row);
                localComponentColumns[root].insert(col);
                continue;
            }

            // 将所有行合并到第一个行
            int firstRow = *rows.begin();
            for (int row : rows) {
                if (row != firstRow) {
                    localUnite(firstRow, row);
                }
            }
            
            // 为整个连通分量添加该列
            int root = localFind(firstRow);
            localComponentColumns[root].insert(col);
        }
    }


public:
    // 连通分量结构
    struct Component {
        int id;                          // 分量ID（根节点）
        std::vector<int> rows;           // 包含的行
        std::vector<int> columns;        // 包含的列（排序后）
        std::unordered_set<int> columnSet; // 列集合（快速查询）
        
        Component() : id(-1) {}
        
        Component(int _id, std::vector<int> _rows, std::unordered_set<int> _cols) 
            : id(_id), rows(std::move(_rows)), columnSet(std::move(_cols)) {
            columns.assign(columnSet.begin(), columnSet.end());
            std::sort(columns.begin(), columns.end());
            std::sort(rows.begin(), rows.end());
        }
        
        size_t size() const { return rows.size(); }
        size_t columnCount() const { return columns.size(); }
        
        bool hasColumn(int col) const {
            return columnSet.count(col) > 0;
        }
        
        void print() const {
            std::cout << "分量 " << id << ": " 
                      << rows.size() << " 行, " 
                      << columns.size() << " 列" << std::endl;
            std::cout << "  行: ";
            for (size_t i = 0; i < std::min(size_t(10), rows.size()); ++i) {
                std::cout << rows[i] << " ";
            }
            if (rows.size() > 10) std::cout << "...";
            std::cout << std::endl;
            
            std::cout << "  列: ";
            for (size_t i = 0; i < std::min(size_t(10), columns.size()); ++i) {
                std::cout << columns[i] << " ";
            }
            if (columns.size() > 10) std::cout << "...";
            std::cout << std::endl;
        }
    };
    
    IncrementalConnectedGraph(int rows) 
        : N(rows), parent(rows), rank(rows, 0), rowStates(rows),
          componentColumns(rows), numComponents(rows), needsRebuild(false) {
        
        for (int i = 0; i < N; ++i) {
            parent[i] = i;
        }
    }
    
    // **初始化：添加所有初始元素**
    void initialize(const DancingMatrix& matrix) {
        std::cerr << "开始初始化图..." << std::endl;
        
        const ColumnHeader* root = matrix.getColumnHeader(0);
        Node* curCol = root->right;
        
        int colIndex = 1;
        size_t totalElements = 0;
        
        while (curCol != root) {
            std::vector<int> rowsInColumn;
            Node* curRow = curCol->down;
            
            while (curRow != curCol) {
                int row = curRow->row;
                rowsInColumn.push_back(row);
                rowStates[row].activeColumns.insert(colIndex);
                totalElements++;
                curRow = curRow->down;
            }
            
            if (!rowsInColumn.empty()) {
                for (int row : rowsInColumn) {
                    activeColumnToRows[colIndex].insert(row);
                }
                
                if (rowsInColumn.size() >= 2) {
                    int firstRow = rowsInColumn[0];
                    for (size_t i = 1; i < rowsInColumn.size(); ++i) {
                        unite(firstRow, rowsInColumn[i]);
                    }
                }
                
                // 为所有涉及的行添加该列到其连通分量
                for (int row : rowsInColumn) {
                    int root = find(row);
                    componentColumns[root].insert(colIndex);
                }
            }
            
            curCol = curCol->right;
            colIndex++;
        }
        
        // std::cerr << "初始化完成: " 
        //           << totalElements << " 个元素, "
        //           << numComponents << " 个连通分量" << std::endl;
    }
    
    // **移除元素（标记为非激活）**
    void deactivateElement(int row, int col) {
        if (!rowStates[row].activeColumns.count(col)) {
            return;
        }
        
        rowStates[row].activeColumns.erase(col);
        rowStates[row].removedColumns.insert(col);
        activeColumnToRows[col].erase(row);
        
        needsRebuild = true;
    }
    
    // **批量移除行的所有列**
    void deactivateRow(int row) {
        std::unique_lock<std::shared_mutex> lock(mtx);  // 写锁，自动释放

        for (int col : rowStates[row].activeColumns) {
            activeColumnToRows[col].erase(row);
            rowStates[row].removedColumns.insert(col);
        }
        rowStates[row].activeColumns.clear();
        
        needsRebuild = true;
    }
    
    // **批量移除多行**
    void deactivateRows(const std::vector<int>& rows) {
        for (int row : rows) {
            for (int col : rowStates[row].activeColumns) {
                activeColumnToRows[col].erase(row);
                rowStates[row].removedColumns.insert(col);
            }
            rowStates[row].activeColumns.clear();
        }
        
        needsRebuild = true;
    }
    
    // **恢复元素**
    void reactivateElement(int row, int col) {
        if (!rowStates[row].removedColumns.count(col)) {
            return;
        }
        
        rowStates[row].removedColumns.erase(col);
        rowStates[row].activeColumns.insert(col);
        activeColumnToRows[col].insert(row);
        
        needsRebuild = true;
    }
    
    // **批量恢复行的所有列**
    void reactivateRow(int row) {
        std::unique_lock<std::shared_mutex> lock(mtx);  // 写锁，自动释放

        for (int col : rowStates[row].removedColumns) {
            rowStates[row].activeColumns.insert(col);
            activeColumnToRows[col].insert(row);
        }
        rowStates[row].removedColumns.clear();
        
        needsRebuild = true;
    }
    
    // **批量恢复多行**
    void reactivateRows(const std::vector<int>& rows) {
        for (int row : rows) {
            for (int col : rowStates[row].removedColumns) {
                rowStates[row].activeColumns.insert(col);
                activeColumnToRows[col].insert(row);
            }
            rowStates[row].removedColumns.clear();
        }
        
        needsRebuild = true;
    }
    
    // **核心功能：全局计算所有连通分量**
    std::vector<Component> computeComponents() {
        rebuildIfNeeded();
        
        // 收集每个根节点的所有行
        std::unordered_map<int, std::vector<int>> componentMap;
        
        for (int i = 0; i < N; ++i) {
            // 只考虑有激活列的行
            if (rowStates[i].activeColumns.empty()) {
                continue;
            }
            
            int root = find(i);
            componentMap[root].push_back(i);
        }
        
        // 构建 Component 对象
        std::vector<Component> components;
        components.reserve(componentMap.size());
        
        for (auto& [root, rows] : componentMap) {
            // 创建该连通分量的列集合副本
            std::unordered_set<int> cols = componentColumns[root];
            components.emplace_back(root, std::move(rows), std::move(cols));
        }
        
        // 按分量大小排序（可选）
        std::sort(components.begin(), components.end(), 
                  [](const Component& a, const Component& b) {
                      return a.size() > b.size();
                  });
        
        return components;
    }

    /**
     * 在指定的行集合内计算连通分量（不限制列）
     * @param rows 限定的行集合
     * @return 该行集合内的所有连通分量
     */
    std::vector<Block> computeComponentsInRows(const std::set<int>& rows) const {
        // 过滤出有效的行
        // std::unordered_set<int> validRows;
        // for (int row : rows) {
        //     if (row >= 0 && row < N && !rowStates[row].activeColumns.empty()) {
        //         validRows.insert(row);
        //     }
        // }
        
        // if (validRows.empty()) {
        //     return {};
        // }
        
        // 局部并查集
        std::vector<int> localParent;
        std::vector<int> localRank;
        std::vector<std::unordered_set<int>> localComponentColumns;
        
        rebuildLocally(rows, localParent, localRank, localComponentColumns);
        
        // 局部 find 函数
        auto localFind = [&localParent](int x) {
            while (localParent[x] != x) {
                x = localParent[x];
            }
            return x;
        };
        
        // 收集连通分量
        std::unordered_map<int, std::vector<int>> componentMap;
        
        for (int row : rows) {
            int root = localFind(row);
            componentMap[root].push_back(row);
        }
        
        // 构建 Component 对象
        std::vector<Block> blocks;
        blocks.reserve(componentMap.size());
        
        for (auto& [root, rows] : componentMap) {
            std::unordered_set<int> cols = localComponentColumns[root];
            blocks.emplace_back(std::move(rows), std::move(cols));
        }
        
        std::sort(blocks.begin(), blocks.end(), 
                  [](const Block& a, const Block& b) {
                      return a.size() > b.size();
                  });
        
        return blocks;
    }
    
    // **计算连通分量的数量**
    int computeNumComponents() {
        rebuildIfNeeded();
        
        std::unordered_set<int> roots;
        for (int i = 0; i < N; ++i) {
            if (!rowStates[i].activeColumns.empty()) {
                roots.insert(find(i));
            }
        }
        
        return roots.size();
    }
    
    // **获取所有连通分量的列集合（返回列表）**
    std::vector<std::vector<int>> getComponentColumnSets() {
        auto components = computeComponents();
        
        std::vector<std::vector<int>> result;
        result.reserve(components.size());
        
        for (const auto& comp : components) {
            result.push_back(comp.columns);
        }
        
        return result;
    }
    
    // **获取所有连通分量的列集合及行数**
    std::vector<Block> getComponentColumnSetsAsSet() {
        auto components = computeComponents();
        
        std::vector<Block> result;
        result.reserve(components.size());
        
        for (auto& comp : components) {
            // int rowCount = static_cast<int>(comp.rows.size());
            result.emplace_back(std::move(comp.rows), std::move(comp.columnSet));
        }
        
        return result;
    }

    // **获取特定行所在连通分量的列集合**
    std::vector<int> getColumnsForRow(int row) {
        rebuildIfNeeded();
        
        if (rowStates[row].activeColumns.empty()) {
            return {};
        }
        
        int root = find(row);
        std::vector<int> result(componentColumns[root].begin(), 
                                componentColumns[root].end());
        std::sort(result.begin(), result.end());
        return result;
    }
    
    // **获取特定行所在连通分量的所有行**
    std::vector<int> getComponentRows(int row) {
        rebuildIfNeeded();
        
        if (rowStates[row].activeColumns.empty()) {
            return {};
        }
        
        int targetRoot = find(row);
        std::vector<int> result;
        
        for (int i = 0; i < N; ++i) {
            if (!rowStates[i].activeColumns.empty() && find(i) == targetRoot) {
                result.push_back(i);
            }
        }
        
        std::sort(result.begin(), result.end());
        return result;
    }
    
    // **查询两行是否连通**
    bool isConnected(int row1, int row2) {
        rebuildIfNeeded();
        
        if (rowStates[row1].activeColumns.empty() || 
            rowStates[row2].activeColumns.empty()) {
            return false;
        }
        
        return find(row1) == find(row2);
    }
    
    // **获取行的激活列**
    const std::unordered_set<int>& getActiveColumns(int row) const {
        return rowStates[row].activeColumns;
    }
    
    // **获取行的已移除列**
    const std::unordered_set<int>& getRemovedColumns(int row) const {
        return rowStates[row].removedColumns;
    }
    
    // **获取连通分量数量**
    int getNumComponents() {
        return computeNumComponents();
    }
    
    // **强制立即重建**
    void forceRebuild() {
        needsRebuild = true;
        rebuildIfNeeded();
    }
    
    // **打印所有连通分量的详细信息**
    void printComponents() {
        auto components = computeComponents();
        
        std::cout << "\n========== 连通分量详情 ==========" << std::endl;
        std::cout << "总共 " << components.size() << " 个连通分量" << std::endl;
        std::cout << "===================================\n" << std::endl;
        
        for (size_t i = 0; i < components.size(); ++i) {
            std::cout << "[" << i << "] ";
            components[i].print();
            std::cout << std::endl;
        }
    }
    
    // **获取统计信息**
    void printStats() const {
        int totalActive = 0;
        int totalRemoved = 0;
        int activeRows = 0;
        
        for (const auto& state : rowStates) {
            totalActive += state.activeColumns.size();
            totalRemoved += state.removedColumns.size();
            if (!state.activeColumns.empty()) {
                activeRows++;
            }
        }
        
        std::cerr << "\n=== 图统计信息 ===" << std::endl;
        std::cerr << "总行数: " << N << std::endl;
        std::cerr << "激活行数: " << activeRows << std::endl;
        std::cerr << "激活元素: " << totalActive << std::endl;
        std::cerr << "移除元素: " << totalRemoved << std::endl;
        std::cerr << "需要重建: " << (needsRebuild ? "是" : "否") << std::endl;
        std::cerr << "===================" << std::endl;
    }
    
    // **导出连通分量为 JSON 格式（用于调试）**
    std::string exportComponentsJSON() {
        auto components = computeComponents();
        
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"num_components\": " << components.size() << ",\n";
        oss << "  \"components\": [\n";
        
        for (size_t i = 0; i < components.size(); ++i) {
            const auto& comp = components[i];
            oss << "    {\n";
            oss << "      \"id\": " << comp.id << ",\n";
            oss << "      \"rows\": [";
            for (size_t j = 0; j < comp.rows.size(); ++j) {
                oss << comp.rows[j];
                if (j < comp.rows.size() - 1) oss << ", ";
            }
            oss << "],\n";
            oss << "      \"columns\": [";
            for (size_t j = 0; j < comp.columns.size(); ++j) {
                oss << comp.columns[j];
                if (j < comp.columns.size() - 1) oss << ", ";
            }
            oss << "]\n";
            oss << "    }";
            if (i < components.size() - 1) oss << ",";
            oss << "\n";
        }
        
        oss << "  ]\n";
        oss << "}\n";
        
        return oss.str();
    }
};