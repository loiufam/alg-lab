#include "../include/ComponentDetector.h"

// 定义 thread_local 静态成员
// thread_local std::vector<CoverHistory> ComponentDetector::cover_stack_;
// thread_local std::set<int> ComponentDetector::current_rows_;
// thread_local std::vector<std::set<int>> ComponentDetector::current_components_;

ComponentDetector::ComponentDetector(const int n, const int m) 
    : num_rows(n),  num_cols(m) {

    ett = std::make_unique<SplayETT>(n);

    // 初始化邻接表和激活状态
    adj_list.resize(n);
    row_active.resize(n, true);  // 初始时所有行都激活

    for (int i = 1; i <= m; ++i) {
        active_cols.insert(i);
    }

}

void ComponentDetector::Initialize(const std::unordered_map<int, std::vector<int>>& col_rows_map) {
    col_to_rows = col_rows_map;
   
    // 构建 row_to_cols 和邻接表
    for (const auto& [col, rows] : col_to_rows) {
        for (int row : rows) {
            row_to_cols[row].insert(col);
        }
        
        for (size_t i = 0; i < rows.size(); ++i) {
            for (size_t j = i + 1; j < rows.size(); ++j) {
                addEdgeToAdjList(rows[i], rows[j]);
            }
        }
    }
    
    // BFS构建生成森林
    BuildSpanningForest();
}

void ComponentDetector::BuildSpanningForest() {
    std::vector<bool> visited(num_rows, false);
    
    // 对每个连通分量进行BFS
    for (int start = 0; start < num_rows; start++) {
        if (!row_active[start] || visited[start]) continue;
        
        // BFS构建这个分量的生成树
        std::queue<int> q;
        std::vector<int> component_vertices;
        std::vector<std::pair<int, int>> tree_edge_list;
        std::unordered_set<unsigned long long> all_edges_in_comp;
        
        q.push(start);
        visited[start] = true;
        component_vertices.push_back(start);
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            for (int v : adj_list[u]) {
                
                unsigned long long key = makeEdgeKey(u, v);
                all_edges_in_comp.insert(key);
                
                if (!visited[v]) {
                    visited[v] = true;
                    component_vertices.push_back(v);
                    tree_edge_list.push_back({u, v});
                    tree_edges.insert(key);
                    q.push(v);
                }
            }
        }
        
        // Link树边到ETT
        ett->batchLink(tree_edge_list);

        // 获取连通分量ID
        int comp_id = ett->getComponentId(start);
        
        // 创建连通分量信息
        ComponentInfo comp_info;
        comp_info.vertices = std::unordered_set<int>(
            component_vertices.begin(), 
            component_vertices.end()
        );
        
        int max_level = calculateMaxLevel(component_vertices.size());
        comp_info.non_tree_edges = std::make_unique<LayeredNonTreeEdges>(max_level);
                
        // 识别非树边并添加到最高层
        for (unsigned long long key : all_edges_in_comp) {
            if (tree_edges.find(key) == tree_edges.end()) {
                comp_info.non_tree_edges->addEdge(key, max_level);
            }
        }

        component_map[comp_id] = std::move(comp_info);
        
    }

}

void ComponentDetector::Cover(int c) {
    auto it = col_to_rows.find(c);
    if (it == col_to_rows.end()) return;
    
    const auto& rows = it->second;
    for (int row : rows) {
        if (row_active[row]) {
            row_active[row] = false;
        }
    }
}

void ComponentDetector::Uncover() {
    // 仅恢复行的激活状态（从历史记录中）
    if (operation_stack.empty()) {
        throw std::runtime_error("Cannot uncover: operation stack is empty");
    }
    
    EdgeOperationHistory& history = operation_stack.back();
    
    // 恢复所有被删除边涉及的行
    for (const auto& [u, v] : history.deleted_edges) {
        if (u >= 0 && u < num_rows) row_active[u] = true;
        if (v >= 0 && v < num_rows && v != u) row_active[v] = true;
    }
}

void ComponentDetector::DeleteEdges(const std::set<int>& deleted_rows) {
    if (deleted_rows.empty()) return;
    
    
    // 收集所有涉及删除顶点的边
    std::vector<std::pair<int, int>> edges_to_delete;
    std::unordered_set<unsigned long long> processed_edges;
    
    for (int u : deleted_rows) {
        row_active[u] = false;
        for (int v : adj_list[u]) {
            if (!row_active[v] || deleted_rows.count(v)) continue;
            
            unsigned long long key = makeEdgeKey(u, v);
            if (processed_edges.count(key)) continue;
            processed_edges.insert(key);
            
            // history.deleted_edges.push_back({u, v});
            edges_to_delete.push_back({u, v});
        }
    }
    
    // std::unique_lock<std::shared_mutex> lock(access_ett_mutex);

    // 按顺序处理每条边
    for (const auto& [u, v] : edges_to_delete) {
        unsigned long long key = makeEdgeKey(u, v);
        
        // 确定活跃顶点和分量ID, 须确保有一端是活跃的
        int active_vertex = deleted_rows.count(u) ? v : u;
        int component_id = ett->getComponentId(active_vertex);
        
        if (ett->isTreeEdge(u, v)) {
            // 删除树边
            ett->cut(u, v);
            
            // 查找替代边
            int new_comp_v_vertex = deleted_rows.count(u) ? u : v;
            
            // 只有当另一端也是活跃顶点时才查找替代边
            if (!deleted_rows.count(new_comp_v_vertex)) {
                int new_comp_u = ett->getComponentId(active_vertex);
                int new_comp_v = ett->getComponentId(new_comp_v_vertex);
                
                if (new_comp_u != new_comp_v && component_map.count(component_id)) {
                    auto replacement = FindReplacementLayered(component_map[component_id]);
                    
                    if (replacement.has_value()) {

                        int ru = replacement->first;
                        int rv = replacement->second;
                        unsigned long long repl_key = makeEdgeKey(ru, rv);
                        
                        // 升级为树边
                        ett->link(ru, rv);
                        // tree_edges.insert(repl_key);
                        // non_tree_edges.erase(repl_key);
 
                        
                        if (component_map.count(component_id)) {
                            // component_map[component_id].tree_edges.insert(repl_key);
                            if (component_map[component_id].non_tree_edges) {
                                component_map[component_id].non_tree_edges->removeEdge(repl_key);
                            }
                        }
                    }
                }
            }
            
        } else {
            // 删除非树边
            // non_tree_edges.erase(key);
            
            if (component_map.count(component_id) && 
                component_map[component_id].non_tree_edges) {
                component_map[component_id].non_tree_edges->removeEdge(key);
            }
        }
        
        // edge_info_map.erase(key);
        // history.operations.push_back(op);
    }
    // lock.unlock();
}

void ComponentDetector::AddEdges(const std::set<int>& add_rows) {

    if (add_rows.empty()) return;
    for (int row : add_rows) {
        if (!row_active[row]) {
            row_active[row] = true;
        }
    }

    // 收集所有需要添加的边
    std::vector<std::pair<int, int>> edges_to_add;
    std::unordered_set<unsigned long long> processed_edges;
    
    for (int u : add_rows) {
        for (int v : adj_list[u]) {
            // 只添加两端都活跃的边
            if (!row_active[v]) continue;
            
            unsigned long long key = makeEdgeKey(u, v);
            if (processed_edges.count(key)) continue;
            processed_edges.insert(key);
            
            edges_to_add.push_back({u, v});
        }
    }

    // std::unique_lock<std::shared_mutex> lock(access_ett_mutex);

    // 尝试link所有边，成功的成为树边，失败的成为非树边
    for (const auto& [u, v] : edges_to_add) {
        unsigned long long key = makeEdgeKey(u, v);

        bool linked = ett->link(u, v);

        if (!linked) {
            // Link失败（已连通），成为非树边

            int comp_id = ett->getComponentId(u);
            if (component_map.count(comp_id) && 
                component_map[comp_id].non_tree_edges) {
                int level = calculateMaxLevel(component_map[comp_id].vertices.size());
                
                component_map[comp_id].non_tree_edges->addEdge(key, level);
            }
        }
    }

}

std::vector<Block> ComponentDetector::GetBlocks(const std::set<int>& block_rows) {
    std::vector<Block> blocks;
    
    // 根据连通分量分组
    std::unordered_map<int, std::set<int>> comp_rows_map;
    for (int row : block_rows) {
        if (row_active[row]) {
            int comp_id = ett->getComponentId(row);
            comp_rows_map[comp_id].insert(row);
        }
    }
    
    // 只有多个分量时才分解
    if (comp_rows_map.size() <= 1) {
        return blocks;
    }
    
    // 转换为Block
    for (const auto& [comp_id, rows] : comp_rows_map) {
        std::set<int> component_cols;
        std::vector<int> component_rows_vec(rows.begin(), rows.end());
        
        for (int r : rows) {
            auto it = row_to_cols.find(r);
            if (it != row_to_cols.end()) {
                for (int col : it->second) {
                    component_cols.insert(col);
                }
            }
        }
        
        if (!component_rows_vec.empty()) {
            blocks.emplace_back(std::move(component_rows_vec), 
                               std::move(component_cols));
        }
    }
    
    return blocks;
}

// 在指定分量中查找替代边
// std::optional<std::pair<int,int>> ComponentDetector::FindReplacementInComponent(
//     int u, int v, int comp_root) {
    
//     if (!component_map.count(comp_root)) return std::nullopt;
    
//     int comp_u = ett->getComponentId(u);
//     int comp_v = ett->getComponentId(v);
    
//     if (comp_u == comp_v) return std::nullopt; // 仍然连通
    
//     // 在较小的分量中搜索
//     int size_u = ett->componentSize(u);
//     int size_v = ett->componentSize(v);
//     if (size_u > size_v) {
//         std::swap(u, v);
//         std::swap(comp_u, comp_v);
//     }
    
//     // 在分量的非树边中查找
//     const auto& non_tree_edge_set = component_map[comp_root].non_tree_edges;
    
//     for (unsigned long long key : non_tree_edge_set) {
//         int e_u = static_cast<int>(key >> 32);
//         int e_v = static_cast<int>(key & 0xFFFFFFFF);
        
//         if (!row_active[e_u] || !row_active[e_v]) continue;
        
//         int root_eu = ett->getComponentId(e_u);
//         int root_ev = ett->getComponentId(e_v);
        
//         // 检查是否连接两个分裂的分量
//         if ((root_eu == comp_u && root_ev == comp_v) ||
//             (root_eu == comp_v && root_ev == comp_u)) {
//             return std::make_pair(e_u, e_v);
//         }
//     }
    
//     return std::nullopt;
// }

// 分层查找替代边
std::optional<std::pair<int,int>> ComponentDetector::FindReplacementLayered(ComponentInfo& comp_info) {
    
    if (!comp_info.non_tree_edges) return std::nullopt;
    
    LayeredNonTreeEdges* non_tree = comp_info.non_tree_edges.get();
    
    // 从最高层向下查找
    for (int level = non_tree->max_level; level >= 0; level--) {
        const auto& edges_at_level = non_tree->getEdgesAtLevel(level);
        
        std::vector<unsigned long long> edges_to_demote;
        
        for (unsigned long long key : edges_at_level) {
            int e_u, e_v;
            decodeEdgeKey(key, e_u, e_v);
            
            if (!row_active[e_u] || !row_active[e_v]) {
                edges_to_demote.push_back(key);
                continue;
            }
            
            int root_eu = ett->getComponentId(e_u);
            int root_ev = ett->getComponentId(e_v);
            
            // 找到跨分量的边，作为替代边
            if (root_eu != root_ev) {
                return std::make_pair(e_u, e_v);
            }
            
            // 如果两端点在同一分量，降级
            if (root_eu == root_ev) {
                edges_to_demote.push_back(key);
            }
        }
        
        // 执行降级
        for (unsigned long long key : edges_to_demote) {
            // int original_level = non_tree->getLevel(key);
            non_tree->demoteEdge(key);
            
            if (edge_info_map.count(key)) {
                edge_info_map[key].level = level - 1;
            }
            
            // 记录降级操作（用于反向恢复）
            // demoted_edges.push_back({key, original_level});
        }
    }
    
    return std::nullopt;
}

std::vector<Block> ComponentDetector::convertComponentsToBlocks(
    const std::vector<std::set<int>>& components) {
    
    std::vector<Block> blocks;
    blocks.reserve(components.size());
    
    for (const auto& comp_rows : components) {
        std::set<int> component_cols;
        std::vector<int> component_rows_vec(comp_rows.begin(), comp_rows.end());
        
        for (int r : comp_rows) {
            auto it = row_to_cols.find(r);
            if (it != row_to_cols.end()) {
                for (int col : it->second) {
                    if (active_cols.count(col)) {
                        component_cols.insert(col);
                    }
                }
            }
        }
        
        if (!component_rows_vec.empty()) {
            blocks.emplace_back(std::move(component_rows_vec), 
                               std::move(component_cols));
        }
    }
    
    return blocks;
}

// vector<Block> ComponentDetector::detect_blocks(const set<int> &block_rows) {
//     if (block_rows.empty()) return {}; // 如果没有行，则返回空集合
//     std::shared_lock<std::shared_mutex> lock(state_mutex);

//     // 创建局部 Union-Find
//     std::unordered_map<int, int> local_parent;
//     for (int r : block_rows) {
//         local_parent[r] = r;
//     }
    
//     // 迭代版本的 find（带路径压缩）
//     auto local_find = [&](int x) -> int {
//         int root = x;
//         // 找到根节点
//         while (local_parent[root] != root) {
//             root = local_parent[root];
//         }
//         // 路径压缩
//         while (x != root) {
//             int next = local_parent[x];
//             local_parent[x] = root;
//             x = next;
//         }
//         return root;
//     };
    
//     auto local_unite = [&](int x, int y) {
//         x = local_find(x);
//         y = local_find(y);
//         if (x != y) {
//             local_parent[x] = y;
//         }
//     };
    
//     std::unordered_map<int, std::vector<int>> col_to_active_rows_in_block;
    
//     for (int r : block_rows) {
//         auto it = row_to_cols.find(r);
//         if (it == row_to_cols.end()) continue;

//         for (int c : it->second) {
//             if (active_cols.count(c)) {
//                 // 记录该列包含当前块内的哪些行
//                 // 只有当一列包含 >= 2 个当前块的行时，它才提供连通性
//                 col_to_active_rows_in_block[c].push_back(r);
//             }
//         }
//     }

//     // 根据列进行合并
//     for (const auto& [col, rows_in_col] : col_to_active_rows_in_block) {
//         if (rows_in_col.size() < 2) continue;
        
//         // 将该列下的所有行合并到第一个行
//         int first_row = rows_in_col[0];
//         for (size_t i = 1; i < rows_in_col.size(); ++i) {
//             local_unite(first_row, rows_in_col[i]);
//         }
//     }

//     // 按代表元分组
//     std::unordered_map<int, std::vector<int>> rep_to_rows_list;
//     for (int r : block_rows) {
//         rep_to_rows_list[local_find(r)].push_back(r);
//     }
    
//     // 构建 Block
//     std::vector<Block> blocks;
//     blocks.reserve(rep_to_rows_list.size());
//     for (const auto& [rep, rows] : rep_to_rows_list) {
//         std::set<int> component_cols;
//         std::vector<int> component_rows_vec = rows;
        
//         for (int r : rows) {
//             auto it = row_to_cols.find(r);
//             if (it != row_to_cols.end()) {
//                 for (int col : it->second) {
//                     if (active_cols.count(col)) {
//                         component_cols.insert(col);
//                     }
//                 }
//             }
//         }
        
//         // 只有非空的块才有意义
//         if (!component_rows_vec.empty()) {
//             blocks.emplace_back(std::move(component_rows_vec), std::move(component_cols));
//         }
//     }
    
//     return blocks;
// };
