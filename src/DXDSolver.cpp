#include "../include/DXD.h"

shared_ptr<DNNFNode> DanceDNNF::buildDecisionNode(int r, shared_ptr<DNNFNode> lo, shared_ptr<DNNFNode> hi) {
    if (hi == F) {
        return lo;
    }

    std::shared_lock<std::shared_mutex> readLock(tableMutex);
    size_t key = gen_key(r, lo.get(), hi.get());
    if (node_table.find(key) != node_table.end()) {
        return node_table[key];
    }

    auto decision_node = make_shared<DNNFNode>(NodeType::Decision, lo, hi);
    node_table[key] = decision_node;
    if (dxz_mode) {
        num_of_DNNFNodes += 1;
    }
    num_of_DNNFNodes++;
    return decision_node;
}

shared_ptr<DNNFNode> DanceDNNF::buildDecomposableNode(vector<shared_ptr<DNNFNode>>& subDNNFs) {
    auto decompose_node = make_shared<DNNFNode>(NodeType::Decomposed, -3);
    decompose_node->children = subDNNFs;
    return decompose_node;
}

// 串行处理每个子块，组合为 分解 节点
DNNFResult DanceDNNF::serialSearch(vector<Block>& blocks, int parent_depth) {

    DNNFResult totalResult(1);
    
    for (auto& b : blocks) {
        auto result = DXD(b, parent_depth + 1);
        if (result.isZero()) {
            return DNNFResult(0);
        }
        
        totalResult = totalResult * result;
    }
    
    return totalResult;
}

// 开启多线程并行搜索，多个子线程继承父线程的检测结果，并求解其中一个分块
DNNFResult DanceDNNF::parallelSearchUseOmp(vector<Block>& blocks, int parent_depth) {

    const int n = blocks.size();
    std::vector<DNNFResult> results(n);
    std::atomic<bool> has_failure(false);
    std::atomic<bool> has_timeout(false);
    
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        if (has_failure.load(std::memory_order_acquire) || 
            has_timeout.load(std::memory_order_acquire)) {
            continue;
        }

        try {
            // 子线程的depth从parent_depth+1开始
            auto result = DXD(blocks[i], parent_depth + 1);

            if (result.isZero()) {
                has_failure.store(true, std::memory_order_release);
            } else {
                results[i] = result;
            }
        } catch (const std::runtime_error& e) {
            if (std::string(e.what()).find("Time bound") != std::string::npos) {
                has_timeout.store(true, std::memory_order_release);
            } else {
                has_failure.store(true, std::memory_order_release);
            }
        } catch (...) {
            has_failure.store(true, std::memory_order_release);
        }
    }
    
    if (has_timeout.load()) {
        throw std::runtime_error("Time bound broken");
    }
    
    if (has_failure.load()) {
        return DNNFResult(0);
    }
    
    // 计算总计数
    DNNFResult totalResult(1);
    for (const auto& result : results) {
        totalResult = totalResult * result;
    }
    
    return totalResult;
}


// DXD IDXD
DNNFResult DanceDNNF::DXD(Block& block, int depth) {
    
    // max_depth = std::max(max_depth, depth);

    if(timer.timeBoundBroken()) {
        throw std::runtime_error("Time bound broken");
    }
    
    if(block.cols.empty()) {
        return DNNFResult(1);
    } 

    // 先查缓存
    size_t state = hashBlockState(block.cols); 
    {
        std::shared_lock<std::shared_mutex> readLock(cacheMutex);
        auto it = countCache.find(state);
        if (it != countCache.end()) {
            return it->second;
        }
    }

    if (block.rows.size() > 2 && isGraphSyncEnabled()) {
        
        vector<Block> curBlock;
        if (useETT ) {
            curBlock = detector->GetBlocks(block.rows);
        } else if (useIG) {
            curBlock = getComponentsByIG(block.rows);
        }
        // addRecordCount();

        MAX_B_COUNT = std::max(MAX_B_COUNT, curBlock.size());

        if (curBlock.size() > 1) {
            // 检测到多个独立分块，则并行处理
            if(useETT && isParallelSearch) turnOffGraphSync();

            DNNFResult result;

            if (isParallelSearch) {
                result = parallelSearchUseOmp(curBlock, depth);
            } else {
                result = serialSearch(curBlock, depth);
            }
            setCacheCount(state, result);
            return result;
        } 
        
        // if (getRecordCount() > MAX_DECOMPOSE_TIMES) {
        //     turnOffGraphSync();
        // }
        // 如果只有一个分块，则直接求解
    }

    ColumnHeader* choose = selectOptimalColumn(block.cols); 

    if(choose->size <= 0) {
        setCacheCount(state, DNNFResult(0));
        return DNNFResult(0);
    }

    // 将choose列下的行节点作为Decision节点加入children

    DNNFResult totalResult(0);
    shared_ptr<DNNFNode> x = F;

    set<int> deleted_rows;
    coverInBlock(choose->col, block, deleted_rows);
    detector->DeleteEdges(deleted_rows);

    Node* curC = choose->down;
    while(curC != choose) {
        
        Node* curR = curC->right;
        set<int> deleted_rows_;
        while (curR != curC) {
            coverInBlock(curR->col, block, deleted_rows_);
            curR = curR->right;
        }
        detector->DeleteEdges(deleted_rows_);
 
        auto result = DXD(block, depth + 1);

        if(!result.isZero()) {
            auto y = make_shared<DNNFNode>(NodeType::Decision, curR->row, result.count);
            buildDecisionNode(curR->row, x, y);
            totalResult = totalResult + result;
        }
        
        curR = curC->left;
        while (curR != curC) {
            uncoverInBlock(curR->col, block);
            curR = curR->left;
        }
        detector->AddEdges(deleted_rows_);
        curC = curC->down;
    }
    uncoverInBlock(choose->col, block);
    detector->AddEdges(deleted_rows);

    // 插入缓存
    setCacheCount(state, totalResult);

    return totalResult;
}


void DanceDNNF::startDXD() {

    if(!controlOUTPUT)  logger.logLine("开始单线程DXD搜索...");

    isParallelSearch = false; // 单线程搜索
    MAX_B_COUNT = 1;
    if(!dxz_mode) {
        single_thread_mode = true; // 启用单线程模式
    }
    
    try{

        timer.reset();
        timer.markStartTime();
        auto start = std::chrono::high_resolution_clock::now();
        auto ResSols = DXD(InitBlock, 1);  
        auto end = std::chrono::high_resolution_clock::now();
        timer.markStopTime();

        searchTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        logger.logLine("Time: " + std::to_string(searchTime) + " s");
        timeout = false;

        solutionCount = ResSols.toString();
        logger.logLine("Solutions: " + solutionCount);
    
        if(!controlOUTPUT) logger.logLine("Max Blocks: " + std::to_string(MAX_B_COUNT));

        if(dxz_mode) {
            logger.logLine("ZDD Size: " + std::to_string(num_of_DNNFNodes + max_depth));
        } else {
            logger.logLine("DNNF Size: " + std::to_string(num_of_DNNFNodes));
        }

        return;
    } catch (std::runtime_error &e) {
        timeout = true;
        if(!controlOUTPUT) logger.logLine("DXD搜索超时: " + std::string(e.what()));
        return;
    }

}

void DanceDNNF::startMultiThreadDXD() {

    logger.logLine("开始多线程DXD搜索...");
    
    isParallelSearch = true;  // 开启多线程搜索标志
    MAX_B_COUNT = 1;

    try {

        timer.reset();
        timer.markStartTime();
        auto start = std::chrono::high_resolution_clock::now();
        auto ResSols = DXD(InitBlock, 1);  // 多线程DXD搜索
        auto end = std::chrono::high_resolution_clock::now();
        timer.markStopTime();
   
        searchTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        logger.logLine("Time: " + std::to_string(searchTime) + " s");
        timeout = false;

        solutionCount = ResSols.toString();
        logger.logLine("Solutions: " + solutionCount);
    
        logger.logLine("Max Blocks: " + std::to_string(MAX_B_COUNT));
        if(dxz_mode) {
            logger.logLine("ZDD Size: " + std::to_string(num_of_DNNFNodes + max_depth));
        } else {
            logger.logLine("DNNF Size: " + std::to_string(num_of_DNNFNodes));
        }
        return;
    } catch (std::runtime_error &e) {
        timeout = true;
        logger.logLine("DXD搜索超时: " + std::string(e.what()));
        return;
    }
}

DNNFResult DanceDNNF::parallelSearchMDLX(vector<Block>& blocks) {
    const int n = blocks.size();
    
    // 并行处理每个子块
    std::vector<DNNFResult> results(n, 0);
    std::atomic<bool> has_timeout(false);
    std::atomic<bool> has_failure(false);
    

    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        // 提前检查超时标志
        if (has_timeout.load(std::memory_order_acquire) ||
             has_failure.load(std::memory_order_acquire)) {
            continue;
        }
        
        try {
            // 每个线程使用block的副本和独立的sols
            Block blockCopy = blocks[i];
            vector<int> threadSols;
            
            // 递归调用MDLX（每个线程独立计数）
            auto result = MDLX(threadSols, blockCopy);
            if (result.isZero()) {
                has_failure.store(true, std::memory_order_release);
            } else {
                results[i] = result;
            }
            
        } catch (const std::runtime_error& e) {
            // 捕获超时异常
            if (std::string(e.what()).find("Time out") != std::string::npos) {
                has_timeout.store(true, std::memory_order_release);
            } else {
                has_failure.store(true, std::memory_order_release);
            }
        } catch (...) {
            has_failure.store(true, std::memory_order_release);
        }
    }

    
    // 检查超时（在主线程重新抛出）
    if (has_timeout.load()) {
        throw std::runtime_error("Time out");
    }

    // 检查失败（在主线程重新抛出）
    if (has_failure.load()) {
        return DNNFResult(0);
    }
    
    // 合并结果：计算所有子块解的笛卡尔积
    DNNFResult totalCount(1);
    for (const auto& result : results) {
        totalCount = totalCount * result;
    }
    
    return totalCount;
}

DNNFResult DanceDNNF::MDLX(vector<int>& sols, Block& block) {

    if (timer.timeBoundBroken()) {
        throw std::runtime_error("Time out");
    }

    if( block.cols.empty() ) {
        return DNNFResult(1);
    }
    
    if(isGraphSyncEnabled() && block.rows.size() >= MIN_BLOCK_ROWS) {

        vector<Block> curBlock = getComponentsByIG(block.rows);

        MAX_B_COUNT = std::max(MAX_B_COUNT, curBlock.size());
        if (curBlock.size() > 1) {
            
            turnOffGraphSync(); // 关闭图同步，提升性能
            p_count.fetch_add(1);

            // 多线程搜索
            auto parallelCount = parallelSearchMDLX(curBlock);
            return parallelCount;
        } 
    }

    ColumnHeader* choose = selectColumnHeuristic(block.cols);
    if( !choose || choose->size <= 0 ) {
        return DNNFResult(0);  
    }

    DNNFResult totalResult = DNNFResult(0);

    set<int> deleted_rows;
    coverInBlock( choose->col, block, deleted_rows );
    Node* curC = choose->down;  

    while( curC != choose )  
    {  
         
        Node* curR = curC->right;  
        while( curR != curC )  
        {  
            coverInBlock( curR->col, block, deleted_rows );  
            curR = curR->right;  
        }  

        sols.push_back(curC->row + 1); 
        // 递归搜索
        auto result = MDLX(sols, block);
        if (!result.isZero()) {
            totalResult = totalResult + result;
        }
       
        sols.pop_back();  // 回溯，移除当前行
        curR = curC->left;  
        while( curR != curC )  
        {  
            uncoverInBlock( curR->col, block );  
            curR = curR->left;  
        }  
        curC = curC->down;  
    }  
    uncoverInBlock( choose->col, block );  
    return totalResult; 
}

void DanceDNNF::start_MDLX_Search() {

    logger.logLine("开始多线程DLX搜索...");

    p_count = 0;
    MAX_B_COUNT = 1;
    
    try {
        vector<int> sols;

        timer.reset();
        timer.markStartTime();
        auto start = std::chrono::high_resolution_clock::now();
        auto res = MDLX(sols, InitBlock);
        auto end = std::chrono::high_resolution_clock::now();
        timer.markStopTime();

        searchTime = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        logger.logLine("Time: " + std::to_string(searchTime) + " s");
        timeout = false;

        solutionCount = res.toString();
        logger.logLine("Solutions: " + solutionCount);

        logger.logLine("Max Blocks: " + std::to_string(MAX_B_COUNT));
        return;
    } catch (std::runtime_error &e) {
        timeout = true;
        logger.logLine("MDLX搜索超时: " + std::string(e.what()));
        return;
    } 
}



