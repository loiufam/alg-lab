#include "../include/DXD.h"

shared_ptr<DNNFNode> DanceDNNF::buildDecisionNode(int r, shared_ptr<DNNFNode> lo, shared_ptr<DNNFNode> hi) {
    if (hi == F) {
        return lo;
    }

    size_t key = gen_key(r, lo.get(), hi.get());

    {    
        std::shared_lock<std::shared_mutex> readLock(tableMutex);
        auto it = node_table.find(key);
        if (it != node_table.end()) {
            return it->second;
        }
    }

    std::unique_lock<std::shared_mutex> writeLock(tableMutex);
    if (node_table.find(key) != node_table.end()) {
        return node_table[key];
    }

    auto decision_node = make_shared<DNNFNode>(NodeType::Decision, lo, hi);
    node_table[key] = decision_node;

    if (dxz_mode) {
        num_of_zddNodes += 2;
    }else{
        num_of_DNNFNodes += 1;
    }
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
    SubGraph* outerSubgraph = activeSubgraph_;

    std::vector<std::unique_ptr<splaytree::EulerTourTree>> stash;
    stash.swap(components);

    auto restoreStash = [&]() {
        components.clear();
        for (auto& t : stash)
            if (t) components.push_back(std::move(t));
        activeSubgraph_ = outerSubgraph;
    };

    for (size_t i = 0; i < blocks.size(); ++i) {

        if (i >= stash.size() || !stash[i]) {
            std::cerr << "serialSearch: component " << i << " missing\n";
            restoreStash();
            return DNNFResult(0);
        }

        int anyV = stash[i]->getAnyVertex();
        activeSubgraph_ = (anyV >= 0) ? graph->subgraphOf(anyV) : nullptr;

        components.push_back(std::move(stash[i]));

        auto result = DXD(blocks[i], parent_depth + 1);

        if (!components.empty()) {
            stash[i] = std::move(components[0]);
            components.clear();
        }

        if (result.isZero()) {
            restoreStash();
            return DNNFResult(0);
        }
        totalResult = totalResult * result;
    }
    restoreStash();
    
    return totalResult;
}

// 开启多线程并行搜索，多个子线程继承父线程的检测结果，并求解其中一个分块
DNNFResult DanceDNNF::parallelSearchUseOmp(vector<Block>& blocks, int parent_depth) {

    const int n = blocks.size();
    std::atomic<bool> has_failure(false);
    std::atomic<bool> has_timeout(false);

    // 准备每个线程的初始树（在主线程中）
    std::vector<std::unique_ptr<splaytree::EulerTourTree>> extracted(n);
    for (int i = 0; i < n; ++i)
        extracted[i] = std::move(components[i]);
    components.clear();

    std::vector<DNNFResult> results(n);
    // 子线程搜索完毕后，将（可能被 Dec/Inc 修改过、但已回溯还原的）树写回此处
    std::vector<std::unique_ptr<splaytree::EulerTourTree>> returned(n);
    
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n; i++) {
        if (has_failure.load(std::memory_order_acquire) || 
            has_timeout.load(std::memory_order_acquire)) {
            returned[i] = std::move(extracted[i]);
            continue;
        }

        try {
            // === 初始化线程局部状态 ===
            initThreadLocalState(blocks[i], std::move(extracted[i]));

            // === 执行搜索（自动使用线程局部数据） ===
            auto result = DXD(blocks[i], parent_depth + 1);

            if (result.isZero()) {
                has_failure.store(true, std::memory_order_release);
            } else {
                results[i] = result;
            }
        } catch (const std::runtime_error& e) {
            std::string msg = e.what();
            if (msg.find("Time bound") != std::string::npos) {
                has_timeout.store(true, std::memory_order_release);
            } else {
                has_failure.store(true, std::memory_order_release);
                std::cerr << "Thread " << omp_get_thread_num() 
                         << " error: " << msg << "\n";
            }
        } catch (const std::exception& e) {
            has_failure.store(true, std::memory_order_release);
            std::cerr << "Thread " << omp_get_thread_num() 
                     << " exception: " << e.what() << "\n";
        } catch (...) {
            has_failure.store(true, std::memory_order_release);
            std::cerr << "Thread " << omp_get_thread_num() 
                     << " unknown error\n";
        }

        cleanupThreadLocalState();
    }

    components.resize(n);
    for (int i = 0; i < n; ++i)
        components[i] = std::move(returned[i]);
    
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
    
    // std::cout << "\n============================\n";
    // std::cout << "[Before] DXD called at depth " << depth << "\n";
    // printComponents();

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

    if (block.rows.size() > 2 && shouldDecompose()) {
        
        vector<Block> curBlock;
        if (useETT ) {
            curBlock = getComponentsByETT();
        } else if (useIG) {
            curBlock = getComponentsByIG(block.rows);
        }

        MAX_B_COUNT = std::max(MAX_B_COUNT, curBlock.size());
        // addTriedNumbers(1);

        int block_size = curBlock.size();
        if (block_size  > 1) {
            num_of_DNNFNodes += block_size - 1; // 生成一个分解节点和block_size个子节点
            // std::cout << "Detected " << curBlock.size() << " independent blocks at depth " << depth << ".\n";
            // 检测到多个独立分块，则并行处理
            if(useETT && !single_thread_mode) turnOffGraphSync();
            // addConcurrentThread(block_size);

            DNNFResult result;
            if (isParallelSearch) {
                result = parallelSearchUseOmp(curBlock, depth);
            } else {
                result = serialSearch(curBlock, depth);
            }

            setCacheCount(state, result);
            return result;
        } 

    }

    ColumnHeader* choose = selectOptimalColumn(block.cols); 
    // std::cout << "Chosen column: " << choose->col << " (size: " << choose->size << ")\n";

    if(choose->size <= 0) {
        setCacheCount(state, DNNFResult(0));
        return DNNFResult(0);
    }

    // 将choose列下的行节点作为Decision节点加入children

    DNNFResult totalResult(0);
    shared_ptr<DNNFNode> x = F;

    set<int> deleted_rows;
    coverInBlock(choose->col, block, deleted_rows);
    DecUpdateCC(deleted_rows);

    Node* curC = choose->down;
    while(curC != choose) {
        
        Node* curR = curC->right;
        set<int> deleted_rows_;

        while (curR != curC) {
            coverInBlock(curR->col, block, deleted_rows_);
            curR = curR->right;
        }
        DecUpdateCC(deleted_rows_);
 
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
        IncUpdateCC(deleted_rows_);

        curC = curC->down;
    }
    uncoverInBlock(choose->col, block);
    IncUpdateCC(deleted_rows);

    // std::cout << "\n============================\n";
    // std::cout << "[After] DXD called at depth " << depth << "\n";
    // printComponents();

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
            logger.logLine("ZDD Size: " + std::to_string(num_of_zddNodes));
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
            logger.logLine("ZDD Size: " + std::to_string(num_of_zddNodes));
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



