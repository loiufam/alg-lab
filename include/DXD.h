#ifndef DXD_H
#define DXD_H

#include "../include/DancingMatrix.h"
#include "../include/DXDTime.h"

const int MIN_BLOCK_ROWS = 20;
const int MAX_BLOCK_ROWS = 200;
const int TIME_LIMIT_SECONDS = 1200; 
const int TIME_LIMIT_BUILDING_SECONDS = 1200;
const int MAX_DECOMPOSE_TIMES = 5;
using namespace std;

enum class NodeType { OR, Decision, Decomposed, Variable, Terminal };  // 节点类型 AND node 分为Decision和Decomposed两种

struct DNNFNode {
    NodeType type;
    int label; // -1 for T, -2 for F
    uint64_t count;
    vector<std::shared_ptr<DNNFNode>> children; // 记录行id
    std::shared_ptr<DNNFNode> left;
    std::shared_ptr<DNNFNode> right;

    DNNFNode() = default;
    DNNFNode(NodeType t, int l) : type(t), label(l) {} // 构建变量节点和终端节点
    DNNFNode(NodeType t, int l, uint64_t c) : type(t), label(l), count(c) {} // 构造函数用于OR节点和 Decomposed类型的AND节点   
    DNNFNode(NodeType t, shared_ptr<DNNFNode> l, shared_ptr<DNNFNode> r) : type(t), left(l), right(r) {} // 构建Decision类型的AND节点
    DNNFNode(int l, shared_ptr<DNNFNode> r, uint64_t c)  : type(NodeType::Decision), label(l), right(r), count(c) {} // 简化版决策节点

};

class DanceDNNF : DancingMatrix { 

    public:
        DanceDNNF(int rows, int cols, int** matrix, Logger& l, bool verbose = false, int pool_size = 8) 
            : DancingMatrix(rows, cols, matrix, verbose), logger(l) {
            
 
            timer.setTimeBound(TIME_LIMIT_SECONDS);

            std::cout<< "初始化DanceDNNF完成." << endl;
        }

        DanceDNNF(const string& file_path, int from, Logger& l, 
                       const bool useIG = false, const bool useETT = false, int pool_size = 8, bool debug = false)
            : DancingMatrix(file_path, from, useIG, useETT), 
            logger(l), 
            max_threads(pool_size), 
            debug(debug), 
            num_of_DNNFNodes(0),
            max_depth(1) {

            if(pool_size > 1) {
                omp_set_num_threads(pool_size); // 设置并行线程数
            }
            
            timer.setTimeBound(TIME_LIMIT_SECONDS);
        }

        ~DanceDNNF() = default;

        CStopWatch timer;   // 计时器

        const int MAX_P_COUNT = 1; // 最大并行搜索次数   
        atomic<int> p_count{0}; // 记录当前并行的子进程数
        int detect_record = 0; // 记录第几次检测
        size_t MAX_B_COUNT;
        int max_threads; // 最大线程数

        double searchTime = 0.0;
        string solutionCount; // 记录解的数量
        bool timeout = false; // 是否超时
        bool isParallelSearch = false; // 是否并行搜索
        double decomposeTime = 0.0;
        bool debug = false;

        // 构建Decision-ZDNNF
        shared_ptr<DNNFNode> buildDecisionNode(int r, shared_ptr<DNNFNode> lo, shared_ptr<DNNFNode> hi);

        shared_ptr<DNNFNode> buildDecomposableNode(vector<shared_ptr<DNNFNode>>& subDNNFs);

        // 多线程DLX
        DNNFResult parallelSearchMDLX(vector<Block>& blocks);
        DNNFResult MDLX(vector<int>& sols, Block& block);

        DNNFResult DXD(Block& block, int depth);
        DNNFResult serialSearch(vector<Block>& blocks, int depth);
        DNNFResult parallelSearchUseOmp(vector<Block>& blocks, int depth);

        // 启动搜索函数
        void startDXD();
        void startMultiThreadDXD();
        void start_MDLX_Search();

        bool queryRecord (size_t key) {
            std::shared_lock<std::shared_mutex> readLock(recordMutex);
            return records.find(key) != records.end();
        }

        void insertRecord (size_t key) {
            std::unique_lock<std::shared_mutex> writeLock(recordMutex);
            records.insert(key);
        }

        Block getBlock() {
            Block fullBlock(rowsSet, colsSet);
            return fullBlock;
        };

        std::shared_ptr<DNNFNode> getCache(const size_t& key){
            std::shared_lock<std::shared_mutex> readLock(cacheMutex);
            if(C.find(key) != C.end()){
                return C[key];
            }
            return nullptr;
        }

        void setCache(const size_t& key, std::shared_ptr<DNNFNode> node){
            std::unique_lock<std::shared_mutex> writeLock(cacheMutex);
            if(C.find(key) == C.end()){
                C[key] = node;
            }
        }

        void setCacheCount(const size_t& key, DNNFResult count){
            std::unique_lock<std::shared_mutex> writeLock(cacheMutex);
            countCache[key] = count;
        }

        int getRecordCount(){
            std::shared_lock<std::shared_mutex> readLock(recordMutex);
            return detect_record;
        }

        void addRecordCount(){
            std::unique_lock<std::shared_mutex> writeLock(recordMutex);
            detect_record++;
        }

        void runDXZ() {
            turnOffGraphSync();
            controlOUTPUT = true;
            dxz_mode = true;
            startDXD();
        }

        inline size_t gen_key(int r, DNNFNode* x, DNNFNode* y) {
            return std::hash<int>()(r) ^ (std::hash<int>()(x->label) << 1) ^ (std::hash<int>()(y->label) << 2);
        }

    private:
        // ThreadPool& pool;
        Logger& logger;
        bool controlOUTPUT = false;
        
        // DNNF相关
        int num_of_DNNFNodes;
        int max_depth;
        vector<string> cache_input_order; // 记录缓存的输入顺序，便于输出
        std::shared_ptr<DNNFNode> rootDNNF;
        std::unordered_set<size_t> records; // 用于记录无法分解的矩阵状态
        std::shared_ptr<DNNFNode> T = std::make_shared<DNNFNode>(NodeType::Terminal, -1, 1);
        std::shared_ptr<DNNFNode> F = std::make_shared<DNNFNode>(NodeType::Terminal, -2, 0);
        
        // DNNF缓存
        mutable std::shared_mutex cacheMutex;
        mutable std::shared_mutex recordMutex; // 记录互斥锁
        unordered_map<size_t, shared_ptr<DNNFNode>> C;

        // DNNF Nodes table
        mutable std::shared_mutex tableMutex;
        unordered_map<size_t, shared_ptr<DNNFNode>> node_table;

        // 轻量级缓存：只存计数
        unordered_map<size_t, DNNFResult> countCache;

        ThreadPool& getThreadPool(int poolSize) {
            return ThreadPoolManager::get_instance(poolSize);
        }
};

#endif