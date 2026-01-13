#include "../include/DXD.h"

static Logger logger("../run_results.txt");  // 全局日志
// const string muti_thread_dxd_log_file = "../muti_thread_dxd_log.csv";
static const int DEFAULT_THREADS = 16;  // 线程数

// 算法类型枚举
enum class algorithm_type {
    dxd,
    mdxd
};

// 将字符串转换为枚举
algorithm_type parseAlgorithmType(const std::string& name) {
    if (name == "dxd") return algorithm_type::dxd;
    if (name == "mdxd") return algorithm_type::mdxd;
    throw std::invalid_argument("Unknown algorithm type: " + name);
}

// ./main <algorithm> <input> <read_mode> [pool_size]
int main(int argc, char *argv[]){
    
    if (argc < 4) {
            std::cout << "Usage: " << argv[0] << "<algorithm> <input> <read_mode>" << std::endl;
            return 1;
    }
    
    try
    {
        std::string algType = argv[1];
        std::string input_file = argv[2];
        int read_mode = std::stoi(argv[3]);
        bool use_ett = false;
        if (argc > 4) {
            use_ett = std::string(argv[4]) == "ett";
        }

        int num_threads = (argc > 5) ? std::stoi(argv[5]) : DEFAULT_THREADS; // 默认线程数
        bool debug = false;

        string filename = fs::path(input_file).stem().string();
        algorithm_type type = parseAlgorithmType(algType);
        switch (type) {

            case algorithm_type::dxd: 
                {
                    if (use_ett) {
                        logger.logLine("启用DXD算法求解: " + filename);
                        DanceDNNF danceDNNF(input_file, read_mode, logger, false, true, 2, debug);
                        danceDNNF.startDXD();
                        logger.logLine("DXD算法求解结束: " + filename);
                    } else {
                        logger.logLine("启用DXD算法求解: " + filename);
                        DanceDNNF danceDNNF(input_file, read_mode, logger, true, false, 2, debug);
                        danceDNNF.startDXD();
                        logger.logLine("DXD算法求解结束: " + filename);
                    }
                    break;
                }
            case algorithm_type::mdxd:
                {
                    if (use_ett) {
                        logger.logLine("启用多线程DXD算法求解: " + filename);
                        DanceDNNF danceDNNF(input_file, read_mode, logger, false, true, num_threads, debug);
                        danceDNNF.startMultiThreadDXD();
                        logger.logLine("多线程DXD算法求解结束: " + filename);
                    } else {
                        logger.logLine("启用多线程DXD算法求解: " + filename);
                        DanceDNNF danceDNNF(input_file, read_mode, logger, true, false, num_threads, debug);
                        danceDNNF.startMultiThreadDXD();
                        logger.logLine("多线程DXD算法求解结束: " + filename);
                    }
                    break;
                }

            default:
                std::cout << "Unknowed algorithm type" << std::endl;
                return 1;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "错误：" << e.what() << '\n';
    }
    
    return 0;
}