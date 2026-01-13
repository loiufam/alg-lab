#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <map>
#include <bitset>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <stack>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <shared_mutex>
#include <utility>
#include <limits>
#include <omp.h>
#include <cstdint>

using namespace std;
namespace fs = std::filesystem;


// 签名用于Memo Cache
struct Signature {
    vector<bool> covered;  // 哪些列已被覆盖
    
    bool operator==(const Signature& other) const {
        return covered == other.covered;
    }
};

// 哈希函数
struct SignatureHash {
    size_t operator()(const Signature& sig) const {
        size_t h = 0;
        for (size_t i = 0; i < sig.covered.size(); i++) {
            if (sig.covered[i]) h ^= (i + 1) * 2654435761u;
        }
        return h;
    }
};

// 科学计数法结构体
struct ScientificCount {
    long double mantissa;
    int exponent;
    
    ScientificCount() : mantissa(1.0L), exponent(0) {}
    ScientificCount(uint64_t value) {
        if (value == 0) {
            mantissa = 0.0L;
            exponent = 0;
        } else {
            mantissa = static_cast<long double>(value);
            exponent = 0;
            normalize();
        }
    }
    
    void normalize() {
        if (mantissa == 0.0L) {
            exponent = 0;
            return;
        }
        while (std::abs(mantissa) >= 10.0L) {
            mantissa /= 10.0L;
            exponent++;
        }
        while (std::abs(mantissa) < 1.0L && mantissa != 0.0L) {
            mantissa *= 10.0L;
            exponent--;
        }
    }
    
    bool isZero() const {
        return mantissa == 0.0L;
    }
    
    std::string toString(int precision = 6) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << mantissa;
        if (exponent >= 0) {
            oss << "e+" << exponent;
        } else {
            oss << "e" << exponent;
        }
        return oss.str();
    }
    
    // 乘法操作
    ScientificCount operator*(const ScientificCount& other) const {
        ScientificCount result;
        result.mantissa = this->mantissa * other.mantissa;
        result.exponent = this->exponent + other.exponent;
        result.normalize();
        return result;
    }
    
    ScientificCount operator*(uint64_t value) const {
        if (value == 0) {
            return ScientificCount(0);
        }
        ScientificCount result;
        result.mantissa = this->mantissa * value;
        result.exponent = this->exponent;
        result.normalize();
        return result;
    }
    
    // 加法操作（用于同指数级的相加）
    ScientificCount operator+(const ScientificCount& other) const {
        if (this->isZero()) return other;
        if (other.isZero()) return *this;
        
        ScientificCount result;
        int expDiff = this->exponent - other.exponent;
        
        if (expDiff > 15 || expDiff < -15) {
            // 指数差距太大，直接返回较大的那个
            return (this->exponent > other.exponent) ? *this : other;
        }
        
        if (expDiff >= 0) {
            result.mantissa = this->mantissa * std::pow(10.0L, expDiff) + other.mantissa;
            result.exponent = other.exponent;
        } else {
            result.mantissa = this->mantissa + other.mantissa * std::pow(10.0L, -expDiff);
            result.exponent = this->exponent;
        }
        result.normalize();
        return result;
    }
};

struct Result {
    uint64_t count;          // 精确计数（未溢出时使用）
    bool overflowed;         // 是否溢出
    ScientificCount sciCount; // 科学计数法（溢出后使用）

    
    // 从精确值构造
    Result(uint64_t c) : count(c), overflowed(false), sciCount(c) {}
    
    // 从科学计数法构造
    Result(const ScientificCount& sc) : count(0), overflowed(true), sciCount(sc) {}
    
    // 默认构造（零值）
    Result() : count(0), overflowed(false), sciCount(0) {}
    
    bool isZero() const {
        return overflowed ? sciCount.isZero() : (count == 0);
    }
    
    bool isFailure() const { 
        return isZero(); 
    }
    
    std::string toString() const {
        if (overflowed) {
            return sciCount.toString();
        } else {
            return std::to_string(count);
        }
    }
    
    // 乘法操作
    Result operator*(const Result& other) const {
        if (this->isZero() || other.isZero()) {
            return Result(0);
        }
        
        // 如果任意一方已经溢出，使用科学计数法
        if (this->overflowed || other.overflowed) {
            ScientificCount result = this->sciCount * other.sciCount;
            return Result(result);
        }
        
        // 检查是否会溢出
        if (this->count <= ULLONG_MAX / other.count) {
            return Result(this->count * other.count);
        } else {
            // 溢出，转换为科学计数法
            ScientificCount result = this->sciCount * other.sciCount;
            return Result(result);
        }
    }
    
    // 加法操作
    Result operator+(const Result& other) const {
        if (this->isZero()) return other;
        if (other.isZero()) return *this;
        
        // 如果任意一方已经溢出，使用科学计数法
        if (this->overflowed || other.overflowed) {
            ScientificCount result = this->sciCount + other.sciCount;
            return Result(result);
        }
        
        // 检查是否会溢出
        if (this->count <= ULLONG_MAX - other.count) {
            return Result(this->count + other.count);
        } else {
            // 溢出，转换为科学计数法
            ScientificCount result = this->sciCount + other.sciCount;
            return Result(result);
        }
    }
};

// 轻量级结果结构
struct DNNFResult {
    uint64_t count;          // 精确计数（未溢出时使用）
    bool overflowed;         // 是否溢出
    ScientificCount sciCount; // 科学计数法（溢出后使用）
    
    // 从精确值构造
    DNNFResult(uint64_t c) : count(c), overflowed(false), sciCount(c) {}
    
    // 从科学计数法构造
    DNNFResult(const ScientificCount& sc) : count(0), overflowed(true), sciCount(sc) {}
    
    // 默认构造（零值）
    DNNFResult() : count(0), overflowed(false), sciCount(0) {}
    
    bool isZero() const {
        return overflowed ? sciCount.isZero() : (count == 0);
    }
    
    bool isFailure() const { 
        return isZero(); 
    }
    
    std::string toString() const {
        if (overflowed) {
            return sciCount.toString();
        } else {
            return std::to_string(count);
        }
    }
    
    // 乘法操作
    DNNFResult operator*(const DNNFResult& other) const {
        if (this->isZero() || other.isZero()) {
            return DNNFResult(0);
        }
        
        // 如果任意一方已经溢出，使用科学计数法
        if (this->overflowed || other.overflowed) {
            ScientificCount result = this->sciCount * other.sciCount;
            return DNNFResult(result);
        }
        
        // 检查是否会溢出
        if (this->count <= ULLONG_MAX / other.count) {
            return DNNFResult(this->count * other.count);
        } else {
            // 溢出，转换为科学计数法
            ScientificCount result = this->sciCount * other.sciCount;
            return DNNFResult(result);
        }
    }
    
    // 加法操作
    DNNFResult operator+(const DNNFResult& other) const {
        if (this->isZero()) return other;
        if (other.isZero()) return *this;
        
        // 如果任意一方已经溢出，使用科学计数法
        if (this->overflowed || other.overflowed) {
            ScientificCount result = this->sciCount + other.sciCount;
            return DNNFResult(result);
        }
        
        // 检查是否会溢出
        if (this->count <= ULLONG_MAX - other.count) {
            return DNNFResult(this->count + other.count);
        } else {
            // 溢出，转换为科学计数法
            ScientificCount result = this->sciCount + other.sciCount;
            return DNNFResult(result);
        }
    }
};

class Logger 
{
    private:
        std::ofstream logFile;
        bool enableConsole;
    
    public:
        Logger(const std::string& filename, bool console = true) 
            : enableConsole(console) {
            logFile.open(filename, std::ios::out | std::ios::trunc);
        }
        
        ~Logger() {
            if (logFile.is_open()) {
                logFile.close();
            }
        }
        
        template<typename T>
        void log(const T& message) {
            if (logFile.is_open()) {
                logFile << message;
                logFile.flush();
            }
            if (enableConsole) {
                std::cout << message;
            }
        }
        
        template<typename T>
        void logLine(const T& message) {
            log(message);
            if (logFile.is_open()) {
                logFile << std::endl;
            }
            if (enableConsole) {
                std::cout << std::endl;
            }
        }
        
        void enableConsoleOutput(bool enable) {
            enableConsole = enable;
        }
};

class PreProccess
{
    public:
        //构造函数
        PreProccess() {};
        //处理exact_cover_benchmark文件
        static int** processFileToMatrix1(const std::string& filename, int& r, int& c);
        //释放内存
        static void freeMatrix(int** matrix, int rows);
        //从字符串中提取n和m的值
        static void extractNM(const std::string& line, int& n, int& m);
        //处理set_partitioning_benchmarks文件
        static int** processFileToMatrix2(const fs::path& filename, int& r, int& c);
        // 处理d3x数据集
        static int** processFileToMatrix3(const std::string& filename, int& r, int& c);
};

#endif // COMMON_H