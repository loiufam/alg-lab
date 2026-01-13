#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <vector>
#include <memory>
#include <atomic>
#include <random>
#include <chrono>
#include <type_traits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <climits>
#include <execution>

const size_t MAX_THREADS = std::thread::hardware_concurrency();

// ========================
// 无锁环形队列实现
// ========================
template<typename T>
class ThreadSafeQueue {
private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
    std::atomic<bool> shutdown_{false};
    const size_t max_size_;

public:
    explicit ThreadSafeQueue(size_t max_size = 1024) : max_size_(max_size) {}
    
    bool push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_.load() || queue_.size() >= max_size_) {
            return false;
        }
        queue_.push(std::move(item));
        condition_.notify_one();
        return true;
    }
    
    bool pop(T& result) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !queue_.empty() || shutdown_.load(); });
        
        if (shutdown_.load() && queue_.empty()) {
            return false;
        }
        
        if (!queue_.empty()) {
            result = std::move(queue_.front());
            queue_.pop();
            return true;
        }
        return false;
    }
    
    bool try_pop(T& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        result = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    
    void shutdown() {
        shutdown_.store(true);
        condition_.notify_all();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
};

// ========================
// 线程池实现
// ========================
class ThreadPool {
public:
    // 构造函数，指定线程数量
    explicit ThreadPool(size_t threads = std::thread::hardware_concurrency()) 
        : stop(false) {
        if (threads == 0 || threads > MAX_THREADS) {
            threads = std::thread::hardware_concurrency();
        }
        
        // 预分配线程向量空间，避免重分配
        workers.reserve(threads);
        
        // 创建工作线程
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                this->worker_thread();
            });
        }
    }

    // 禁用拷贝构造和赋值
    // ThreadPool(const ThreadPool&) = delete;
    // ThreadPool& operator=(const ThreadPool&) = delete;

    // 允许移动构造和赋值
    ThreadPool(ThreadPool&&) = default;
    ThreadPool& operator=(ThreadPool&&) = default;

    // 析构函数，确保所有资源正确释放
    ~ThreadPool() {
        shutdown();
    }

    // 提交任务到线程池
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::invoke_result_t<F, Args...>> {
        
        using return_type = typename std::invoke_result_t<F, Args...>;

        // 创建打包任务
        auto task = std::make_shared<std::packaged_task<return_type()>>(
                [f = std::forward<F>(f), args = std::make_tuple(std::forward<Args>(args)...)]() mutable -> return_type {
                    return std::apply(std::move(f), std::move(args));
            }
        );

        std::future<return_type> res = task->get_future();

        // 线程池已停止，抛出异常
        if (stop.load()) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }

        // 将任务添加到队列
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            
            // 再次检查停止状态（双重检查）
            if (stop.load()) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            
            tasks.emplace([task]() { (*task)(); });
        }
        
        condition.notify_one();
        return res;
    }

    // 获取当前队列中任务数量
    size_t pending_tasks() const {
        std::unique_lock<std::mutex> lock(queue_mutex);
        return tasks.size();
    }

    // 获取线程池大小
    size_t size() const {
        return workers.size();
    }

    // 等待所有任务完成
    void wait_for_tasks() {
        std::unique_lock<std::mutex> lock(queue_mutex);
        finished_condition.wait(lock, [this] {
            return tasks.empty() && (active_threads == 0);
        });
    }

    // 优雅关闭线程池
    void shutdown() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (stop.load()) {
                return; // 已经关闭
            }
            stop = true;
        }
        
        condition.notify_all();
        
        // 等待所有工作线程结束
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        
        workers.clear();
    }

    // 立即停止（不等待当前任务完成）
    void force_shutdown() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
            // 清空任务队列
            std::queue<std::function<void()>> empty;
            tasks.swap(empty);
        }
        
        condition.notify_all();
        
        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        
        workers.clear();
    }

private:
    // 工作线程函数
    void worker_thread() {
        while (true) {
            std::function<void()> task;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                
                condition.wait(lock, [this] {
                    return stop.load() || !tasks.empty();
                });
                
                if (stop.load() && tasks.empty()) {
                    return;
                }
                
                if (!tasks.empty()) {
                    task = std::move(tasks.front());
                    tasks.pop();
                    ++active_threads;
                }
            }
            
            if (task) {
                try {
                    task(); // 执行任务
                } catch (...) {
                    // 捕获并忽略任务执行中的异常，避免线程终止
                    // 实际应用中可能需要日志记录
                }
                
                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    --active_threads;
                }
                finished_condition.notify_all();
            }
        }
    }

    // 线程向量
    std::vector<std::thread> workers;
    
    // 任务队列
    std::queue<std::function<void()>> tasks;
    
    // 同步原语
    mutable std::mutex queue_mutex;
    std::condition_variable condition;
    std::condition_variable finished_condition;
    
    // 停止标志
    std::atomic<bool> stop;
    
    // 活跃线程计数
    std::atomic<size_t> active_threads{0};
};

class ThreadPoolManager {
public:
    static ThreadPool& get_instance(size_t threads = 0) {
        static ThreadPool instance(threads == 0 ? std::thread::hardware_concurrency() : threads);
        return instance;
    }

private:
    ThreadPoolManager() = default;
};

// 便利函数：异步执行任务
template<class F, class... Args>
auto async_execute(F&& f, Args&&... args) 
    -> std::future<typename std::invoke_result_t<F, Args...>> {
    return ThreadPoolManager::get_instance().enqueue(
        std::forward<F>(f), 
        std::forward<Args>(args)...
    );
}

#endif