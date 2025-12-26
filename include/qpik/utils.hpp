#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <chrono>
#include <unordered_map>
#include <iomanip>
#include <string>

namespace qpik::utils {
/**
 * 时间测量工具类
 * 提供类似 MATLAB 的 tic/toc 功能
 */
class TimeMeasure {
  public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::milli>;

    /**
     * 开始计时
     * @param tag 计时器标签，默认为 "default"
     */
    static void tic(const std::string &tag = "default") {
        start_times_[tag] = Clock::now();
    }

    /**
     * 结束计时并返回经过的时间（毫秒）
     * @param tag 计时器标签，默认为 "default"
     * @return 经过的时间（毫秒）
     */
    static double toc(const std::string &tag = "default") {
        auto it = start_times_.find(tag);
        if (it == start_times_.end()) {
            std::cerr << "警告: 未找到标签 '" << tag
                      << "' 的计时器，请先调用 tic()" << std::endl;
            return -1.0;
        }

        auto end_time = Clock::now();
        Duration duration =
            std::chrono::duration_cast<Duration>(end_time - it->second);
        return duration.count();
    }

    /**
     * 结束计时并打印经过的时间
     * @param tag 计时器标签，默认为 "default"
     * @param message 可选的提示信息
     */
    static void toc_print(
        const std::string &tag = "default", const std::string &message = "") {
        double elapsed = toc(tag);
        if (elapsed >= 0) {
            if (!message.empty()) {
                std::cout << message << ": ";
            } else {
                std::cout << "[" << tag << "] ";
            }
            std::cout << std::fixed << std::setprecision(3) << elapsed << " ms"
                      << std::endl;
        }
    }

    /**
     * 清除指定标签的计时器
     * @param tag 计时器标签
     */
    static void clear(const std::string &tag = "default") {
        start_times_.erase(tag);
    }

    /**
     * 清除所有计时器
     */
    static void clearAll() { start_times_.clear(); }

  private:
    static std::unordered_map<std::string, TimePoint> start_times_;
};

// 静态成员变量定义
std::unordered_map<std::string, TimeMeasure::TimePoint>
    TimeMeasure::start_times_;

/**
 * 作用域计时器类
 * 使用 RAII 模式，在构造时开始计时，在析构时自动结束计时
 *
 * 使用示例:
 *   {
 *       ScopedTimer timer("函数执行");
 *       // 执行一些代码
 *   } // 离开作用域时自动打印时间
 */
class ScopedTimer {
  public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::milli>;

    /**
     * 构造函数，开始计时
     * @param message 计时器消息，将在析构时打印
     * @param auto_print 是否在析构时自动打印，默认为 true
     */
    ScopedTimer(const std::string &message = "", bool auto_print = true)
        : message_(message.empty() ? "作用域计时" : message)
        , start_time_(Clock::now())
        , auto_print_(auto_print) {}

    /**
     * 析构函数，自动结束计时并打印（如果启用）
     */
    ~ScopedTimer() {
        if (auto_print_) {
            auto end_time = Clock::now();
            Duration duration =
                std::chrono::duration_cast<Duration>(end_time - start_time_);
            double elapsed = duration.count();

            if (!message_.empty()) {
                std::cout << message_ << ": ";
            } else {
                std::cout << "作用域计时: ";
            }
            std::cout << std::fixed << std::setprecision(3) << elapsed << " ms"
                      << std::endl;
        }
    }

    /**
     * 手动获取当前经过的时间（不结束计时）
     * @return 经过的时间（毫秒）
     */
    double elapsed() const {
        auto end_time = Clock::now();
        Duration duration =
            std::chrono::duration_cast<Duration>(end_time - start_time_);
        return duration.count();
    }

    /**
     * 禁用自动打印
     */
    void disableAutoPrint() { auto_print_ = false; }

    /**
     * 启用自动打印
     */
    void enableAutoPrint() { auto_print_ = true; }

    // 禁止拷贝和赋值
    ScopedTimer(const ScopedTimer &) = delete;
    ScopedTimer &operator=(const ScopedTimer &) = delete;

  private:
    std::string message_;
    TimePoint start_time_;
    bool auto_print_;
};

} // namespace qpik::utils

#endif // UTILS_HPP
