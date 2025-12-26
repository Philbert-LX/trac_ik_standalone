/**
 * ROS2 Logger Stub for Non-ROS Builds
 * 这个文件提供了一个简单的日志接口来替换 rclcpp::Logger
 * 允许在非 ROS 环境下编译 TRAC-IK
 */

#ifndef RCLCPP_LOGGER_STUB_HPP
#define RCLCPP_LOGGER_STUB_HPP

#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <string>
#include <cstdint>
#include <memory>
#include <ctime>

// 定义 uint 类型（TRAC-IK 代码中使用）
#ifndef uint
typedef unsigned int uint;
#endif

// 简单的日志级别枚举
enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

// 简单的 Logger 类，模拟 rclcpp::Logger 的接口
class Logger {
public:
    Logger(const std::string& name = "trac_ik") : name_(name) {}
    
    // 自定义拷贝构造函数（因为 stringstream 的拷贝可能有问题）
    Logger(const Logger& other) : name_(other.name_) {
        // stream_ 不需要拷贝，每次使用时会重新创建
    }
    
    Logger& operator=(const Logger& other) {
        if (this != &other) {
            name_ = other.name_;
            // stream_ 不需要拷贝
        }
        return *this;
    }
    
    // 允许移动
    Logger(Logger&&) = default;
    Logger& operator=(Logger&&) = default;
    
    // 获取日志级别名称
    static const char* levelToString(LogLevel level) {
        switch(level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }
    
    // 格式化时间戳
    static std::string getTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        
        // Windows 使用 localtime_s，Linux/Mac 使用 localtime
        #ifdef _WIN32
            struct tm timeinfo;
            localtime_s(&timeinfo, &time);
            ss << std::put_time(&timeinfo, "%Y-%m-%d %H:%M:%S");
        #else
            ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        #endif
        
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }
    
    // 日志输出函数
    void log(LogLevel level, const std::string& message) const {
        std::cerr << "[" << getTimestamp() << "] [" << levelToString(level) 
                  << "] [" << name_ << "] " << message << std::endl;
    }
    
    void debug(const std::string& message) const { log(LogLevel::DEBUG, message); }
    void info(const std::string& message) const { log(LogLevel::INFO, message); }
    void warn(const std::string& message) const { log(LogLevel::WARN, message); }
    void error(const std::string& message) const { log(LogLevel::ERROR, message); }
    void fatal(const std::string& message) const { log(LogLevel::FATAL, message); }
    
    // 支持流式输出
    template<typename T>
    Logger& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }
    
    // 支持 std::endl
    Logger& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(stream_);
        return *this;
    }
    
    // 刷新并输出
    void flush(LogLevel level = LogLevel::INFO) {
        log(level, stream_.str());
        stream_.str("");
        stream_.clear();
    }
    
private:
    std::string name_;
    mutable std::stringstream stream_;
};

// 模拟 rclcpp 命名空间
namespace rclcpp {
    using Logger = ::Logger;
    
    // 获取默认 logger
    inline Logger get_logger(const std::string& name = "trac_ik") {
        return Logger(name);
    }
    
    // 模拟 Node 类
    class Node {
    public:
        Logger get_logger() const {
            return Logger("trac_ik");
        }
    };
    
    using NodeSharedPtr = std::shared_ptr<Node>;
    
    // 前向声明 Time 类
    class Time;
    
    // 模拟 Clock 类
    class Clock {
    public:
        Clock() {}
        
        // 返回当前时间
        Time now() const;
    };
    
    // 模拟 Time 类
    class Time {
    public:
        Time() : nanoseconds_(0) {}
        Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
        Time(const std::chrono::system_clock::time_point& tp) 
            : nanoseconds_(std::chrono::duration_cast<std::chrono::nanoseconds>(
                tp.time_since_epoch()).count()) {}
        
        // 获取秒数（double）
        double seconds() const {
            return static_cast<double>(nanoseconds_) / 1e9;
        }
        
        // 运算符重载
        Time operator-(const Time& other) const {
            return Time(nanoseconds_ - other.nanoseconds_);
        }
        
        bool operator<(const Time& other) const {
            return nanoseconds_ < other.nanoseconds_;
        }
        
        bool operator>(const Time& other) const {
            return nanoseconds_ > other.nanoseconds_;
        }
        
        bool operator==(const Time& other) const {
            return nanoseconds_ == other.nanoseconds_;
        }
        
        bool operator!=(const Time& other) const {
            return nanoseconds_ != other.nanoseconds_;
        }
        
    private:
        int64_t nanoseconds_;
    };
    
    // Clock::now() 的实现
    inline Time Clock::now() const {
        auto now_tp = std::chrono::system_clock::now();
        return Time(now_tp);
    }
}

// 宏定义，模拟 RCLCPP_* 宏
#define RCLCPP_DEBUG(logger, ...) do { \
    std::stringstream ss; \
    ss << __VA_ARGS__; \
    logger.debug(ss.str()); \
} while(0)

#define RCLCPP_DEBUG_STREAM(logger, stream) do { \
    std::stringstream ss; \
    ss << stream; \
    logger.debug(ss.str()); \
} while(0)

#define RCLCPP_INFO(logger, ...) do { \
    std::stringstream ss; \
    ss << __VA_ARGS__; \
    logger.info(ss.str()); \
} while(0)

#define RCLCPP_WARN(logger, ...) do { \
    std::stringstream ss; \
    ss << __VA_ARGS__; \
    logger.warn(ss.str()); \
} while(0)

#define RCLCPP_WARN_THROTTLE(logger, clock, duration, ...) RCLCPP_WARN(logger, __VA_ARGS__)

#define RCLCPP_ERROR(logger, ...) do { \
    std::stringstream ss; \
    ss << __VA_ARGS__; \
    logger.error(ss.str()); \
} while(0)

#define RCLCPP_ERROR_THROTTLE(logger, clock, duration, ...) RCLCPP_ERROR(logger, __VA_ARGS__)

#define RCLCPP_FATAL(logger, ...) do { \
    std::stringstream ss; \
    ss << __VA_ARGS__; \
    logger.fatal(ss.str()); \
} while(0)

#define RCLCPP_FATAL_STREAM(logger, stream) do { \
    std::stringstream ss; \
    ss << stream; \
    logger.fatal(ss.str()); \
} while(0)

#endif // RCLCPP_LOGGER_STUB_HPP
