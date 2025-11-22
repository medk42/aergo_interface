#pragma once

#include <iostream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>

// ----------------------------------------------
// Format timestamp
// ----------------------------------------------
inline std::string currentTimestamp()
{
    using namespace std::chrono;

    const auto now      = system_clock::now();
    const auto epoch_ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
    const auto ms       = epoch_ms % 1000;

    std::time_t t = system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&t), "%Y/%m/%d %H:%M:%S.")
        << std::setw(3) << std::setfill('0') << ms;

    return oss.str();
}

// ----------------------------------------------
// Core logging function
// ----------------------------------------------
inline void logMessage(
        const char* level,
        const char* file,
        int         line,
        const char* func,
        std::ostringstream&& msg)
{
    std::cout << "[" << level << "] "
              << "[" << file << ":" << line << " " << func << "] "
              << currentTimestamp() << " — "
              << msg.str()
              << std::endl;
}

// ----------------------------------------------
// User-facing macros (tiny, clean)
// ----------------------------------------------
#define LOG_INFO(msg)  do { std::ostringstream _os; _os << msg; \
    logMessage("INFO",  __FILE__, __LINE__, __func__, std::move(_os)); } while(0)

#define LOG_WARN(msg)  do { std::ostringstream _os; _os << msg; \
    logMessage("WARN",  __FILE__, __LINE__, __func__, std::move(_os)); } while(0)

#define LOG_ERR(msg)   do { std::ostringstream _os; _os << msg; \
    logMessage("ERROR", __FILE__, __LINE__, __func__, std::move(_os)); } while(0)
