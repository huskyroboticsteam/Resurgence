
#pragma once

#include <string>

constexpr int LOG_TRACE = 0;
constexpr int LOG_DEBUG = 1;
constexpr int LOG_INFO = 2;
constexpr int LOG_WARN = 3;
constexpr int LOG_ERROR = 4;

/**
 * @brief Prints a message with a given log level to log.
 * @param level the log level of the message.
 * @param fmt the printf-style string formatter to specify log message format
 */
void log(int level, const char* fmt, ...);

/**
 * @brief Sets the log level threshold to print log messages at or above 
 * @param level, the log level threshold (LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR) 
 */
void setLogLevel(int level);
