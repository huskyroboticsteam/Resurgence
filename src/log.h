
#pragma once

#include <string>

constexpr int LOG_TRACE = 0;
constexpr int LOG_DEBUG = 1;
constexpr int LOG_INFO = 2;
constexpr int LOG_WARN = 3;
constexpr int LOG_ERROR = 4;

void log(int level, const char* fmt, ...);
