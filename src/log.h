
#pragma once

#include <string>

extern int LOG_LEVEL;

extern int LOG_TRACE;
extern int LOG_DEBUG;
extern int LOG_INFO;
extern int LOG_WARN;
extern int LOG_ERROR;

void log(int level, const char *fmt, ...);
