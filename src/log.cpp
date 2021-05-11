
#include <stdarg.h>
#include <iostream>
#include "log.h"

int LOG_LEVEL = LOG_INFO;

int LOG_TRACE = 0;
int LOG_DEBUG = 1;
int LOG_INFO = 2;
int LOG_WARN = 3;
int LOG_ERROR = 4;

void log(int level, const char *fmt, ...)
{
  va_list arg;
  if (level >= LOG_LEVEL)
  {
    va_start(arg, fmt);
    vprintf(fmt, arg);
    va_end(arg);
    std::cout  << std::flush;
  }
}
