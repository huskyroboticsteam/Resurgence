
#include <stdarg.h>
#include <iostream>
#include "log.h"

int LOG_LEVEL = LOG_TRACE;

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
