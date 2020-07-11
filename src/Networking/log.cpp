
#include "log.h"
#include <iostream>

int LOG_LEVEL = LOG_INFO;

int LOG_DEBUG = 0;
int LOG_INFO = 1;
int LOG_WARN = 2;
int LOG_ERROR = 3;

void log(int level, std::string const &msg)
{
  if (level >= LOG_LEVEL)
  {
    std::cout << msg << std::endl;
  }
}
