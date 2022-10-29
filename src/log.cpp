#include "Globals.h"
#include "log.h"

#include <iostream>
#include <stdarg.h>

static int logLevel = LOG_INFO;

void log(int level, const char* fmt, ...) {
	va_list arg;
	if (level >= logLevel) {
		va_start(arg, fmt);
		vprintf(fmt, arg);
		va_end(arg);
		std::cout << std::flush;
	}
}

void setLogLevel(int level) {
	logLevel = level;
}