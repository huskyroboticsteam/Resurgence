#include "Globals.h"
#include "log.h"

#include <iostream>
#include <stdarg.h>

void log(int level, const char* fmt, ...) {
	va_list arg;
	if (level >= Globals::logLevel) {
		va_start(arg, fmt);
		vprintf(fmt, arg);
		va_end(arg);
		std::cout << std::flush;
	}
}
