#include "log.h"

#include <iostream>
#include <stdarg.h>

static int logLevel = LOG_INFO;
static bool useColors = true;

void log(int level, const char* fmt, ...) {

	if (level >= logLevel) {
		if (useColors) {
			std::string prefixStarter = "";

			switch (level) {
				case LOG_DEBUG:
					prefixStarter = "\033[36m"; // cyan
					break;

				case LOG_INFO:
					prefixStarter = "\033[32m"; // green 
					break;

				case LOG_WARN:
					prefixStarter = "\033[33m"; // yellow
					break;

				case LOG_ERROR:
					prefixStarter = "\033[31m"; // red
					break;		

				case LOG_TRACE:
				default:
					prefixStarter = "\033[37m"; // white
					break;	
			}

			std::cout << prefixStarter;
		}

		va_list arg;
		va_start(arg, fmt);
		vprintf(fmt, arg);
		va_end(arg);

		if (useColors) {
			std::cout << "\033[0m"; // ANSI escape for color reset
		}

		std::cout << std::flush;
	}
}

void setLogLevel(int level) {
	logLevel = level;
}

void setColorsEnabled(bool enabled) {
	useColors = enabled;
}