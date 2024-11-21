#ifndef LOGGING_LEVEL_H

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_FATAL 5

namespace SlimeVR {
namespace Logging {
enum Level {
	TRACE = LOG_LEVEL_TRACE,
	DEBUG = LOG_LEVEL_DEBUG,
	INFO = LOG_LEVEL_INFO,
	WARN = LOG_LEVEL_WARN,
	ERROR = LOG_LEVEL_ERROR,
	FATAL = LOG_LEVEL_FATAL
};

const char* levelToString(Level level);
}  // namespace Logging
}  // namespace SlimeVR

#define LOGGING_LEVEL_H
#endif
