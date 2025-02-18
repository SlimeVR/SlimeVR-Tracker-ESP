#include "Logger.h"

namespace SlimeVR {
namespace Logging {
void Logger::setTag(const char* tag) {
	m_Tag = (char*)malloc(strlen(tag) + 1);
	strcpy(m_Tag, tag);
}

void Logger::trace(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(TRACE, format, args);
	va_end(args);
}

void Logger::debug(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(DEBUG, format, args);
	va_end(args);
}

void Logger::info(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(INFO, format, args);
	va_end(args);
}

void Logger::warn(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(WARN, format, args);
	va_end(args);
}

void Logger::error(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(ERROR, format, args);
	va_end(args);
}

void Logger::fatal(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log(FATAL, format, args);
	va_end(args);
}

void Logger::log(Level level, const char* format, va_list args) {
	if (level < LOG_LEVEL) {
		return;
	}

	char buffer[256];
	vsnprintf(buffer, 256, format, args);

	char buf[strlen(m_Prefix) + (m_Tag == nullptr ? 0 : strlen(m_Tag)) + 2];
	strcpy(buf, m_Prefix);
	if (m_Tag != nullptr) {
		strcat(buf, ":");
		strcat(buf, m_Tag);
	}

	Serial.printf("[%-5s] [%s] %s\n", levelToString(level), buf, buffer);
}
}  // namespace Logging
}  // namespace SlimeVR
