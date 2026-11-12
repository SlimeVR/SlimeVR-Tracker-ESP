#ifndef LOGGING_LOGGER_H
#define LOGGING_LOGGER_H

#include <Arduino.h>

#include "Level.h"
#include "debug.h"

namespace SlimeVR::Logging {
class Logger {
public:
	Logger(const char* prefix)
		: m_Prefix(prefix)
		, m_Tag(nullptr){};
	Logger(const char* prefix, const char* tag)
		: m_Prefix(prefix)
		, m_Tag(nullptr) {
		setTag(tag);
	};

	~Logger() {
		if (m_Tag != nullptr) {
			free(m_Tag);
		}
	}

	void setTag(const char* tag);

	void trace(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void debug(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void info(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void warn(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void error(const char* str, ...) const __attribute__((format(printf, 2, 3)));
	void fatal(const char* str, ...) const __attribute__((format(printf, 2, 3)));

	static void tick();

	template <typename T>
	void loghex(Level level, const T* data, size_t elements) const {
		if (level < LOG_LEVEL) {
			return;
		}
		const size_t linelength = 80;
		const uint8_t* tData = reinterpret_cast<const uint8_t*>(data);
		size_t tDataSize = sizeof(T) * elements;
		size_t pos = 0;
		char line[linelength + 1];
		size_t linesize = sizeof(line);
		for (size_t i = 0; i < tDataSize; i++) {
			snprintf(
				line + pos,
				linesize - pos,
				"%02X ",
				static_cast<unsigned int>(tData[i])
			);
			pos = pos + 3;
			if ((pos + 1 >= linesize - 1) || (i >= tDataSize - 1)) {
				print(level, "%s", line);
				pos = 0;
			}
		}
	}

private:
	void print(Level level, const char* str, ...) const
		__attribute__((format(printf, 3, 4)));
	void log(Level level, const char* str, va_list args) const;

	const char* const m_Prefix;
	char* m_Tag;
};
}  // namespace SlimeVR::Logging

#endif
