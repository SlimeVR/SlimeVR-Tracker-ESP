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

	template <typename T>
	inline void traceArray(const char* str, const T* array, size_t size) const {
		logArray(TRACE, str, array, size);
	}

	template <typename T>
	inline void debugArray(const char* str, const T* array, size_t size) const {
		logArray(DEBUG, str, array, size);
	}

	template <typename T>
	inline void infoArray(const char* str, const T* array, size_t size) const {
		logArray(INFO, str, array, size);
	}

	template <typename T>
	inline void warnArray(const char* str, const T* array, size_t size) const {
		logArray(WARN, str, array, size);
	}

	template <typename T>
	inline void errorArray(const char* str, const T* array, size_t size) const {
		logArray(ERROR, str, array, size);
	}

	template <typename T>
	inline void fatalArray(const char* str, const T* array, size_t size) const {
		logArray(FATAL, str, array, size);
	}

private:
	void log(Level level, const char* str, va_list args) const;

	template <typename T>
	void logArray(Level level, const char* str, const T* array, size_t size) const {
		if (level < LOG_LEVEL) {
			return;
		}

		char buf[strlen(m_Prefix) + (m_Tag == nullptr ? 0 : strlen(m_Tag)) + 2];
		strcpy(buf, m_Prefix);
		if (m_Tag != nullptr) {
			strcat(buf, ":");
			strcat(buf, m_Tag);
		}

		Serial.printf("[%-5s] [%s] %s", levelToString(level), buf, str);

		for (size_t i = 0; i < size; i++) {
			Serial.print(array[i]);
		}

		Serial.println();
	}

	const char* const m_Prefix;
	char* m_Tag;
};
}  // namespace SlimeVR::Logging

#endif
