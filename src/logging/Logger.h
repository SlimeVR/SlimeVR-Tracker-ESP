#ifndef LOGGING_LOGGER_H
#define LOGGING_LOGGER_H

#include <Arduino.h>

#include <optional>
#include <string>

#include "Level.h"
#include "debug.h"

namespace SlimeVR::Logging {

template <typename IDType>
class Logger {
public:
	explicit Logger(const char* prefix, const char* identifier)
		: m_Prefix(prefix)
		, m_Identifier(identifier){};
	Logger(const char* prefix, const char* identifier, const char* tag)
		: m_Prefix(prefix)
		, m_Identifier(identifier)
		, m_Tag(tag){};

	void setTag(const char* tag) { m_Tag = tag; }

	void trace(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, TRACE, format, args);
		va_end(args);
	}

	void debug(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, DEBUG, format, args);
		va_end(args);
	}

	void info(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, INFO, format, args);
		va_end(args);
	}

	void warn(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, WARN, format, args);
		va_end(args);
	}

	void error(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, ERROR, format, args);
		va_end(args);
	}

	void fatal(IDType id, const char* format, ...) const
		__attribute__((format(printf, 3, 4))) {
		va_list args;
		va_start(args, format);
		log(id, FATAL, format, args);
		va_end(args);
	}

	template <typename T>
	inline void traceArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, Level::TRACE, str, array, size);
	}

	template <typename T>
	inline void debugArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, DEBUG, str, array, size);
	}

	template <typename T>
	inline void infoArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, INFO, str, array, size);
	}

	template <typename T>
	inline void warnArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, WARN, str, array, size);
	}

	template <typename T>
	inline void errorArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, ERROR, str, array, size);
	}

	template <typename T>
	inline void fatalArray(IDType id, const char* str, const T* array, size_t size)
		const {
		logArray(id, FATAL, str, array, size);
	}

private:
	void log(IDType id, Level level, const char* format, va_list args) const {
		if (level < LOG_LEVEL) {
			return;
		}

		printPrefix(id, level);
		vprintf(format, args);
	}

	template <typename T>
	void logArray(IDType id, Level level, const char* str, const T* array, size_t size)
		const {
		if (level < LOG_LEVEL) {
			return;
		}

		printPrefix(id, level);
		Serial.print(str);

		for (size_t i = 0; i < size; i++) {
			Serial.print(array[i]);
		}

		Serial.println();
	}

	void printPrefix(IDType id, Level level) const {
		Serial.printf(
			"{%s-%d} [%-5s] [%s",
			m_Identifier,
			static_cast<uint32_t>(id),
			levelToString(level),
			m_Prefix
		);

		if (m_Tag) {
			Serial.printf(":%s", m_Tag->c_str());
		}

		Serial.printf("] ");
	}

	const char* const m_Prefix;
	const char* const m_Identifier;
	std::optional<std::string> m_Tag;
};
}  // namespace SlimeVR::Logging

#endif
