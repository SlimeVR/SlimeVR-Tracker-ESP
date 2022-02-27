#include "Logger.h"

namespace SlimeVR
{
  namespace Logging
  {
    void Logger::trace(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(TRACE, format, args);
      va_end(args);
    }

    void Logger::debug(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(DEBUG, format, args);
      va_end(args);
    }

    void Logger::info(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(INFO, format, args);
      va_end(args);
    }

    void Logger::warn(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(WARN, format, args);
      va_end(args);
    }

    void Logger::error(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(ERROR, format, args);
      va_end(args);
    }

    void Logger::fatal(const char *format, ...)
    {
      va_list args;
      va_start(args, format);
      log(FATAL, format, args);
      va_end(args);
    }

    void Logger::log(Level level, const char *format, va_list args)
    {
      if (level < LOG_LEVEL)
      {
        return;
      }

      char buffer[256];
      vsnprintf(buffer, 256, format, args);

      Serial.printf("[%-5s] [%s] %s\n", levelToString(level), m_Prefix, buffer);
    }
  }
}
