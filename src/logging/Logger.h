#ifndef LOGGING_LOGGER_H
#define LOGGING_LOGGER_H

#include "Level.h"
#include "debug.h"
#include <Arduino.h>

namespace SlimeVR
{
  namespace Logging
  {
    class Logger
    {
    public:
      Logger(const char *prefix) : m_Prefix(prefix){};

      void trace(const char *str, ...);
      void debug(const char *str, ...);
      void info(const char *str, ...);
      void warn(const char *str, ...);
      void error(const char *str, ...);
      void fatal(const char *str, ...);

      template <typename T>
      inline void traceArray(const char *str, const T *array, int size)
      {
        logArray(TRACE, str, array, size);
      }

      template <typename T>
      inline void debugArray(const char *str, const T *array, int size)
      {
        logArray(DEBUG, str, array, size);
      }

      template <typename T>
      inline void infoArray(const char *str, const T *array, int size)
      {
        logArray(INFO, str, array, size);
      }

      template <typename T>
      inline void warnArray(const char *str, const T *array, int size)
      {
        logArray(WARN, str, array, size);
      }

      template <typename T>
      inline void errorArray(const char *str, const T *array, int size)
      {
        logArray(ERROR, str, array, size);
      }

      template <typename T>
      inline void fatalArray(const char *str, const T *array, int size)
      {
        logArray(FATAL, str, array, size);
      }

    private:
      void log(Level level, const char *str, va_list args);

      template <typename T>
      void logArray(Level level, const char *str, const T *array, int size)
      {
        if (level < LOG_LEVEL)
        {
          return;
        }

        Serial.printf("[%-5s] [%s] %s", levelToString(level), m_Prefix, str);

        for (size_t i = 0; i < size; i++)
        {
          Serial.print(array[i]);
        }

        Serial.println();
      }

      const char *m_Prefix;
    };
  }
}

#endif
