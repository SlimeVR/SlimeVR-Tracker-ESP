#ifndef LOGGING_LOGQUEUE_H
#define LOGGING_LOGQUEUE_H

#include <array>
#include <cstdint>
#include <cstring>
#include <numeric>

namespace SlimeVR::Logging {

template <typename T, T Modulus>
class Modulo {
public:
    Modulo(T value) : m_Value(value) {}

    Modulo<T, Modulus>& operator++()
    {
        m_Value = (m_Value + 1) % Modulus;
        return *this;
    }

    Modulo<T, Modulus> operator+(T other) const
    {
		// WARNING: Does not consider overflow or negative values
		T newValue = (m_Value + other) % Modulus;
        return Modulo<T, Modulus>{newValue};
    }

    T get() const
    {
        return m_Value;
    }

private:
    T m_Value;
};

class LogQueue {
public:
    // Whether there are any messages in the queue.
    bool empty() const
    {
        return m_Count == 0;
    }

    // First message in the queue.
    const char* front() const
    {
        if (empty())
        {
            return nullptr;
        }

        return m_MessageQueue[m_StartIndex.get()].data();
    }

    // Adds a message to the end of the queue. Messages that are too long will be truncated.
    void push(const char* message)
    {
        if (m_Count < m_MessageQueue.max_size())
        {
            setMessageAt(m_Count, message);
            ++m_Count;
        }
        else
        {
            // Overwrite the last message
            setMessageAt(m_Count - 1, OverflowMessage);
        }
    }

    // Removes a message from the front of the queue.
    void pop()
    {
        if (m_Count > 0)
        {
            ++m_StartIndex;
            --m_Count;
        }
    }

    // Global instance of the log queue
    static LogQueue& instance()
    {
        return s_Instance;
    }

private:
    static constexpr int MaxMessages = 100;
    static constexpr int MaxMessageLength = std::numeric_limits<uint8_t>::max();
    static constexpr char OverflowMessage[] = "[OVERFLOW]";

    void setMessageAt(int offset, const char* message)
    {
		Modulo<size_t, MaxMessages> index = m_StartIndex + offset;
        std::array<char, MaxMessageLength>& entry = m_MessageQueue[index.get()];

        std::strncpy(entry.data(), message, entry.max_size());
        entry[entry.max_size() - 1] = '\0';	// NULL terminate string in case message overflows because strncpy does not do that
    }

    Modulo<size_t, MaxMessages> m_StartIndex{0};
    size_t m_Count = 0;
    std::array<std::array<char, MaxMessageLength>, MaxMessages> m_MessageQueue;

    static LogQueue s_Instance;
};

} // namespace SlimeVR::Logging

#endif /* LOGGING_LOGQUEUE_H */
