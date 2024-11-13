/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Jabberrock & SlimeVR contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#ifndef LOGGING_LOGQUEUE_H
#define LOGGING_LOGQUEUE_H

#include <array>
#include <cstdint>
#include <cstring>
#include <numeric>

namespace SlimeVR::Logging {

class LogQueue {
public:
	// Whether there are any messages in the queue.
	bool empty() const;

	// First message in the queue.
	const char* front() const;

	// Adds a message to the end of the queue.
	// - Messages that are too long will be truncated.
	// - If the queue is full, the earliest message will be silently dropped
	void push(const char* message);

	// Removes a message from the front of the queue.
	void pop();

	// Global instance of the log queue
	static LogQueue& instance();

private:
	// Sets the message at a particular offset
	void setMessageAt(int offset, const char* message);

	static constexpr size_t MaxMessages = 30;
	static constexpr size_t MaxMessageLength = std::numeric_limits<uint8_t>::max();

	size_t m_StartIndex = 0;
	size_t m_Count = 0;
	std::array<std::array<char, MaxMessageLength>, MaxMessages> m_MessageQueue;

	static LogQueue s_Instance;
};

} // namespace SlimeVR::Logging

#endif /* LOGGING_LOGQUEUE_H */
