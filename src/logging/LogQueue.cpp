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

#include "LogQueue.h"

namespace SlimeVR::Logging
{

bool LogQueue::empty() const
{
	return m_Count == 0;
}

const char* LogQueue::front() const
{
	if (empty())
	{
		return nullptr;
	}

	return m_MessageQueue[m_StartIndex.get()].data();
}

void LogQueue::push(const char* message)
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

void LogQueue::pop()
{
	if (m_Count > 0)
	{
		++m_StartIndex;
		--m_Count;
	}
}

void LogQueue::setMessageAt(int offset, const char* message)
{
	Modulo<size_t, MaxMessages> index = m_StartIndex + offset;
	std::array<char, MaxMessageLength>& entry = m_MessageQueue[index.get()];

	std::strncpy(entry.data(), message, entry.max_size());
	entry[entry.max_size() - 1] = '\0';	// NULL terminate string in case message overflows because strncpy does not do that
}

// Global instance of the log queue
LogQueue& LogQueue::instance()
{
	return s_Instance;
}

LogQueue LogQueue::s_Instance{};

} // namespace SlimeVR::Logging
