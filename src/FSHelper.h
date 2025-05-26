/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 TheDevMinerTV

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

#ifndef UTILS_FSHELPER_H
#define UTILS_FSHELPER_H

#include <LittleFS.h>
#include <logging/Logger.h>

#include <functional>

namespace SlimeVR::Utils {

class File {
public:
	File(fs::File file)
		: m_File(file) {}

	~File() {
		if (m_File) {
			m_File.close();
		}
	}

	const char* name() const { return m_File.name(); }
	size_t size() const { return m_File.size(); }
	bool isDirectory() { return m_File.isDirectory(); }
	bool seek(size_t pos) { return m_File.seek(pos); }
	bool read(uint8_t* buffer, size_t size) { return m_File.read(buffer, size); }
	bool write(const uint8_t* buffer, size_t size) {
		return m_File.write(buffer, size);
	}
	void close() { return m_File.close(); }

private:
	fs::File m_File;
};

bool ensureDirectory(const char* directory);

File openFile(const char* path, const char* mode);

void forEachFile(const char* directory, std::function<void(File file)> callback);
}  // namespace SlimeVR::Utils

#endif
