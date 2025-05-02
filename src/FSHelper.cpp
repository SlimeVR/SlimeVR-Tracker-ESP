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

#include "./FSHelper.h"

#include <functional>

namespace SlimeVR::Utils {
SlimeVR::Logging::Logger m_Logger("FSHelper");

bool ensureDirectory(const char* directory) {
	if (!LittleFS.exists(directory)) {
		if (!LittleFS.mkdir(directory)) {
			m_Logger.error("Failed to create directory: %s", directory);
			return false;
		}
	}

	auto dir = LittleFS.open(directory, "r");
	auto isDirectory = dir.isDirectory();
	dir.close();

	if (!isDirectory) {
		if (!LittleFS.remove(directory)) {
			m_Logger.error("Failed to remove directory: %s", directory);
			return false;
		}

		if (!LittleFS.mkdir(directory)) {
			m_Logger.error("Failed to create directory: %s", directory);
			return false;
		}
	}

	return true;
}

File openFile(const char* path, const char* mode) {
	return File(LittleFS.open(path, mode));
}

void forEachFile(const char* directory, std::function<void(File file)> callback) {
	if (!ensureDirectory(directory)) {
		return;
	}

#ifdef ESP32
	auto dir = LittleFS.open(directory);
	while (auto f = dir.openNextFile()) {
		if (f.isDirectory()) {
			continue;
		}

		callback(File(f));
	}

	dir.close();
#else
	auto dir = LittleFS.openDir(directory);
	while (dir.next()) {
		auto fd = dir.openFile("r");
		if (!fd.isFile()) {
			continue;
		}

		callback(File(fd));
	}
#endif
}
}  // namespace SlimeVR::Utils
