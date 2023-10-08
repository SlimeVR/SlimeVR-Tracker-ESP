/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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

#ifndef SLIMEVR_SERIALCOMMANDS_H_
#define SLIMEVR_SERIALCOMMANDS_H_

namespace SerialCommands {
/* lengthCheck:
 *   Description:
 *     Checks the length of a text (const char *) and to length. If the length is > than \0 terminated text, it prints a ERROR and returns false
 *     If (the pointer is Null) and (the text is smaller than length) it returns true.
 *   Parameters:
 *     text - Pointer to input const char*
 *     length - How long the text max should be
 *     cmd - Begin of the error text
 *     name - How the text should be Called in the error Message
 *   Returns:
 *     true when size is smaller or equal than lenght or pointer = NULL
 *     false when size is bigger than length error "%s ERROR: %s is longer than %d bytes / Characters"
 */
    bool lengthCheck (const char* text, unsigned int length, const char* cmd, const char* name);
	void setUp();
    void update();
    void printState();
}

#endif // SLIMEVR_SERIALCOMMANDS_H_
