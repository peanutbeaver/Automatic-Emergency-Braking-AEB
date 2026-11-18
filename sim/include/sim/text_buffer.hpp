#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sim {

/// Minimal string builder for targets without printf. Always NUL-terminated;
/// silently truncates when the buffer is full.
class TextBuffer {
public:
    TextBuffer(char* buffer, size_t capacity);

    TextBuffer& str(const char* text);
    TextBuffer& integer(int32_t value);
    /// Fixed-point decimal, e.g. fixed(3.14159f, 2) -> "3.14". |value| is clamped to 999999.
    TextBuffer& fixed(float value, uint8_t decimals);
    /// Appends spaces until the text is `column` characters long.
    TextBuffer& padTo(size_t column);

    size_t size() const { return length_; }
    const char* c_str() const { return buffer_; }

private:
    void put(char c);

    char* buffer_;
    size_t capacity_;
    size_t length_;
};

}  // namespace sim
