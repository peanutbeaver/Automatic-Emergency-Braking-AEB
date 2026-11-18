#include "sim/text_buffer.hpp"

namespace sim {

TextBuffer::TextBuffer(char* buffer, size_t capacity)
    : buffer_(buffer), capacity_(capacity), length_(0U) {
    if (capacity_ > 0U) {
        buffer_[0] = '\0';
    }
}

void TextBuffer::put(char c) {
    if (length_ + 1U < capacity_) {
        buffer_[length_] = c;
        ++length_;
        buffer_[length_] = '\0';
    }
}

TextBuffer& TextBuffer::str(const char* text) {
    while (*text != '\0') {
        put(*text);
        ++text;
    }
    return *this;
}

TextBuffer& TextBuffer::integer(int32_t value) {
    // Work with the magnitude as unsigned so INT32_MIN is handled correctly.
    uint32_t magnitude = static_cast<uint32_t>(value);
    if (value < 0) {
        put('-');
        magnitude = 0U - magnitude;
    }
    char digits[10];
    size_t count = 0U;
    do {
        digits[count] = static_cast<char>('0' + static_cast<char>(magnitude % 10U));
        ++count;
        magnitude /= 10U;
    } while (magnitude != 0U);
    while (count > 0U) {
        --count;
        put(digits[count]);
    }
    return *this;
}

TextBuffer& TextBuffer::fixed(float value, uint8_t decimals) {
    constexpr float kMaxMagnitude = 999999.0f;
    if (decimals > 3U) {
        decimals = 3U;
    }
    uint32_t scale = 1U;
    for (uint8_t i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }

    const bool negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    if (!(magnitude <= kMaxMagnitude)) {  // also catches NaN
        magnitude = kMaxMagnitude;
    }
    const uint32_t scaled = static_cast<uint32_t>(magnitude * static_cast<float>(scale) + 0.5f);
    const uint32_t whole = scaled / scale;

    if (negative && scaled != 0U) {
        put('-');
    }
    integer(static_cast<int32_t>(whole));
    if (decimals > 0U) {
        put('.');
        uint32_t fraction = scaled % scale;
        char digits[3];
        for (uint8_t i = decimals; i > 0U; --i) {
            digits[i - 1U] = static_cast<char>('0' + static_cast<char>(fraction % 10U));
            fraction /= 10U;
        }
        for (uint8_t i = 0U; i < decimals; ++i) {
            put(digits[i]);
        }
    }
    return *this;
}

TextBuffer& TextBuffer::padTo(size_t column) {
    while (length_ < column && length_ + 1U < capacity_) {
        put(' ');
    }
    return *this;
}

}  // namespace sim
